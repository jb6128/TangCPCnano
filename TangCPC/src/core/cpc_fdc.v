//============================================================================
// cpc_fdc.v -- uPD765 floppy controller of the CPC, glued to the SD card layer
//
//   Z80 side (Amstrad CPC I/O map, partial decoding on A10, A8, A7):
//     &FA7E  write   motor on/off (D0), both drives share one motor line
//     &FB7E  read    FDC main status register      (A0 = 0)
//     &FB7F  read/write  FDC data register         (A0 = 1)
//       FDC   : A10 = 0, A8 = 1, A7 = 0
//       motor : A10 = 0, A8 = 0, A7 = 0
//
//   SD side: the u765 requests 512 byte sectors of the mounted .dsk image.
//   The request signals (sd_rd/sd_wr/sd_lba) go to the sd_card layer as
//   sources 0 (drive A) and 1 (drive B); top.sv routes sd_ack and the sector
//   buffer strobes back only while the transfer belongs to source 0 or 1.
//
//   The FDC runs from a 4 MHz clock enable derived from the 32 MHz clk, which
//   is what u765's CYCLES = 4000 (cycles per millisecond) assumes.
//============================================================================
`default_nettype none

module cpc_fdc (
    input  wire        clk,            // clk32
    input  wire        reset,          // active high

    // Z80 bus
    input  wire        iorq,
    input  wire        rd,
    input  wire        wr,
    input  wire [15:0] addr,
    input  wire  [7:0] din,            // data from the Z80
    output wire  [7:0] dout,           // 8'hFF unless the FDC is being read

    // configuration (OSD)
    input  wire  [1:0] wprot,          // write protect A / B
    input  wire        fast,           // fast FDC mode

    // image info from sd_card (valid while the matching img_mounted bit is 1)
    input  wire  [1:0] img_mounted,
    input  wire [31:0] img_size,

    // sector requests to sd_card, sources 0 and 1
    output wire [31:0] sd_lba,
    output wire  [1:0] sd_rd,
    output wire  [1:0] sd_wr,
    input  wire        sd_ack,         // high while a transfer of source 0/1 runs
    input  wire  [8:0] sd_buff_addr,
    input  wire  [7:0] sd_buff_dout,   // sector bytes read from the SD card
    input  wire        sd_buff_wr,
    output wire  [7:0] sd_buff_din,    // sector bytes to write to the SD card

    output wire  [1:0] motor_o,
    output wire  [1:0] ready_o,

    // debug (LEDs): sticky activity flags and the last main-status value read
    output reg   [5:0] dbg_flags,
    output reg   [7:0] dbg_msr,
    output wire [10:0] dbg_scan,     // u765 image ready / scan state (mount replay gate in top.sv)
    output wire [339:0] log_rec      // TEMPORARY: record ports for dbg_uart_log (4 x 85 bit)
);

    //------------------------------------------------------------------------
    // address decoding
    //------------------------------------------------------------------------
    wire io_rd    = iorq & rd;
    wire io_wr    = iorq & wr;
    wire fdc_cs   = ~addr[10] &  addr[8] & ~addr[7];
    wire motor_cs = ~addr[10] & ~addr[8] & ~addr[7];

    reg  [1:0] motor;
    always @(posedge clk) begin
        if (reset)                  motor <= 2'b00;
        else if (io_wr & motor_cs)  motor <= {2{din[0]}};
    end

    //------------------------------------------------------------------------
    // 4 MHz clock enable
    //------------------------------------------------------------------------
    reg [2:0] ce_cnt;
    always @(posedge clk) ce_cnt <= ce_cnt + 3'd1;
    wire ce = (ce_cnt == 3'd0);

    //------------------------------------------------------------------------
    // drive ready: a disk is in the drive once an image of non-zero size was
    // mounted; mounting an empty selection ejects it
    //------------------------------------------------------------------------
    reg [1:0] ready;
    always @(posedge clk) begin
        if (img_mounted[0]) ready[0] <= |img_size;
        if (img_mounted[1]) ready[1] <= |img_size;
    end
    initial ready = 2'b00;

    // write protect is latched by the u765 at the img_mounted edge
    wire img_wp = (img_mounted[0] & wprot[0]) | (img_mounted[1] & wprot[1]);

    //------------------------------------------------------------------------
    // the controller
    //------------------------------------------------------------------------
    wire [7:0] fdc_dout;
    wire [31:0] dbg_trk;       // TEMPORARY: u765 present cylinder / sectors per track (UART log)

    u765 #(.CYCLES(20'd4000), .SPECCY_SPEEDLOCK_HACK(0)) u765_inst (
        .clk_sys      ( clk                       ),
        .ce           ( ce                        ),
        .reset        ( reset                     ),
        .ready        ( ready                     ),
        .motor        ( motor                     ),
        .available    ( 2'b11                     ),
        .fast         ( fast                      ),
        .a0           ( addr[0]                   ),
        .nRD          ( ~(io_rd & fdc_cs)         ),
        .nWR          ( ~(io_wr & fdc_cs)         ),
        .din          ( din                       ),
        .dout         ( fdc_dout                  ),

        .img_mounted  ( img_mounted               ),
        .img_wp       ( img_wp                    ),
        .img_size     ( img_size                  ),
        .sd_lba       ( sd_lba                    ),
        .sd_rd        ( sd_rd                     ),
        .sd_wr        ( sd_wr                     ),
        .sd_ack       ( sd_ack                    ),
        .sd_buff_addr ( sd_buff_addr              ),
        .sd_buff_dout ( sd_buff_dout              ),
        .sd_buff_din  ( sd_buff_din               ),
        .sd_buff_wr   ( sd_buff_wr                ),
        .dbg_trk      ( dbg_trk                   ),
        .dbg_scan     ( dbg_scan                  )
    );

    // flags: 0 any FDC access, 1 MSR read, 2 data write (command bytes),
    //        3 data read (result bytes), 4 motor port write, 5 SD request from the FDC
    wire acc_rd = io_rd & fdc_cs;
    wire acc_wr = io_wr & fdc_cs;
    always @(posedge clk) begin
        if (reset) begin
            dbg_flags <= 6'd0;       // dbg_msr is deliberately kept: it is shown while S2 (reset) is held
        end else begin
            if (acc_rd | acc_wr)        dbg_flags[0] <= 1'b1;
            if (acc_rd & ~addr[0]) begin dbg_flags[1] <= 1'b1; dbg_msr <= fdc_dout; end
            if (acc_wr &  addr[0])      dbg_flags[2] <= 1'b1;
            if (acc_rd &  addr[0])      dbg_flags[3] <= 1'b1;
            if (io_wr & motor_cs)       dbg_flags[4] <= 1'b1;
            if ((|sd_rd) | (|sd_wr))    dbg_flags[5] <= 1'b1;
        end
    end


    // -----------------------------------------------------------------------------------------------
    // TEMPORARY debug log records for dbg_uart_log (see the end of this file and doc 2.8). Four record ports:
    //   p0: 'C' = one complete FDC command (command byte + parameters, length from the command table),
    //       'M' = write to the motor port (&FA7E)
    //   p1: 'R' = result of the last command: number of data register reads (2 bytes, big endian) + the last 7 bytes read
    //       (for Read Data this is the 7 result bytes; ST0 ST1 ST2 C H R N); flushed when the next command starts or after a pause;
    //       'Z' = change of the controller reset
    //   p2: 'Q' / 'W' = SD sector read / write request of the u765: drive mask, LBA (4 bytes, big endian)
    //   p3: 'S' = change of the u765 state: scan byte 0 {watchdog seen, dirty B, dirty A}, byte 1 {ready B, ready A, scan B (2),
    //       scan A (2), scan lock, mount busy}, sectors of the current track A, B, present cylinder A, B
    // Record port layout (85 bit): [0] valid (one clock), [8:1] tag, [12:9] length, [84:13] data (byte 0 = [20:13] ... max 9 bytes)
    // -----------------------------------------------------------------------------------------------
    function [3:0] cmd_len(input [7:0] c);
        case (c[4:0])
            5'h02, 5'h05, 5'h06, 5'h09, 5'h0C, 5'h11, 5'h19, 5'h1D: cmd_len = 4'd9;
            5'h0D:                         cmd_len = 4'd6;
            5'h03, 5'h0F:                  cmd_len = 4'd3;
            5'h04, 5'h07, 5'h0A:           cmd_len = 4'd2;
            default:                       cmd_len = 4'd1;
        endcase
    endfunction

    reg         l_rd_q = 1'b0, l_wr_q = 1'b0, l_mot_q = 1'b0, l_rd_a0 = 1'b0, l_reset_q = 1'b0;
    reg   [7:0] l_dout = 8'h00;
    reg  [71:0] wbuf = 72'd0;
    reg   [3:0] wcnt = 4'd0, wexp = 4'd1;
    reg  [55:0] rbuf = 56'd0;
    reg  [15:0] rcnt = 16'd0;
    reg  [20:0] l_idle = 21'd0;
    reg  [71:0] wtmp;
    reg         p0_v = 1'b0, p1_v = 1'b0, p2_v = 1'b0, p3_v = 1'b0;
    reg   [7:0] p0_tag = 8'h00, p1_tag = 8'h00, p2_tag = 8'h00, p3_tag = 8'h00;
    reg   [3:0] p0_len = 4'd0,  p1_len = 4'd0,  p2_len = 4'd0,  p3_len = 4'd0;
    reg  [71:0] p0_dat = 72'd0, p1_dat = 72'd0, p2_dat = 72'd0, p3_dat = 72'd0;
    reg   [1:0] l_sd_q = 2'b00;
    reg  [10:0] l_scan_q = 11'd0;
    reg  [31:0] l_trk_q = 32'd0;

    always @(posedge clk) begin
        l_rd_q    <= acc_rd;
        l_wr_q    <= acc_wr;
        l_mot_q   <= io_wr & motor_cs;
        l_reset_q <= reset;
        l_sd_q    <= {|sd_wr, |sd_rd};
        p0_v <= 1'b0; p1_v <= 1'b0; p2_v <= 1'b0; p3_v <= 1'b0;

        if (acc_rd) begin l_dout <= fdc_dout; l_rd_a0 <= addr[0]; end

        // pause detector: data register accesses only (the main status is polled all the time)
        if ((acc_rd | acc_wr) & addr[0]) l_idle <= 21'd0;
        else if (!l_idle[20])            l_idle <= l_idle + 21'd1;

        // result bytes: the value presented at the end of a data register read
        if (~acc_rd & l_rd_q & l_rd_a0) begin
            rbuf <= {l_dout, rbuf[55:8]};
            if (rcnt != 16'hFFFF) rcnt <= rcnt + 16'd1;
        end

        // flush the results after a pause
        if (l_idle[20] && rcnt != 16'd0) begin
            p1_v <= 1'b1; p1_tag <= "R"; p1_len <= 4'd9; p1_dat <= {rbuf, rcnt[7:0], rcnt[15:8]};
            rcnt <= 16'd0;
        end

        // motor port
        if ((io_wr & motor_cs) & ~l_mot_q) begin
            p0_v <= 1'b1; p0_tag <= "M"; p0_len <= 4'd1; p0_dat <= {64'd0, din};
        end

        // command bytes (data register writes)
        if (acc_wr & ~l_wr_q & addr[0]) begin
            if (wcnt == 4'd0) begin
                // first byte of a command: the results of the previous command are complete
                if (rcnt != 16'd0) begin
                    p1_v <= 1'b1; p1_tag <= "R"; p1_len <= 4'd9; p1_dat <= {rbuf, rcnt[7:0], rcnt[15:8]};
                    rcnt <= 16'd0;
                end
                wbuf <= {64'd0, din};
                wexp <= cmd_len(din);
                if (cmd_len(din) == 4'd1) begin
                    p0_v <= 1'b1; p0_tag <= "C"; p0_len <= 4'd1; p0_dat <= {64'd0, din};
                end else
                    wcnt <= 4'd1;
            end else begin
                wtmp = wbuf;
                wtmp[wcnt*8 +: 8] = din;
                wbuf <= wtmp;
                if (wcnt + 4'd1 == wexp) begin
                    p0_v <= 1'b1; p0_tag <= "C"; p0_len <= wexp; p0_dat <= wtmp;
                    wcnt <= 4'd0;
                end else
                    wcnt <= wcnt + 4'd1;
            end
        end

        // SD requests of the u765
        if (|sd_rd & ~l_sd_q[0]) begin
            p2_v <= 1'b1; p2_tag <= "Q"; p2_len <= 4'd5;
            p2_dat <= {32'd0, sd_lba[7:0], sd_lba[15:8], sd_lba[23:16], sd_lba[31:24], 6'd0, sd_rd};
        end else if (|sd_wr & ~l_sd_q[1]) begin
            p2_v <= 1'b1; p2_tag <= "W"; p2_len <= 4'd5;
            p2_dat <= {32'd0, sd_lba[7:0], sd_lba[15:8], sd_lba[23:16], sd_lba[31:24], 6'd0, sd_wr};
        end

        // u765 state changes
        // (scan state / lock / busy bits [5:0] do not trigger a line, they toggle for every header byte during a scan)
        if ({dbg_scan[10:6], dbg_trk} != {l_scan_q[10:6], l_trk_q}) begin
            l_scan_q <= dbg_scan; l_trk_q <= dbg_trk;
            p3_v <= 1'b1; p3_tag <= "S"; p3_len <= 4'd6;
            p3_dat <= {24'd0, dbg_trk[31:24], dbg_trk[23:16], dbg_trk[15:8], dbg_trk[7:0], dbg_scan[7:0], 5'd0, dbg_scan[10:8]};
        end

        // controller reset: forget a half written command / unread results
        if (reset != l_reset_q) begin
            p1_v <= 1'b1; p1_tag <= "Z"; p1_len <= 4'd1; p1_dat <= {71'd0, reset};
        end
        if (reset) begin wcnt <= 4'd0; rcnt <= 16'd0; end
    end

    assign log_rec = { p3_dat, p3_len, p3_tag, p3_v,
                       p2_dat, p2_len, p2_tag, p2_v,
                       p1_dat, p1_len, p1_tag, p1_v,
                       p0_dat, p0_len, p0_tag, p0_v };

    assign dout    = (io_rd & fdc_cs) ? fdc_dout : 8'hFF;
    assign motor_o = motor;
    assign ready_o = ready;

endmodule

//============================================================================
// dbg_uart_log -- TEMPORARY debug text log on a UART pin (disk A + B problem, see doc 2.8).
//   Several record ports (85 bit each, layout see cpc_fdc.v) -> one line per record:
//       <tag> <time in ms, 6 hex digits> <data bytes, hex, each followed by a blank> CR LF
//   Each port latches one record until it is formatted; a record that arrives while the port is still
//   occupied, is dropped and an "!" line is written. "B" = banner after power-on. 115200 8N1, 2 KB FIFO.
//============================================================================
module dbg_uart_log #(
    parameter CLK_FREQ = 32_000_000,
    parameter BAUD     = 115200,
    parameter NP       = 6
)(
    input  wire             clk,
    input  wire [NP*85-1:0] rec,
    output wire             tx
);
    localparam NQ  = NP + 2;               // ports + banner + overflow marker
    localparam DIV = CLK_FREQ / BAUD;

    // time base: milliseconds since power-on
    reg [14:0] mscnt = 15'd0;
    reg [23:0] ms    = 24'd0;
    always @(posedge clk) begin
        if (mscnt == CLK_FREQ/1000 - 1) begin mscnt <= 15'd0; ms <= ms + 24'd1; end
        else mscnt <= mscnt + 15'd1;
    end

    // character FIFO (written by the formatter, read by the UART)
    reg  [7:0]  mem [0:2047];
    reg  [10:0] wp = 11'd0, rp = 11'd0;
    reg  [7:0]  rdata = 8'h00;
    wire        full = ((wp + 11'd1) == rp);
    always @(posedge clk) rdata <= mem[rp];

    function [7:0] hexch(input [3:0] n);
        hexch = (n < 4'd10) ? (8'h30 + {4'b0000, n}) : (8'h37 + {4'b0000, n});
    endfunction

    // record latches and formatter
    reg [NQ-1:0]    pend   = ({{(NQ-1){1'b0}}, 1'b1} << NP);   // the banner is pending at power-on
    reg [NP*8-1:0]  q_tag  = {(NP*8){1'b0}};
    reg [NP*4-1:0]  q_len  = {(NP*4){1'b0}};
    reg [NP*72-1:0] q_data = {(NP*72){1'b0}};
    reg [NP*24-1:0] q_time = {(NP*24){1'b0}};
    reg             ovf_req = 1'b0;

    reg        busy = 1'b0;
    reg  [3:0] ph = 4'd0;
    reg  [2:0] cnt = 3'd0;
    reg  [3:0] bidx = 4'd0;
    reg  [7:0] f_tag = 8'h00;
    reg  [3:0] f_len = 4'd0;
    reg [71:0] f_data = 72'd0;
    reg [23:0] f_time = 24'd0;
    reg  [7:0] c;
    reg [23:0] tsh;
    reg [71:0] dsh;
    reg        put;
    integer    i, sel;

    always @(posedge clk) begin
        // overflow marker request -> pending line (before the latch loop, so that a new overflow wins)
        if (ovf_req && !pend[NP+1]) begin pend[NP+1] <= 1'b1; ovf_req <= 1'b0; end

        // latch the records of the ports
        for (i = 0; i < NP; i = i + 1) begin
            if (rec[i*85]) begin
                if (!pend[i]) begin
                    pend[i]                <= 1'b1;
                    q_tag [i*8  +:  8]     <= rec[i*85+1  +:  8];
                    q_len [i*4  +:  4]     <= rec[i*85+9  +:  4];
                    q_data[i*72 +: 72]     <= rec[i*85+13 +: 72];
                    q_time[i*24 +: 24]     <= ms;
                end else
                    ovf_req <= 1'b1;
            end
        end

        if (!busy) begin
            // take the pending record with the lowest number
            sel = NQ;
            for (i = NQ-1; i >= 0; i = i - 1) if (pend[i]) sel = i;
            if (sel < NQ) begin
                busy <= 1'b1;
                ph   <= 4'd0;
                pend[sel] <= 1'b0;
                if (sel < NP) begin
                    f_tag  <= q_tag [sel*8  +:  8];
                    f_len  <= q_len [sel*4  +:  4];
                    f_data <= q_data[sel*72 +: 72];
                    f_time <= q_time[sel*24 +: 24];
                end else begin
                    f_tag  <= (sel == NP) ? "B" : "!";
                    f_len  <= 4'd0;
                    f_data <= 72'd0;
                    f_time <= ms;
                end
            end
        end else if (!full) begin
            put = 1'b1;
            c   = 8'h20;
            case (ph)
                4'd0: begin c = f_tag; ph <= 4'd1; end
                4'd1: begin c = 8'h20; ph <= 4'd2; cnt <= 3'd0; end
                4'd2: begin
                    tsh = f_time >> (20 - 4*cnt);
                    c   = hexch(tsh[3:0]);
                    if (cnt == 3'd5) ph <= 4'd3; else cnt <= cnt + 3'd1;
                end
                4'd3: begin
                    c = 8'h20;
                    if (f_len == 4'd0) ph <= 4'd7;
                    else begin ph <= 4'd4; bidx <= 4'd0; end
                end
                4'd4: begin dsh = f_data >> (8*bidx); c = hexch(dsh[7:4]); ph <= 4'd5; end
                4'd5: begin dsh = f_data >> (8*bidx); c = hexch(dsh[3:0]); ph <= 4'd6; end
                4'd6: begin
                    c = 8'h20;
                    if (bidx + 4'd1 == f_len) ph <= 4'd7;
                    else begin bidx <= bidx + 4'd1; ph <= 4'd4; end
                end
                4'd7: begin c = 8'h0D; ph <= 4'd8; end
                4'd8: begin c = 8'h0A; busy <= 1'b0; end
                default: begin put = 1'b0; busy <= 1'b0; end
            endcase
            if (put) begin mem[wp] <= c; wp <= wp + 11'd1; end
        end
    end

    // UART transmitter: 8N1, LSB first
    reg [1:0] fs   = 2'd0;
    reg [8:0] bdiv = 9'd0;
    reg [9:0] sh   = 10'h3FF;
    reg [3:0] nb   = 4'd0;
    reg       txo  = 1'b1;
    always @(posedge clk) begin
        case (fs)
            2'd0: if (rp != wp) fs <= 2'd1;
            2'd1: begin
                sh   <= {1'b1, rdata, 1'b0};
                nb   <= 4'd0;
                bdiv <= 9'd0;
                rp   <= rp + 11'd1;
                fs   <= 2'd2;
            end
            default: begin
                if (bdiv == DIV - 1) begin
                    bdiv <= 9'd0;
                    txo  <= sh[0];
                    sh   <= {1'b1, sh[9:1]};
                    if (nb == 4'd9) fs <= 2'd0; else nb <= nb + 4'd1;
                end else
                    bdiv <= bdiv + 9'd1;
            end
        endcase
    end
    assign tx = txo;
endmodule

`default_nettype wire

//============================================================================
// cpc_taprec.v -- CPC tape recorder (SAVE) into the mounted CDT/TZX image
//
//   The CPC writes the tape by toggling PPI port C bit 5 (tape_out) while the
//   tape motor (port C bit 4) is on. This block timestamps every edge, converts the
//   interval to 3.5 MHz T-states (clk32 cycles * 7/64, the unit of the CDT format)
//   and stores it as TZX blocks into the image, which has to be big enough
//   (a blank image is made by tools/make_blank_tape.py; the file cannot grow):
//
//     0x13 pulse sequence  [0x13][count 1..255][T-states lo, hi] * count
//     0x20 pause           [0x20][ms lo, ms hi]    (interval >= 65536 T-states = 18.7 ms)
//     end marker           [0x2A 00 00 00 00][0x00]  (0x00 is no valid block ID: end of tape for cpc_tape)
//
//   Recording starts at the tape position given by cpc_tape (`snap`, sampled when
//   the motor is switched on), but only with the first edge of tape_out: a motor
//   cycle that does not write (LOAD, CAT) never touches the image. The first edge
//   only starts the time measurement, so a first partial interval is not stored.
//
//   The bytes are collected in a 16 KiB ring buffer (32 sectors) and written to the SD
//   card sector by sector (image index 2, requester `rec` of sd_arb). The sector the
//   recording starts in is read first (read-modify-write) unless the start is sector
//   aligned. The count byte of the open 0x13 block is patched when the block is
//   closed, so sectors containing it are not flushed before.
//
//   At the end (motor off) the end marker is appended, the rest of the sector is
//   zero filled, the sectors are flushed and `done` is pulsed with `new_pos` = file
//   offset of the end marker, where cpc_tape repositions the player (a following
//   SAVE continues there, a LOAD finds the end of the tape). Errors (image full,
//   SD too slow = ring overrun) abort the recording, set `err` and report done with
//   new_pos = start of the recording.
//============================================================================
`default_nettype none

module cpc_taprec (
    input  wire        clk,          // clk32
    input  wire        reset,        // active high

    input  wire        enable,       // OSD "Tape write"
    input  wire        present,      // image mounted
    input  wire        img_mounted,  // pulse: image 2 (re)mounted
    input  wire [31:0] img_size,
    input  wire        motor,        // tape motor (PPI C4, gated while the CPC is in reset)
    input  wire        tape_out,     // PPI C5
    input  wire [31:0] snap,         // tape position (file offset) for a recording starting now

    output reg         active,       // recording in progress: the player must be frozen
    output reg         done,         // 1 clk pulse: recording finished, new_pos valid
    output reg  [31:0] new_pos,
    output reg   [2:0] err,          // error code, sticky until the next recording / mount: 1 start beyond end of image, 2 edge overrun, 3 SD too slow, 4 write beyond end of image, 5 flush position beyond write position, 6 SD request never started, 7 SD transfer never ended

    // SD requests (sd_arb requester `rec`, image index 2)
    output reg         sd_rd,
    output reg         sd_wr,
    output reg  [31:0] sd_lba,
    input  wire        sd_busy,      // granted to us and SD busy
    input  wire  [8:0] sd_addr,      // sector byte address (read data in / write data out)
    input  wire  [7:0] sd_din,
    input  wire        sd_din_wr,
    output reg   [7:0] sd_dout       // 1 clk read latency, like the FDC buffer
);
    //------------------------------------------------------------------------
    // image
    //------------------------------------------------------------------------
    reg  [31:0] size_r;
    wire [22:0] tsec = size_r[31:9] + (|size_r[8:0] ? 23'd1 : 23'd0);

    //------------------------------------------------------------------------
    // ring buffer: file offset [RSB+8:0] is the ring address
    //------------------------------------------------------------------------
    localparam RSB = 5;                  // ring size 2**RSB sectors (32 sectors = 16 KiB: absorbs a slow SD card)
    reg [7:0] rbuf [0:(1<<(RSB+9))-1];
    reg       rb_we;
    reg [RSB+8:0] rb_wa;
    reg [7:0]  rb_wd;
    reg        pre_busy;                 // sector preload in flight: SD data goes to the ring
    reg [22:0] fl_sec;                   // next sector to write to the SD card
    always @(posedge clk) begin
        if (pre_busy && sd_din_wr) rbuf[{fl_sec[RSB-1:0], sd_addr}] <= sd_din;
        else if (rb_we)            rbuf[rb_wa] <= rb_wd;
        sd_dout <= rbuf[{fl_sec[RSB-1:0], sd_addr}];
    end

    //------------------------------------------------------------------------
    // SD request engine: one read (preload) or write (flush) at a time
    //------------------------------------------------------------------------
    localparam Q_IDLE = 2'd0, Q_REQ = 2'd1, Q_WAIT = 2'd2;
    reg [1:0] qs;
    reg       q_is_wr;
    reg       q_go;                      // start a request (set by the main FSM, one clk)
    reg       q_go_wr;
    reg       q_done;                    // 1 clk pulse: request finished
    reg [31:0] q_lba;
    reg [25:0] qt;                       // request timeout counter
    reg        q_fail;                   // 1 clk pulse: the SD request never started (code 6) or never ended (code 7)
    reg        q_fail_wait;

    // TZX/CDT file header "ZXTape!" 1A 01 0A: checked while sector 0 is preloaded, written if it is missing
    function [7:0] hdr_byte(input [3:0] i);
        case (i)
            4'd0: hdr_byte = 8'h5A; 4'd1: hdr_byte = 8'h58; 4'd2: hdr_byte = 8'h54; 4'd3: hdr_byte = 8'h61;
            4'd4: hdr_byte = 8'h70; 4'd5: hdr_byte = 8'h65; 4'd6: hdr_byte = 8'h21; 4'd7: hdr_byte = 8'h1A;
            4'd8: hdr_byte = 8'h01; default: hdr_byte = 8'h0A;
        endcase
    endfunction
    reg        hdr_bad;
    reg  [3:0] hc;
    always @(posedge clk) begin
        if (q_go && !q_go_wr) hdr_bad <= 1'b0;
        else if (pre_busy && sd_din_wr && sd_addr < 9'd10 && sd_din != hdr_byte(sd_addr[3:0])) hdr_bad <= 1'b1;
    end

    always @(posedge clk) begin
        q_done <= 1'b0;
        q_fail <= 1'b0;
        if (reset) begin
            qs <= Q_IDLE; sd_rd <= 1'b0; sd_wr <= 1'b0; sd_lba <= 32'd0; q_is_wr <= 1'b0; pre_busy <= 1'b0; qt <= 26'd0; q_fail_wait <= 1'b0;
        end else case (qs)
            Q_IDLE: begin
                qt <= 26'd0;
                if (q_go) begin
                    sd_lba  <= q_lba;
                    q_is_wr <= q_go_wr;
                    sd_rd   <= ~q_go_wr;
                    sd_wr   <= q_go_wr;
                    pre_busy <= ~q_go_wr;
                    qs      <= Q_REQ;
                end
            end
            Q_REQ: begin
                qt <= qt + 26'd1;
                if (sd_busy) begin sd_rd <= 1'b0; sd_wr <= 1'b0; qt <= 26'd0; qs <= Q_WAIT; end
                else if (qt[24]) begin                       // 0.5 s: the arbiter has given up, the request never started
                    sd_rd <= 1'b0; sd_wr <= 1'b0; pre_busy <= 1'b0; q_fail <= 1'b1; q_fail_wait <= 1'b0; qs <= Q_IDLE;
                end
            end
            Q_WAIT: begin
                qt <= qt + 26'd1;
                if (!sd_busy) begin pre_busy <= 1'b0; q_done <= 1'b1; qs <= Q_IDLE; end
                else if (qt[25]) begin                       // 2 s: the transfer never ended
                    pre_busy <= 1'b0; q_fail <= 1'b1; q_fail_wait <= 1'b1; qs <= Q_IDLE;
                end
            end
            default: qs <= Q_IDLE;
        endcase
    end

    //------------------------------------------------------------------------
    // edge timing
    //------------------------------------------------------------------------
    reg        to_d;
    reg [19:0] tk;                       // clk32 cycles since the last edge (saturating)
    reg [14:0] sub;                      // clk32 cycles within the current millisecond
    reg [15:0] msc;                      // milliseconds since the last edge (saturating)
    wire       edge_w = (tape_out != to_d);
    wire [22:0] t_w    = {3'b000, tk} * 23'd7;
    wire [16:0] t_ts   = t_w[22:6];      // interval in 3.5 MHz T-states
    wire [15:0] ms_w   = msc + (sub >= 15'd16000 ? 16'd1 : 16'd0);

    //------------------------------------------------------------------------
    // main FSM
    //------------------------------------------------------------------------
    localparam M_IDLE = 3'd0, M_PRE = 3'd1, M_READY = 3'd2, M_REC = 3'd3, M_FIN = 3'd4, M_HDR = 3'd5;
    reg [2:0]  ms;
    reg        motor_d;
    reg [25:0] moff;                     // motor-off time in clk32 cycles (saturates at 2**25)
    reg [31:0] st_off;                   // start offset of the recording
    reg [31:0] wr_off;                   // next write offset
    reg [31:0] cnt_off;                  // offset of the count byte of the open 0x13 block
    reg [31:0] mk_off;                   // offset of the end marker
    reg        blk_open;
    reg [7:0]  cnt;
    reg [15:0] pcount;                   // pulses recorded (saturating); fewer than 64 = nothing is written
    reg        end_req;
    reg        abort;

    // encoder
    localparam E_IDLE = 4'd0, E_HDR0 = 4'd1, E_HDR1 = 4'd2, E_P0 = 4'd3, E_P1 = 4'd4,
               E_CLOSE_N = 4'd5, E_CLOSE_G = 4'd6, E_CLOSE_E = 4'd7,
               E_PAUSE0 = 4'd8, E_PAUSE1 = 4'd9, E_PAUSE2 = 4'd10,
               E_END0 = 4'd11, E_END = 4'd12, E_PAD = 4'd13, E_DONE = 4'd14;
    reg [3:0]  es;
    reg [2:0]  en;                       // end marker byte counter
    reg        ev_v, ev_gap;
    reg [15:0] ev_val;

    // sectors that may be written: complete ones, and not the one holding the count byte of the open block
    wire [22:0] limit    = blk_open ? cnt_off[31:9] : wr_off[31:9];
    wire        flush_en = (pcount >= 16'd64) && !abort;
    wire        flush_ok = (ms == M_REC || ms == M_FIN) && flush_en && (fl_sec < limit) && (qs == Q_IDLE) && !q_go && !q_done;
    wire [22:0] pend_sec = wr_off[31:9] - fl_sec;

    always @(posedge clk) begin
        done    <= 1'b0;
        rb_we   <= 1'b0;
        q_go    <= 1'b0;
        to_d    <= tape_out;
        motor_d <= motor;

        if (reset) begin
            ms <= M_IDLE; active <= 1'b0; err <= 3'd0; new_pos <= 32'd0; size_r <= 32'd0;
            blk_open <= 1'b0; end_req <= 1'b0; abort <= 1'b0; ev_v <= 1'b0; es <= E_IDLE;
            pcount <= 16'd0; fl_sec <= 23'd0; wr_off <= 32'd0; st_off <= 32'd0;
        end else begin
            if (img_mounted) begin
                size_r <= img_size;
                ms <= M_IDLE; active <= 1'b0; err <= 3'd0;
                blk_open <= 1'b0; end_req <= 1'b0; abort <= 1'b0; ev_v <= 1'b0; es <= E_IDLE;
            end else begin
                // time base
                if (tk != 20'hFFFFF) tk <= tk + 20'd1;
                if (sub == 15'd31999) begin
                    sub <= 15'd0;
                    if (msc != 16'hFFFF) msc <= msc + 16'd1;
                end else sub <= sub + 15'd1;

                case (ms)
                M_IDLE:
                    if (enable && present && ((motor && !motor_d) || (motor && edge_w && err == 3'd0))) begin
                        err     <= 1'b0;
                        abort   <= 1'b0;
                        st_off  <= snap;
                        wr_off  <= snap;
                        fl_sec  <= snap[31:9];
                        pcount  <= 16'd0;
                        blk_open <= 1'b0; end_req <= 1'b0; ev_v <= 1'b0; es <= E_IDLE;
                        if (snap[31:9] >= tsec) begin
                            err <= 3'd1;                     // tape end / image full: nothing can be recorded
                        end else if (snap[8:0] == 9'd0) begin
                            ms <= M_READY;
                        end else begin
                            q_lba <= {9'd0, snap[31:9]}; q_go_wr <= 1'b0; q_go <= 1'b1;
                            ms <= M_PRE;
                        end
                    end

                M_PRE:
                    if (q_done) begin
                        if (hdr_bad && st_off[31:9] == 23'd0) begin hc <= 4'd0; ms <= M_HDR; end
                        else ms <= M_READY;
                    end
                    else if (q_fail) ms <= M_IDLE;           // preload failed (err set below)

                M_HDR: begin                                 // the file has no TZX header: write it into the ring (sector 0)
                    rb_we <= 1'b1; rb_wa <= {fl_sec[RSB-1:0], 5'd0, hc}; rb_wd <= hdr_byte(hc);
                    hc <= hc + 4'd1;
                    if (hc == 4'd9) ms <= M_READY;
                end

                M_READY:
                    if (!motor || !enable) ms <= M_IDLE;     // motor cycle without writing
                    else if (edge_w) begin                   // first edge: start measuring
                        ms <= M_REC; active <= 1'b1; moff <= 26'd0;
                        tk <= 20'd0; sub <= 15'd0; msc <= 16'd0;
                    end

                M_REC: begin
                    if (edge_w) begin
                        tk <= 20'd0; sub <= 15'd0; msc <= 16'd0;
                        if (ev_v) begin
                            abort <= 1'b1; if (err == 3'd0) err <= 3'd2;      // encoder still busy: cannot happen with a sane CPC
                        end else begin
                            ev_v <= 1'b1;
                            if (t_ts[16]) begin ev_gap <= 1'b1; ev_val <= (ms_w == 16'd0) ? 16'd1 : ms_w; end
                            else begin ev_gap <= 1'b0; ev_val <= (t_ts[15:0] == 16'd0) ? 16'd1 : t_ts[15:0]; pcount <= (pcount == 16'hFFFF) ? pcount : pcount + 16'd1; end
                        end
                    end
                    // the CPC stops the tape motor between the records of a multi-block SAVE: only a motor that stays
                    // off for ~1 s (2**25 clk32) ends the recording, a shorter stop is a pause (the interval to
                    // the next edge is stored as 0x20 block)
                    if (motor) moff <= 26'd0; else if (!moff[25]) moff <= moff + 26'd1;
                    if (!enable || moff[25]) begin
                        end_req <= 1'b1;
                        ms <= M_FIN;
                    end
                    if (pend_sec >= ((23'd1 << RSB) - 23'd1)) begin abort <= 1'b1; if (err == 3'd0) err <= 3'd3; end   // SD too slow: ring would overrun
                    if (fl_sec > wr_off[31:9]) begin abort <= 1'b1; if (err == 3'd0) err <= 3'd5; end
                end

                M_FIN: begin
                    if (pend_sec >= ((23'd1 << RSB) - 23'd1)) begin abort <= 1'b1; if (err == 3'd0) err <= 3'd3; end
                    if (fl_sec > wr_off[31:9]) begin abort <= 1'b1; if (err == 3'd0) err <= 3'd5; end
                    if (!abort && (pcount < 16'd64)) begin
                        // too short to be a recording: nothing has been written to the SD card, leave the tape alone
                        active <= 1'b0; ms <= M_IDLE; end_req <= 1'b0; ev_v <= 1'b0; es <= E_IDLE; blk_open <= 1'b0;
                    end else if (!abort && es == E_DONE && !ev_v && qs == Q_IDLE && !q_go && fl_sec == wr_off[31:9]) begin
                        active <= 1'b0; ms <= M_IDLE; done <= 1'b1; new_pos <= mk_off;
                    end
                end
                default: ms <= M_IDLE;
                endcase

                // abort from either state: report the start of the recording as the new position
                if (abort && (ms == M_REC || ms == M_FIN) && qs == Q_IDLE && !q_go) begin
                    active <= 1'b0; ms <= M_IDLE; done <= 1'b1; new_pos <= st_off;
                    end_req <= 1'b0; ev_v <= 1'b0; es <= E_IDLE; blk_open <= 1'b0; abort <= 1'b0;
                end

                // sector flush
                if (flush_ok) begin
                    if (fl_sec >= tsec) begin abort <= 1'b1; if (err == 3'd0) err <= 3'd4; end
                    else begin q_lba <= {9'd0, fl_sec}; q_go_wr <= 1'b1; q_go <= 1'b1; end
                end
                if (q_done && q_is_wr) fl_sec <= fl_sec + 23'd1;
                if (q_fail) begin abort <= 1'b1; if (err == 3'd0) err <= q_fail_wait ? 3'd7 : 3'd6; end

                // encoder
                if (!abort) case (es)
                E_IDLE:
                    if (ms == M_REC || ms == M_FIN) begin
                        if (ev_v) begin
                            if (ev_gap) es <= blk_open ? E_CLOSE_G : E_PAUSE0;
                            else        es <= blk_open ? E_P0 : E_HDR0;
                        end else if (end_req) es <= blk_open ? E_CLOSE_E : E_END0;
                    end
                E_HDR0:  begin rb_we <= 1'b1; rb_wa <= wr_off[RSB+8:0]; rb_wd <= 8'h13; wr_off <= wr_off + 32'd1; es <= E_HDR1; end
                E_HDR1:  begin rb_we <= 1'b1; rb_wa <= wr_off[RSB+8:0]; rb_wd <= 8'h00; cnt_off <= wr_off; wr_off <= wr_off + 32'd1;
                               blk_open <= 1'b1; cnt <= 8'd0; es <= E_P0; end
                E_P0:    begin rb_we <= 1'b1; rb_wa <= wr_off[RSB+8:0]; rb_wd <= ev_val[7:0]; wr_off <= wr_off + 32'd1; es <= E_P1; end
                E_P1:    begin rb_we <= 1'b1; rb_wa <= wr_off[RSB+8:0]; rb_wd <= ev_val[15:8]; wr_off <= wr_off + 32'd1;
                               cnt <= cnt + 8'd1; ev_v <= 1'b0;
                               es <= (cnt == 8'd254) ? E_CLOSE_N : E_IDLE; end
                E_CLOSE_N, E_CLOSE_G, E_CLOSE_E: begin
                               rb_we <= 1'b1; rb_wa <= cnt_off[RSB+8:0]; rb_wd <= cnt; blk_open <= 1'b0;
                               es <= (es == E_CLOSE_N) ? E_IDLE : (es == E_CLOSE_G) ? E_PAUSE0 : E_END0; end
                E_PAUSE0: begin rb_we <= 1'b1; rb_wa <= wr_off[RSB+8:0]; rb_wd <= 8'h20; wr_off <= wr_off + 32'd1; es <= E_PAUSE1; end
                E_PAUSE1: begin rb_we <= 1'b1; rb_wa <= wr_off[RSB+8:0]; rb_wd <= ev_val[7:0]; wr_off <= wr_off + 32'd1; es <= E_PAUSE2; end
                E_PAUSE2: begin rb_we <= 1'b1; rb_wa <= wr_off[RSB+8:0]; rb_wd <= ev_val[15:8]; wr_off <= wr_off + 32'd1;
                               ev_v <= 1'b0; es <= E_IDLE; end
                E_END0:  begin rb_we <= 1'b1; rb_wa <= wr_off[RSB+8:0]; rb_wd <= 8'h2A; mk_off <= wr_off; wr_off <= wr_off + 32'd1;
                               en <= 3'd0; es <= E_END; end
                E_END:   begin rb_we <= 1'b1; rb_wa <= wr_off[RSB+8:0]; rb_wd <= 8'h00; wr_off <= wr_off + 32'd1;      // 4 length bytes + terminator
                               en <= en + 3'd1; if (en == 3'd4) es <= E_PAD; end
                E_PAD:   if (wr_off[8:0] == 9'd0) es <= E_DONE;
                         else begin rb_we <= 1'b1; rb_wa <= wr_off[RSB+8:0]; rb_wd <= 8'h00; wr_off <= wr_off + 32'd1; end
                default: ;
                endcase
            end
        end
    end
endmodule

`default_nettype wire

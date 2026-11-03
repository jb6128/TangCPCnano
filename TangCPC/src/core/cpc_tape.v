//============================================================================
// cpc_tape.v -- CDT/TZX tape image playback for the CPC
//
//   OSD file selector index 2 mounts the tape image. The file is streamed from
//   the SD card through a 2 x 512 byte double buffer into the TZX player of
//   NextNano (rtl/tape/tzxplayer.vhd, G. Szombathelyi, GPL), which turns the
//   blocks into the pulse train `cass_read`. That signal is fed to PPI port B
//   bit 7 (tape_in); the player only runs while the tape motor (PPI C4) is on.
//
//   The sector for the next 512 bytes is fetched while the current one is being
//   consumed, so a 1 ms SD access never stretches a pulse.
//
//   Protocol of the player: it toggles tzx_req to ask for the next file byte,
//   the host puts it on host_tap_in and then sets tzx_ack = tzx_req.
//
//   CPC (CDT) timing is selected via the generics: pilot 2000, sync 855/855,
//   zero 855, one 1710, 4096 pilot pulses (all in 3.5 MHz T-states).
//
//   Tape position and end of tape: the wrapper notes the file offset of every block ID
//   handed to the player (`blk_start`). A block ID 0x00 (never valid, it is the zero
//   fill of a blank tape and the terminator written after a recording) or the end of
//   the file is the end of the tape: the player is no longer served.
//
//   Recording (SAVE): see cpc_taprec.v. The recorder samples `snap` (where a new block
//   would start) when the motor is switched on. While it records, the player is frozen;
//   afterwards the player is restarted (without TZX header) at the end marker, so
//   that the next SAVE appends and a LOAD without rewinding finds the end of the tape.
//============================================================================
`default_nettype none

module cpc_tape (
    input  wire        clk,       // clk32
    input  wire        reset,     // active high

    input  wire        img_mounted,   // pulse: image 2 (re)mounted
    input  wire [31:0] img_size,

    input  wire        motor,         // tape motor relay (PPI port C bit 4)
    input  wire        wr_en,         // OSD: Tape write on
    input  wire        tape_out,      // PPI port C bit 5 (cassette write data)
    output wire        rec_active,    // recording (LED)
    output wire  [2:0] rec_err,       // recording failed: error code (LED blink count)
    input  wire        invert,        // OSD: invert the tape signal polarity
    output wire        tape_in,
    output wire        present,

    // sector read requests (sd_arb source 2)
    output reg         sd_rd,
    output reg  [31:0] sd_lba,
    input  wire        sd_busy,       // granted to us and SD busy
    input  wire  [8:0] sd_buff_addr,
    input  wire  [7:0] sd_buff_dout,
    input  wire        sd_buff_wr,

    // recorder: sector write / read-modify-write requests (sd_arb requester `rec`)
    output wire        rec_sd_rd,
    output wire        rec_sd_wr,
    output wire [31:0] rec_sd_lba,
    input  wire        rec_sd_busy,
    input  wire        rec_buff_wr,
    output wire  [7:0] rec_wr_data
);
    //------------------------------------------------------------------------
    // image state
    //------------------------------------------------------------------------
    reg [31:0] size;
    reg [31:0] pos;               // file offset of the next byte for the player
    reg [22:0] ld_next;           // next sector to fetch (= number of sectors loaded)
    reg        present_r;
    assign present = present_r;

    wire [22:0] total_sec = size[31:9] + (|size[8:0] ? 23'd1 : 23'd0);
    wire [22:0] cur_sec   = pos[31:9];

    reg  [2:0] restart_cnt;
    wire       restart_tape = (restart_cnt != 3'd0) | reset;
    reg        nohdr;                  // restart the player at a block ID instead of the TZX header
    wire       rec_done;
    wire [31:0] rec_new_pos;
    wire       rec_act;
    wire       st_id, st_fin;
    reg [31:0] blk_start;              // file offset of the latest block ID handed to the player
    reg        at_end;                 // end of the tape reached: player not served any more
    reg        pend_id;                // the byte the player holds is a block ID it has not consumed yet
    reg        req_id;

    //------------------------------------------------------------------------
    // 1 KiB double buffer
    //------------------------------------------------------------------------
    reg [7:0] bmem [0:1023];
    reg       fetch_half;
    reg [9:0] rd_addr;
    reg [7:0] mem_q;
    always @(posedge clk) begin
        if (sd_buff_wr) bmem[{fetch_half, sd_buff_addr}] <= sd_buff_dout;
        mem_q <= bmem[rd_addr];
    end

    //------------------------------------------------------------------------
    // sector fetch
    //------------------------------------------------------------------------
    localparam F_IDLE = 2'd0, F_REQ = 2'd1, F_WAIT = 2'd2;
    reg [1:0] fstate;
    reg       discard;            // transfer in flight belongs to an unmounted image

    wire need_fetch = present_r && (ld_next < total_sec) && (ld_next <= cur_sec + 23'd1);

    always @(posedge clk) begin
        if (reset) begin
            fstate <= F_IDLE; sd_rd <= 1'b0; sd_lba <= 32'd0; discard <= 1'b0;
            ld_next <= 23'd0; fetch_half <= 1'b0;
        end else begin
            if (img_mounted) begin
                ld_next <= 23'd0;
                discard <= (fstate != F_IDLE);
            end else if (rec_done) begin
                ld_next <= rec_new_pos[31:9];
                discard <= (fstate != F_IDLE);
            end
            case (fstate)
            F_IDLE:
                if (!img_mounted && !rec_done && need_fetch) begin
                    sd_lba     <= {9'd0, ld_next};
                    fetch_half <= ld_next[0];
                    sd_rd      <= 1'b1;
                    fstate     <= F_REQ;
                end
            F_REQ:
                if (sd_busy) begin sd_rd <= 1'b0; fstate <= F_WAIT; end
            F_WAIT:
                if (!sd_busy) begin
                    if (!discard && !img_mounted && !rec_done) ld_next <= ld_next + 23'd1;
                    discard <= 1'b0;
                    fstate  <= F_IDLE;
                end
            default: fstate <= F_IDLE;
            endcase
        end
    end

    //------------------------------------------------------------------------
    // byte delivery to the player
    //------------------------------------------------------------------------
    wire tzx_req;
    reg  tzx_ack;
    reg  [7:0] host_byte;
    localparam D_IDLE = 2'd0, D_RD1 = 2'd1, D_RD2 = 2'd2, D_ACK = 2'd3;
    reg [1:0] dstate;

    always @(posedge clk) begin
        if (reset) begin
            size <= 32'd0; pos <= 32'd0; present_r <= 1'b0; restart_cnt <= 3'd0;
            dstate <= D_IDLE; tzx_ack <= 1'b0; host_byte <= 8'h00; rd_addr <= 10'd0;
            nohdr <= 1'b0; blk_start <= 32'd10; at_end <= 1'b0; pend_id <= 1'b0; req_id <= 1'b0;
        end else begin
            if (restart_cnt != 3'd0) restart_cnt <= restart_cnt - 3'd1;

            if (img_mounted) begin
                size        <= img_size;
                present_r   <= |img_size;
                pos         <= 32'd0;
                dstate      <= D_IDLE;
                restart_cnt <= 3'd4;
                nohdr       <= 1'b0;
                blk_start   <= 32'd10;
                at_end      <= 1'b0;
                pend_id     <= 1'b0;
            end else if (rec_done) begin
                // a recording ended: continue the tape at the end marker (or at the start of an aborted recording)
                pos         <= rec_new_pos;
                blk_start   <= rec_new_pos;
                dstate      <= D_IDLE;
                restart_cnt <= 3'd4;
                nohdr       <= 1'b1;
                host_byte   <= 8'h00;            // the player takes one stale byte after a restart: an ignored ID
                at_end      <= 1'b0;
                pend_id     <= 1'b0;
            end else case (dstate)
            D_IDLE:
                if (tzx_req != tzx_ack && present_r && !at_end) begin
                    pend_id <= 1'b0;                     // the player consumed what it held
                    req_id  <= st_id;
                    if (pos >= size) begin
                        if (st_id) begin at_end <= 1'b1; blk_start <= pos; end
                    end else if (cur_sec < ld_next) begin
                        rd_addr <= pos[9:0];
                        dstate  <= D_RD1;
                    end
                end
            D_RD1: dstate <= D_RD2;                       // BRAM read latency
            D_RD2: begin host_byte <= mem_q; dstate <= D_ACK; end
            D_ACK:
                if (req_id && host_byte == 8'h00) begin   // block ID 0: end of the tape
                    at_end    <= 1'b1;
                    blk_start <= pos;
                    dstate    <= D_IDLE;
                end else begin
                    tzx_ack <= tzx_req;
                    pos     <= pos + 32'd1;
                    if (req_id) begin blk_start <= pos; pend_id <= 1'b1; end
                    dstate  <= D_IDLE;
                end
            endcase
        end
    end

    //------------------------------------------------------------------------
    // recorder
    //------------------------------------------------------------------------
    // where a new block would start: the next block when the player holds a block ID it has not
    // used yet, or when the data of the current block has been fetched completely (the pause or the
    // last bits are playing); otherwise the start of the block the player is in
    wire [31:0] snap = (!at_end && !pend_id && st_fin) ? pos : blk_start;

    cpc_taprec rec (
        .clk         ( clk          ),
        .reset       ( reset        ),
        .enable      ( wr_en        ),
        .present     ( present_r    ),
        .img_mounted ( img_mounted  ),
        .img_size    ( img_size     ),
        .motor       ( motor        ),
        .tape_out    ( tape_out     ),
        .snap        ( snap         ),
        .active      ( rec_act      ),
        .done        ( rec_done     ),
        .new_pos     ( rec_new_pos  ),
        .err         ( rec_err      ),
        .sd_rd       ( rec_sd_rd    ),
        .sd_wr       ( rec_sd_wr    ),
        .sd_lba      ( rec_sd_lba   ),
        .sd_busy     ( rec_sd_busy  ),
        .sd_addr     ( sd_buff_addr ),
        .sd_din      ( sd_buff_dout ),
        .sd_din_wr   ( rec_buff_wr  ),
        .sd_dout     ( rec_wr_data  )
    );
    assign rec_active = rec_act;

    //------------------------------------------------------------------------
    // the player (CPC timings, ce = every clk32 cycle => TZX_MS = 32000)
    //------------------------------------------------------------------------
    wire cass_read;

    tzxplayer #(
        .TZX_MS              ( 32000 ),
        .NORMAL_PILOT_LEN    ( 2000  ),
        .NORMAL_SYNC1_LEN    ( 855   ),
        .NORMAL_SYNC2_LEN    ( 855   ),
        .NORMAL_ZERO_LEN     ( 855   ),
        .NORMAL_ONE_LEN      ( 1710  ),
        .NORMAL_PILOT_PULSES ( 4096  )
    ) player (
        .clk          ( clk          ),
        .ce           ( 1'b1         ),
        .restart_tape ( restart_tape ),
        .no_header    ( nohdr        ),
        .host_tap_in  ( host_byte    ),
        .tzx_req      ( tzx_req      ),
        .tzx_ack      ( tzx_ack      ),
        .loop_start   (              ),
        .loop_next    (              ),
        .stop         (              ),
        .stop48k      (              ),
        .st_id        ( st_id        ),
        .st_fin       ( st_fin       ),
        .cass_read    ( cass_read    ),
        .cass_motor   ( motor & ~rec_act )
    );

    assign tape_in = present_r ? (cass_read ^ invert) : 1'b0;
endmodule

`default_nettype wire

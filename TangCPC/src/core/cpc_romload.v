//============================================================================
// cpc_romload.v -- external upper ROMs (expansion ROM slots 0..31, like WinAPE)
//
//   OSD: list "L" selects the slot, file selector index 3 loads a .rom file
//   into it. A 16 KB image (or 16 KB + 128 byte AMSDOS header, as written by
//   some tools) is read from the SD card sector by sector, buffered, and written
//   byte-wise into SDRAM at 0x400000 + slot * 0x4000 through the boot port.
//   The slot is then marked in rom_map, which the MMU uses to accept the
//   upper-ROM select (&DF00).  Selecting an empty file / "Clear slot" removes it.
//
//   Slots 0 (BASIC) and 7 (DOS) have an INTERNAL ROM (flash copy, loaded by the boot loader of
//   cpc_core, never touched here) and an optional EXTERNAL ROM. The external ROMs live in their own
//   SDRAM areas (slot 0 at 0x480000, slot 7 at 0x484000); present[0] / present[7] mean "external ROM
//   loaded". The OSD options ROM0 / ROM7 (rom0_ext / rom7_ext: 0 = INT, 1 = EXT) choose the source:
//   EXT with a loaded external ROM -> the external one (ext_sel), otherwise the internal one.
//   Slot 7 is visible to the CPC when the external ROM is used or the OSD option "AMSDOS" is On
//   (rom_map[7]); Off and no external ROM = no DOS (tape only). Slots 1..6 and 8..31 as before.
//
//   ROM SET (image 4, "ROM set:" in the OSD): one file with up to 16 consecutive 16 KB blocks;
//   block n goes to slot n (block 0 / 7 into the external slot 0 / 7). An all-0x00 / all-0xFF
//   block is empty and leaves the slot alone.
//   Slots loaded individually through image 3 have priority: the set never overwrites them
//   (flag `indiv`, cleared by "Clear slot" and cold boot), whatever the order of the two mounts.
//
//   The boot port is shared with the boot loader of cpc_core: this block only
//   touches it while `boot_idle` is high.  A ROM only becomes visible to the
//   CPC after the next reset (the firmware scans the upper ROMs at start).
//============================================================================
`default_nettype none

module cpc_romload (
    input  wire        clk,
    input  wire        reset,           // power-on reset (rom_map survives OSD resets)

    input  wire        img_mounted,     // pulse: image 3 (re)mounted
    input  wire        set_mounted,     // pulse: image 4 (ROM set) (re)mounted
    input  wire [31:0] img_size,
    input  wire  [4:0] slot,            // slot chosen in the OSD (latched at mount)
    input  wire        clear,           // level from OSD button; rising edge clears the slot
    input  wire        clear_set,       // level from OSD button; rising edge removes the ROM set
    input  wire        amsdos_en,       // OSD: internal slot 7 (AMSDOS) visible to the MMU
    input  wire        rom0_ext,        // OSD ROM0: 1 = use the external slot-0 ROM when one is loaded
    input  wire        rom7_ext,        // OSD ROM7: 1 = use the external slot-7 ROM when one is loaded

    input  wire        boot_idle,       // SDRAM boot port free
    input  wire        forget,          // Factory boot: nothing is loaded at the next release (mounts stay remembered)

    // sector requests (sd_arb source 3)
    output reg         sd_rd,
    output wire        sd_set,          // 1 = the request is for image 4 (ROM set), 0 = image 3
    output reg  [31:0] sd_lba,
    input  wire        sd_busy,
    input  wire  [8:0] sd_buff_addr,
    input  wire  [7:0] sd_buff_dout,
    input  wire        sd_buff_wr,

    // SDRAM boot port (byte writes)
    output reg  [22:0] ram_addr,
    output reg   [7:0] ram_din,
    output reg         ram_req,
    input  wire        ram_ack,

    output wire [255:0] rom_map,
    output wire  [1:0] ext_sel,         // {slot 7, slot 0}: the MMU reads the external ROM (SDRAM 0x484000 / 0x480000)
    output wire        busy,
    output wire  [4:0] prog             // load progress for the start-up bar: cells 0..16 (slots done)
);
    //------------------------------------------------------------------------
    // slot map
    //------------------------------------------------------------------------
    reg [31:0] present;                   // slot holds a loaded ROM (0 / 7: the EXTERNAL ROM)
    assign ext_sel = {rom7_ext & present[7], rom0_ext & present[0]};
    // slot 0: the internal BASIC is always there; slot 7: visible with an external ROM in use or AMSDOS On
    assign rom_map = {224'd0, present[31:8], ext_sel[1] | amsdos_en, present[6:1], 1'b1};

    reg clearD;
    wire clear_edge = clear & ~clearD;
    reg clearsetD;
    wire clearset_edge = clear_set & ~clearsetD;
    reg [31:0] fromset;                   // slots whose content came from the ROM set

    //------------------------------------------------------------------------
    // 512 byte sector buffer
    //------------------------------------------------------------------------
    reg [7:0] bmem [0:511];
    reg [8:0] rd_addr;
    reg [7:0] mem_q;
    always @(posedge clk) begin
        if (sd_buff_wr) bmem[sd_buff_addr] <= sd_buff_dout;
        mem_q <= bmem[rd_addr];
    end

    //------------------------------------------------------------------------
    // job FSM
    //------------------------------------------------------------------------
    localparam S_IDLE = 4'd0,  S_PREP = 4'd1,  S_WAITB = 4'd2, S_REQ  = 4'd3,
               S_WB   = 4'd4,  S_WD   = 4'd5,  S_RD1   = 4'd6, S_RD2  = 4'd7,
               S_CHK  = 4'd8,  S_WR   = 4'd9,  S_ACK   = 4'd10, S_NEXTI = 4'd11,
               S_FIN  = 4'd12, S_SETNEXT = 4'd13;
    reg [3:0]  state;
    // mount pulses can arrive while a load is running: remember them
    // They survive the reset: at start-up the companion mounts the images from cpc.ini while the
    // machine is still held in cold reset (R=3), and those pulses must not be lost.
    reg         pend_ind = 1'b0, pend_set = 1'b0;
    reg  [31:0] sz_ind = 32'd0, sz_set = 32'd0;
    reg  [4:0]  slot_ind = 5'd0;
    assign busy = (state != S_IDLE) || pend_ind || pend_set;

    // ROM set state
    reg         setmode;                  // current job is a ROM set
    reg         nonempty;                 // block holds data other than 0x00 / 0xFF
    reg  [4:0]  blk;                      // block (= slot) in the set (5 bit: reaches nblk = 16)
    reg  [4:0]  nblk;                     // number of blocks in the file (max 16)
    reg  [31:0] indiv;                    // slot was loaded individually (priority over the set)
    assign sd_set = setmode;

    reg  [4:0]  slot_l;
    reg  [31:0] size_l;
    reg  [7:0]  skip;
    reg  [14:0] nbytes;           // 0..16384
    reg  [5:0]  nsec;
    // progress (cells = slots): ROM set -> blocks done; single ROM -> its slot number + 1 (max 16)
    assign prog = setmode ? blk : (state != S_IDLE) ? ((slot_l >= 5'd16) ? 5'd16 : slot_l + 5'd1) : 5'd0;
    reg resetD = 1'b1;
    reg skip_rearm = 1'b0;
    reg  [5:0]  k;                // sector index
    reg  [8:0]  i;                // byte index in the sector

    wire [15:0] f       = {1'b0, k, 9'd0} + {7'd0, i};
    wire [15:0] f_end   = {8'd0, skip} + {1'b0, nbytes};
    wire        in_rng  = (f >= {8'd0, skip}) && (f < f_end);
    wire [15:0] off     = f - {8'd0, skip};
    // SDRAM address of the slot: 0x400000 + slot * 0x4000, except the external slots 0 / 7 (own areas)
    wire [22:0] sbase   = (slot_l == 5'd0) ? 23'h480000 :
                          (slot_l == 5'd7) ? 23'h484000 : (23'h400000 + {slot_l, 14'd0});

    wire [31:0] avail   = size_l - {24'd0, skip};

    always @(posedge clk) begin
        clearD <= clear;
        clearsetD <= clear_set;
        if (reset) begin
            present <= 32'd0;                  // no external ROM; internal BASIC / AMSDOS come from the flash
            fromset <= 32'd0; clearsetD <= 1'b0;
            state <= S_IDLE; sd_rd <= 1'b0; sd_lba <= 32'd0; ram_req <= 1'b0;
            ram_addr <= 23'd0; ram_din <= 8'd0; clearD <= 1'b0;
            slot_l <= 5'd0; size_l <= 32'd0; skip <= 8'd0; nbytes <= 15'd0;
            nsec <= 6'd0; k <= 6'd0; i <= 9'd0; rd_addr <= 9'd0;
            setmode <= 1'b0; nonempty <= 1'b0; blk <= 5'd0; nblk <= 5'd0; indiv <= 32'd0;
        end else begin
            ram_req <= 1'b0;

            // clear button (only while no load is in progress); slot 0 / 7: removes the external ROM only,
            // the internal BASIC / AMSDOS are separate and stay
            if (clear_edge && state == S_IDLE) begin
                present[slot] <= 1'b0;
                indiv[slot]   <= 1'b0;
                fromset[slot] <= 1'b0;
            end
            // Remove ROM set: all slots filled by the set go away (also external slots 0 / 7).
            // The set is forgotten.
            if (clearset_edge && state == S_IDLE) begin
                present <= present & ~fromset;
                fromset <= 32'd0;
                pend_set <= 1'b0;
                sz_set <= 32'd0;
            end

            case (state)
            S_IDLE:
                if (pend_ind) begin                        // individual ROM file (image 3)
                    pend_ind <= 1'b0;
                    setmode  <= 1'b0;
                    slot_l   <= slot_ind;
                    size_l   <= sz_ind;
                    state    <= S_PREP;
                end else if (pend_set && sz_set == 32'd0) begin   // empty selection (NoDisk) = remove the set
                    pend_set <= 1'b0;
                    present <= present & ~fromset;
                    fromset <= 32'd0;
                end else if (pend_set) begin               // ROM set (image 4); replaces a set loaded before
                    pend_set <= 1'b0;
                    present <= present & ~fromset;
                    fromset <= 32'd0;
                    setmode  <= 1'b1;
                    size_l   <= sz_set;
                    nblk     <= (|sz_set[31:18]) ? 5'd16 : {1'b0, sz_set[17:14]};
                    blk      <= 5'd0;
                    state    <= S_SETNEXT;
                end

            // next block of the ROM set: individually loaded slots are skipped
            S_SETNEXT: begin
                if (blk >= nblk) begin
                    setmode <= 1'b0;
                    state   <= S_IDLE;
                end else if (indiv[blk]) begin
                    blk <= blk + 5'd1;
                end else begin
                    slot_l   <= blk;
                    skip     <= 8'd0;
                    nonempty <= 1'b0;
                    state    <= S_WAITB;
                end
            end

            S_PREP: begin
                if (size_l == 32'd0) begin                 // empty selection = eject
                    present[slot_l] <= 1'b0;
                    indiv[slot_l] <= 1'b0;
                    state <= S_IDLE;
                end else begin
                    skip   <= (size_l == 32'd16512) ? 8'd128 : 8'd0;
                    state  <= S_WAITB;
                end
            end

            S_WAITB: begin     // size and skip known now; nbytes/nsec from them
                nbytes <= (avail >= 32'd16384) ? 15'd16384 : avail[14:0];
                k      <= 6'd0;
                i      <= 9'd0;
                if (boot_idle) state <= S_REQ;
            end

            S_REQ: begin
                // sectors needed = ceil((skip + nbytes) / 512)
                nsec   <= (({1'b0, skip} + {1'b0, nbytes} + 16'd511) >> 9);
                sd_lba <= {26'd0, k} + (setmode ? {22'd0, blk, 5'd0} : 32'd0);
                sd_rd  <= 1'b1;
                state  <= S_WB;
            end
            S_WB: if (sd_busy) begin sd_rd <= 1'b0; state <= S_WD; end
            S_WD: if (!sd_busy) begin i <= 9'd0; rd_addr <= 9'd0; state <= S_RD1; end

            S_RD1: state <= S_RD2;                    // BRAM latency
            S_RD2: state <= S_CHK;
            S_CHK: begin
                if (in_rng && setmode && mem_q != 8'h00 && mem_q != 8'hFF) nonempty <= 1'b1;
                if (in_rng) begin
                    ram_addr <= sbase + {7'd0, off};
                    ram_din  <= mem_q;
                    ram_req  <= 1'b1;
                    state    <= S_ACK;
                end else
                    state <= S_NEXTI;
            end
            S_ACK: if (ram_ack) state <= S_NEXTI;

            S_NEXTI: begin
                if (i == 9'd511) begin
                    k <= k + 6'd1;
                    if (k + 6'd1 >= nsec) state <= S_FIN;
                    else                  state <= S_REQ;
                end else begin
                    i       <= i + 9'd1;
                    rd_addr <= i + 9'd1;
                    state   <= S_RD1;
                end
            end

            S_FIN: begin
                if (!setmode) begin
                    present[slot_l] <= 1'b1;
                    indiv[slot_l]   <= 1'b1;
                    fromset[slot_l] <= 1'b0;
                    state <= S_IDLE;
                end else begin                             // block written
                    present[slot_l] <= nonempty;
                    if (nonempty) fromset[slot_l] <= 1'b1;
                    blk   <= blk + 5'd1;
                    state <= S_SETNEXT;
                end
            end

            default: state <= S_IDLE;
            endcase
        end

        // Cold boot / power-on: when the reset is released and the companion has not mounted anything
        // again (it does not seem to re-send the mounts of images that are still mounted), reload the
        // images that are still mounted (the OSD still shows them): the set and the single ROM, like
        // after power-on with the same cpc.ini. A mount pulse that arrived during the reset is
        // already pending and wins (same flags, so nothing is loaded twice).
        resetD <= reset;
        if (resetD && !reset) begin
            if (!skip_rearm && !pend_ind && !pend_set) begin
                if (sz_ind != 32'd0) pend_ind <= 1'b1;
                if (sz_set != 32'd0) pend_set <= 1'b1;
            end
            skip_rearm <= 1'b0;                  // only the Factory boot release is skipped
        end
        // remember mount pulses (the load FSM may be busy, or in reset); last assignment wins
        // (a cold boot re-runs the companion's init action, which mounts the images from cpc.ini again)
        if (img_mounted) begin pend_ind <= 1'b1; sz_ind <= img_size; slot_ind <= slot; end
        if (set_mounted) begin pend_set <= 1'b1; sz_set <= img_size; end
        // Factory boot: load nothing at this release, but keep the remembered mounts (the companion
        // does not re-send them), so that a later Cold boot loads the ROMs of cpc.ini again.
        if (forget) begin pend_ind <= 1'b0; pend_set <= 1'b0; skip_rearm <= 1'b1; end
    end
endmodule

`default_nettype wire

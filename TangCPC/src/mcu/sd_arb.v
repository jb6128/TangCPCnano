//============================================================================
// sd_arb.v -- arbitration of the shared sector-request interface of sd_card
//
// sd_card has ONE rsector input but eight request lines (source = bit index).
// The companion reads {rstart|wstart, rsector} in its status poll, so while a
// request is pending rsector must belong to exactly one requester. This block
// grants the interface to one requester at a time:
//
//   source 0/1 : floppy controller (cpc_fdc, u765)
//   source 2   : tape loader       (cpc_tape)
//   source 3   : ROM loader        (cpc_romload), image 3; image 4 (ROM set) also uses this requester
//   source 2   : tape recorder     (cpc_taprec): sector writes / read-modify-write of the tape image
//                (same image index 2 as the tape loader, but a requester of its own)
//
// A grant is taken while idle (priority: floppy, tape recorder, ROM loader, tape loader) and released when the transfer
// has started (rbusy seen) and is over again (rbusy low).
//============================================================================
`default_nettype none

module sd_arb #(
    // minimum pause between two transfers (clk cycles, 32 MHz): the FPGA Companion
    // needs time to finish the previous request, otherwise it answers a new request
    // that follows too quickly with the sector of the previous one (data lags by
    // one sector).  640000 cycles = 20 ms.
    parameter [19:0] GAP = 20'd640000
) (
    input  wire        clk,
    input  wire        reset,

    input  wire [31:0] fdc_lba,
    input  wire  [1:0] fdc_rd,
    input  wire  [1:0] fdc_wr,

    input  wire [31:0] tape_lba,
    input  wire        tape_rd,

    input  wire [31:0] rec_lba,
    input  wire        rec_rd,
    input  wire        rec_wr,

    input  wire [31:0] rom_lba,
    input  wire        rom_rd,
    input  wire        rom_set,        // ROM loader request is for image 4 (ROM set)

    input  wire        rbusy,          // from sd_card

    output wire  [7:0] rstart,
    output wire  [7:0] wstart,
    output wire [31:0] rsector,
    output wire        grant_fdc,
    output wire        grant_tape,
    output wire        grant_rec,
    output wire        grant_rom
);
    reg g_fdc, g_tape, g_rec, g_rom, seen_busy;
    reg [19:0] gap;
    reg [23:0] gw;                 // watchdog: grant held without a finished transfer

    always @(posedge clk) begin
        if (reset) begin
            g_fdc <= 1'b0; g_tape <= 1'b0; g_rec <= 1'b0; g_rom <= 1'b0; seen_busy <= 1'b0; gap <= 20'd0; gw <= 24'd0;
        end else if (!g_fdc && !g_tape && !g_rec && !g_rom) begin
            seen_busy <= 1'b0;
            gw <= 24'd0;
            if (gap != 20'd0) gap <= gap - 20'd1;
            else if (|fdc_rd || |fdc_wr) g_fdc  <= 1'b1;
            else if (rec_rd || rec_wr)   g_rec  <= 1'b1;
            else if (rom_rd)             g_rom  <= 1'b1;     // ROM loads before tape: a tape that is read without
            else if (tape_rd)            g_tape <= 1'b1;     // pause (unknown block IDs) must not starve a ROM load
        end else begin
            if (rbusy) seen_busy <= 1'b1;
            gw <= gw + 24'd1;
            // a requester that gave up before the transfer started (or an SD card that is
            // not ready yet) would otherwise block the interface for ever: give up after 0.5 s
            if ((seen_busy && !rbusy) || gw[23]) begin
                g_fdc  <= 1'b0;
                g_tape <= 1'b0;
                g_rec  <= 1'b0;
                g_rom  <= 1'b0;
                gap    <= GAP;
            end
        end
    end

    assign rstart  = g_fdc ? {6'b000000, fdc_rd} : (g_rec ? {5'b00000, rec_rd, 2'b00} : g_tape ? 8'b0000_0100 : (g_rom ? (rom_set ? 8'b0001_0000 : 8'b0000_1000) : 8'h00));
    assign wstart  = g_fdc ? {6'b000000, fdc_wr} : (g_rec ? {5'b00000, rec_wr, 2'b00} : 8'h00);
    assign rsector = g_rec ? rec_lba : (g_tape ? tape_lba : (g_rom ? rom_lba : fdc_lba));

    assign grant_fdc  = g_fdc;
    assign grant_tape = g_tape;
    assign grant_rec  = g_rec;
    assign grant_rom  = g_rom;
endmodule

`default_nettype wire

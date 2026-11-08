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
    output wire  [1:0] ready_o
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
        .sd_buff_wr   ( sd_buff_wr                )
    );

    assign dout    = (io_rd & fdc_cs) ? fdc_dout : 8'hFF;
    assign motor_o = motor;
    assign ready_o = ready;

endmodule

`default_nettype wire

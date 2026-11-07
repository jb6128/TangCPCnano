/*

	Reworked Amstrad MMU for simplicity
	(C) 2018 Sorgelig

--------------------------------------------------------------------------------
--    {@{@{@{@{@{@
--  {@{@{@{@{@{@{@{@  This code is covered by CoreAmstrad synthesis r004
--  {@    {@{@    {@  A core of Amstrad CPC 6128 running on MiST-board platform
--  {@{@{@{@{@{@{@{@
--  {@  {@{@{@{@  {@  CoreAmstrad is implementation of FPGAmstrad on MiST-board
--  {@{@        {@{@   Contact : renaudhelias@gmail.com
--  {@{@{@{@{@{@{@{@   @see http://code.google.com/p/mist-board/
--    {@{@{@{@{@{@     @see FPGAmstrad at CPCWiki
--
--
--------------------------------------------------------------------------------
-- FPGAmstrad_amstrad_motherboard.Amstrad_MMU
-- RAM ROM mapping split
--------------------------------------------------------------------------------
*/

//http://www.grimware.org/doku.php/documentations/devices/gatearray

module Amstrad_MMU
(
	input        CLK,
	input        reset,

	input        ram64k,
	input  [1:0] model,         // 0/3 = 6128, 1 = 664, 2 = 464 (lower ROM and upper ROM 0 at other SDRAM addresses)
	input        romen_n,
	input [255:0]rom_map,
	input [1:0]  rom_ext,       // {7, 0}: external upper ROM 7 / 0 in use (own SDRAM areas 0x484000 / 0x480000)

	input        io_WR,

	input  [7:0] D,
	input [15:0] A,
	output reg [22:0] ram_A
);

reg [2:0] RAMmap;
reg [4:0] RAMpage;
reg [7:0] ROMbank;
reg       old_wr = 0;

always @(posedge CLK) begin

	if (reset) begin
		ROMbank    <=0;
		RAMmap     <=0;
		RAMpage    <=3;
	end
	else begin
		old_wr <= io_WR;
		if (~old_wr & io_WR) begin
			if (~A[15] && D[7:6] == 'b11 && ~ram64k) begin //7Fxx PAL MMR
				RAMpage <= {1'b0, ~A[8], D[5:3]} + 5'd3;
				RAMmap  <= D[2:0];
			end

			// As the ROM selection is built into the expansion cartridges,
			// activate it only, when the appropriate ROM is present.
			if (~A[13]) ROMbank <= rom_map[D] ? D : 8'h00;
		end
	end
end

always @(*) begin
	casex({romen_n, RAMmap, A[15:14]})
		'b0_xxx_xx: ram_A[22:14] = A[15] ? ((ROMbank == 8'h00 && rom_ext[0]) ? 9'h120 :                    // upper rom 0: external BASIC (0x480000)
		                                    (ROMbank == 8'h07 && rom_ext[1]) ? 9'h121 :                    // upper rom 7: external DOS (0x484000)
		                                    (model == 2'd1 && ROMbank == 8'h00) ? 9'h002 :                 // internal; 664: BASIC at 0x008000
		                                    (model == 2'd2 && ROMbank == 8'h00) ? 9'h004 : {1'b1, ROMbank}) // 464: BASIC 1.0 at 0x010000
		                                 : (model == 2'd1 ? 9'h001 :                                       // lower rom (664: OS at 0x004000)
		                                    model == 2'd2 ? 9'h003 : 9'h000);                              // 464: OS at 0x00C000
		'b1_0x1_11,                                               // map1&3 bank3
		'b1_010_xx: ram_A[22:14] = {2'b00, RAMpage,    A[15:14]}; // map2   bank0-3 (ext  0..3)
		'b1_011_01: ram_A[22:14] = {2'b00,    5'd2,       2'b11}; // map3   bank1   (base 3)
		'b1_1xx_01: ram_A[22:14] = {2'b00, RAMpage, RAMmap[1:0]}; // map4-7 bank1   (ext  0..3)
		   default: ram_A[22:14] = {2'b00,    5'd2,    A[15:14]}; // base 64KB map  (base 0..3)
	endcase

	ram_A[13:0] = A[13:0];
end

endmodule

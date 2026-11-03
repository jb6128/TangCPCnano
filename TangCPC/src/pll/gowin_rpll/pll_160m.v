//Copyright (C)2014-2023 Gowin Semiconductor Corporation.
//All rights reserved.
//File Title: IP file
//GOWIN Version: V1.9.9 Beta-2
//Part Number: GW2AR-LV18QN88C8/I7
//Device: GW2AR-18
//Device Version: C
//Created Time: Sat Aug 19 10:17:50 2023

// 96 MHz (from flash_pll) * 5 / 3 = 160.000 MHz (exact; VCO 640 MHz, PFD 32 MHz).
// clk32 = 160 MHz / 5 = 32.000 MHz (before: 27 * 53 / 9 = 159 MHz -> clk32 = 31.8 MHz, 0.625 % slow).
module pll_160m (clkout, lock, clkin, reset);

output clkout;
output lock;
input clkin;
input reset;      // hold high until the input PLL (flash_pll) has locked

wire clkoutp_o;
wire clkoutd_o;
wire clkoutd3_o;
wire gw_gnd;

assign gw_gnd = 1'b0;

rPLL #(
    .FCLKIN("96"),
    .DYN_IDIV_SEL("false"),
    .IDIV_SEL(2),
    .DYN_FBDIV_SEL("false"),
    .FBDIV_SEL(4),
    .DYN_ODIV_SEL("false"),
    .ODIV_SEL(4),
    .PSDA_SEL("0000"),
    .DYN_DA_EN("true"),
    .DUTYDA_SEL("1000"),
    .CLKOUT_FT_DIR(1'b1),
    .CLKOUTP_FT_DIR(1'b1),
    .CLKOUT_DLY_STEP(0),
    .CLKOUTP_DLY_STEP(0),
    .CLKFB_SEL("internal"),
    .CLKOUT_BYPASS("false"),
    .CLKOUTP_BYPASS("false"),
    .CLKOUTD_BYPASS("false"),
    .DYN_SDIV_SEL(2),
    .CLKOUTD_SRC("CLKOUT"),
    .CLKOUTD3_SRC("CLKOUT"),
    .DEVICE("GW2AR-18C")
) rpll_inst (
    .CLKOUT(clkout),
    .LOCK(lock),
    .CLKOUTP(clkoutp_o),
    .CLKOUTD(clkoutd_o),
    .CLKOUTD3(clkoutd3_o),
    .RESET(reset),
    .RESET_P(gw_gnd),
    .CLKIN(clkin),
    .CLKFB(gw_gnd),
    .FBDSEL({gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd}),
    .IDSEL({gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd}),
    .ODSEL({gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd}),
    .PSDA({gw_gnd,gw_gnd,gw_gnd,gw_gnd}),
    .DUTYDA({gw_gnd,gw_gnd,gw_gnd,gw_gnd}),
    .FDLY({gw_gnd,gw_gnd,gw_gnd,gw_gnd})
);


endmodule //pll_160m

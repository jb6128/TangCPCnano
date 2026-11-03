//Copyright (C)2014-2023 Gowin Semiconductor Corporation.
//All rights reserved.
//File Title: IP file
//GOWIN Version: V1.9.9 Beta-2
//Part Number: GW2AR-LV18QN88C8/I7
//Device: GW2AR-18
//Device Version: C
//Created Time: Mon Sep 18 14:59:13 2023

// 27 MHz * 32 / 9 = 96.000 MHz (exact; VCO 768 MHz, PFD 3 MHz). Flash clock and, at the same time,
// the input of pll_160m (27 MHz cannot make 160 MHz in one PLL: it would need *160/27).
module flash_pll (clkout, lock, clkoutp, clkin);

output clkout;
output lock;
output clkoutp;
input clkin;

wire clkoutd_o;
wire clkoutd3_o;
wire gw_vcc;
wire gw_gnd;

assign gw_vcc = 1'b1;
assign gw_gnd = 1'b0;

rPLL #(
    .FCLKIN("27"),
    .DYN_IDIV_SEL("false"),
    .IDIV_SEL(8),
    .DYN_FBDIV_SEL("false"),
    .FBDIV_SEL(31),
    .DYN_ODIV_SEL("false"),
    .ODIV_SEL(8),
    .PSDA_SEL("1111"),
    .DYN_DA_EN("false"),
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
    .CLKOUTP(clkoutp),
    .CLKOUTD(clkoutd_o),
    .CLKOUTD3(clkoutd3_o),
    .RESET(gw_gnd),
    .RESET_P(gw_gnd),
    .CLKIN(clkin),
    .CLKFB(gw_gnd),
    .FBDSEL({gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd}),
    .IDSEL({gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd}),
    .ODSEL({gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd}),
    .PSDA({gw_gnd,gw_gnd,gw_gnd,gw_gnd}),
    .DUTYDA({gw_gnd,gw_gnd,gw_gnd,gw_gnd}),
    .FDLY({gw_vcc,gw_vcc,gw_vcc,gw_vcc})
);


endmodule //flash_pll

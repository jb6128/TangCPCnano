// oss_gowin_prims.v -- declarations of the Gowin primitives used by TangCPC, for the OSS CAD
// Suite flow only (tools/build_oss.sh; NOT part of the Gowin build.tcl).
//
// yosys-slang must know the parameters and ports of every module it elaborates (also for the
// defparam/#() overrides in src/pll/*). The attribute (* blackbox *) keeps the module empty;
// synth_gowin / nextpnr map the real primitives by name. Ports and parameters follow
// yosys' techlibs/gowin/cells_sim.v.
(* blackbox *)
module rPLL #(
    parameter FCLKIN = "100.0",
    parameter DYN_IDIV_SEL = "false",
    parameter IDIV_SEL = 0,
    parameter DYN_FBDIV_SEL = "false",
    parameter FBDIV_SEL = 0,
    parameter DYN_ODIV_SEL = "false",
    parameter ODIV_SEL = 8,
    parameter PSDA_SEL = "0000",
    parameter DYN_DA_EN = "false",
    parameter DUTYDA_SEL = "1000",
    parameter CLKOUT_FT_DIR = 1'b1,
    parameter CLKOUTP_FT_DIR = 1'b1,
    parameter CLKOUT_DLY_STEP = 0,
    parameter CLKOUTP_DLY_STEP = 0,
    parameter CLKFB_SEL = "internal",
    parameter CLKOUT_BYPASS = "false",
    parameter CLKOUTP_BYPASS = "false",
    parameter CLKOUTD_BYPASS = "false",
    parameter DYN_SDIV_SEL = 2,
    parameter CLKOUTD_SRC = "CLKOUT",
    parameter CLKOUTD3_SRC = "CLKOUT",
    parameter DEVICE = "GW1N-1"
) (
    output CLKOUT, output LOCK, output CLKOUTP, output CLKOUTD, output CLKOUTD3,
    input  CLKIN, input CLKFB, input RESET, input RESET_P,
    input  [5:0] FBDSEL, input [5:0] IDSEL, input [5:0] ODSEL,
    input  [3:0] PSDA, input [3:0] DUTYDA, input [3:0] FDLY
);
endmodule

(* blackbox *)
module CLKDIV #(
    parameter DIV_MODE = "2",
    parameter GSREN = "false"
) (
    output CLKOUT, input HCLKIN, input RESETN, input CALIB
);
endmodule

(* blackbox *)
module OSER10 #(
    parameter GSREN = "false",
    parameter LSREN = "true"
) (
    output Q,
    input D0, input D1, input D2, input D3, input D4,
    input D5, input D6, input D7, input D8, input D9,
    input FCLK, input PCLK, input RESET
);
endmodule

(* blackbox *)
module ELVDS_OBUF (
    input I, output O, output OB
);
endmodule

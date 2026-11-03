// tb_cpc_fdc.v -- decode test of cpc_fdc with a stub u765 (not run in the
// authoring sandbox: no simulator available). Run:
//   iverilog -g2012 -o tb_fdc tb/tb_cpc_fdc.v src/core/cpc_fdc.v && vvp tb_fdc
`timescale 1ns/1ps
module u765 #(parameter CYCLES=0, parameter SPECCY_SPEEDLOCK_HACK=0) (
    input clk_sys, input ce, input reset,
    input [1:0] ready, input [1:0] motor, input [1:0] available, input fast,
    input a0, input nRD, input nWR, input [7:0] din, output [7:0] dout,
    input [1:0] img_mounted, input img_wp, input [31:0] img_size,
    output [31:0] sd_lba, output [1:0] sd_rd, output [1:0] sd_wr, input sd_ack,
    input [8:0] sd_buff_addr, input [7:0] sd_buff_dout, output [7:0] sd_buff_din,
    input sd_buff_wr);
    assign dout = a0 ? 8'hA1 : 8'h80;      // data / main status stubs
    assign sd_lba = 0; assign sd_rd = 0; assign sd_wr = 0; assign sd_buff_din = 0;
endmodule

module tb_cpc_fdc;
    reg clk = 0; always #15.6 clk = ~clk;
    reg reset = 1, iorq = 0, rd = 0, wr = 0;
    reg [15:0] addr = 0; reg [7:0] din = 0;
    wire [7:0] dout; wire [1:0] motor, ready;
    integer errs = 0;

    cpc_fdc dut (.clk(clk), .reset(reset), .iorq(iorq), .rd(rd), .wr(wr),
        .addr(addr), .din(din), .dout(dout), .wprot(2'b00), .fast(1'b0),
        .img_mounted(2'b00), .img_size(32'd0), .sd_lba(), .sd_rd(), .sd_wr(),
        .sd_ack(1'b0), .sd_buff_addr(9'd0), .sd_buff_dout(8'd0),
        .sd_buff_wr(1'b0), .sd_buff_din(), .motor_o(motor), .ready_o(ready));

    task io_read(input [15:0] a, input [7:0] exp);
        begin addr = a; iorq = 1; rd = 1; #50;
            if (dout !== exp) begin errs = errs + 1;
                $display("FAIL read %h: %h expected %h", a, dout, exp); end
            iorq = 0; rd = 0; #50; end
    endtask
    task io_write(input [15:0] a, input [7:0] d);
        begin addr = a; din = d; iorq = 1; wr = 1; #50; iorq = 0; wr = 0; #50; end
    endtask

    initial begin
        #100 reset = 0;
        io_read (16'hFB7E, 8'h80);   // main status
        io_read (16'hFB7F, 8'hA1);   // data
        io_read (16'hFA7E, 8'hFF);   // motor port is write only
        io_read (16'h7F00, 8'hFF);   // gate array, not FDC
        io_read (16'hFF7F, 8'hFF);   // A10=1 -> no FDC
        io_write(16'hFA7E, 8'h01);
        if (motor !== 2'b11) begin errs = errs + 1; $display("FAIL motor on"); end
        io_write(16'hFA7E, 8'h00);
        if (motor !== 2'b00) begin errs = errs + 1; $display("FAIL motor off"); end
        io_write(16'hFB7F, 8'h01);   // FDC write must not touch the motor
        if (motor !== 2'b00) begin errs = errs + 1; $display("FAIL fdc write hit motor"); end
        $display(errs ? "tb_cpc_fdc: %0d ERRORS" : "tb_cpc_fdc: PASS", errs);
        $finish;
    end
endmodule

// tb_cpc_tape.v -- streaming test of cpc_tape with a stub tzxplayer and a stub SD.
// NOT RUN in the authoring sandbox (no simulator). Run:
//   iverilog -g2012 -o tb_tape tb/tb_cpc_tape.v src/core/cpc_tape.v && vvp tb_tape
`timescale 1ns/1ps

// stub of the VHDL player: requests every byte in turn (toggle protocol)
module tzxplayer #(parameter TZX_MS=0, NORMAL_PILOT_LEN=0, NORMAL_SYNC1_LEN=0,
    NORMAL_SYNC2_LEN=0, NORMAL_ZERO_LEN=0, NORMAL_ONE_LEN=0, NORMAL_PILOT_PULSES=0) (
    input clk, input ce, input restart_tape, input [7:0] host_tap_in,
    output reg tzx_req, input tzx_ack, output loop_start, output loop_next,
    output stop, output stop48k, output reg cass_read, input cass_motor);
    assign loop_start = 0; assign loop_next = 0; assign stop = 0; assign stop48k = 0;
    reg [7:0] got [0:2047]; integer n = 0;
    reg pend = 0;
    always @(posedge clk) begin
        if (restart_tape) begin tzx_req <= tzx_ack; cass_read <= 1; pend <= 0; n <= 0; end
        else if (!pend && tzx_req == tzx_ack) begin tzx_req <= ~tzx_ack; pend <= 1; end
        else if (pend && tzx_req == tzx_ack) begin got[n] <= host_tap_in; n <= n + 1; pend <= 0; end
    end
endmodule

module tb_cpc_tape;
    reg clk = 0; always #15.6 clk = ~clk;
    reg reset = 1, mounted = 0; reg [31:0] size = 0;
    wire sd_rd; wire [31:0] sd_lba;
    reg sd_busy = 0; reg [8:0] baddr = 0; reg [7:0] bdout = 0; reg bwr = 0;
    wire tape_in, present;

    cpc_tape dut (.clk(clk), .reset(reset), .img_mounted(mounted), .img_size(size),
        .motor(1'b1), .invert(1'b0), .tape_in(tape_in), .present(present), .sd_rd(sd_rd), .sd_lba(sd_lba),
        .sd_busy(sd_busy), .sd_buff_addr(baddr), .sd_buff_dout(bdout), .sd_buff_wr(bwr));

    // file content: byte i = (i*7+3) & 0xFF
    function [7:0] fbyte(input integer i); fbyte = (i*7 + 3) & 8'hFF; endfunction

    // SD stub: after sd_rd, go busy, stream 512 bytes, go idle
    integer k; reg [31:0] lba_l;
    always @(posedge clk) if (sd_rd && !sd_busy) begin
        lba_l = sd_lba; sd_busy <= 1;
        for (k = 0; k < 512; k = k + 1) begin
            @(posedge clk); baddr <= k; bdout <= fbyte(lba_l*512 + k); bwr <= 1;
        end
        @(posedge clk); bwr <= 0; repeat (20) @(posedge clk); sd_busy <= 0;
    end

    integer i, errs = 0;
    initial begin
        repeat (10) @(posedge clk); reset = 0;
        size = 1200;                       // 3 sectors, last one partial
        @(posedge clk); mounted = 1; @(posedge clk); mounted = 0;
        repeat (200000) @(posedge clk);
        if (dut.player.n !== 1200) begin errs = errs + 1; $display("FAIL: delivered %0d bytes, expected 1200", dut.player.n); end
        for (i = 0; i < 1200; i = i + 1)
            if (dut.player.got[i] !== fbyte(i)) begin errs = errs + 1;
                if (errs < 10) $display("FAIL byte %0d: %h expected %h", i, dut.player.got[i], fbyte(i)); end
        $display(errs ? "tb_cpc_tape: %0d ERRORS" : "tb_cpc_tape: PASS", errs);
        $finish;
    end
endmodule

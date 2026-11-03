// tb_kbd_uart_cpc.v -- self-checking testbench for src/mcu/kbd_uart_rx_cpc.v
//
//   iverilog -g2012 -o tb_uart tb/tb_kbd_uart_cpc.v src/mcu/kbd_uart_rx_cpc.v
//   vvp tb_uart               -> prints "PASS" or "FAIL ..." lines
//
// A small CLK_FREQ keeps the 1 s watchdog short: 1.152 MHz / 115200 = 10 clk
// per bit and a watchdog of 1.152 M clk (about 1 s of model time, fast to run).
`timescale 1ns/1ps

module tb_kbd_uart_cpc;

    localparam CLK_FREQ = 1_152_000;
    localparam BAUD     = 115_200;
    localparam CLK_NS   = 868;               // ~1.152 MHz
    localparam BIT_CLKS = CLK_FREQ / BAUD;

    reg clk = 1'b0;
    always #(CLK_NS/2) clk = ~clk;

    reg  rst_n, rx;
    wire [79:0] ext_key;
    wire [7:0]  joy0, joy1;
    wire        alive;

    kbd_uart_rx_cpc #(.CLK_FREQ(CLK_FREQ), .BAUD(BAUD)) dut (
        .clk(clk), .rst_n(rst_n), .rx(rx),
        .ext_key(ext_key), .joy_state0(joy0), .joy_state1(joy1), .link_alive(alive)
    );

    integer errors = 0;
    task check; input cond; input [8*40-1:0] what;
        begin if (!cond) begin errors = errors + 1; $display("FAIL: %0s", what); end end
    endtask

    task send; input [7:0] b; integer k;
        begin
            rx = 1'b0; repeat (BIT_CLKS) @(posedge clk);           // start
            for (k = 0; k < 8; k = k + 1) begin
                rx = b[k]; repeat (BIT_CLKS) @(posedge clk);       // LSB first
            end
            rx = 1'b1; repeat (BIT_CLKS) @(posedge clk);           // stop
        end
    endtask

    function [79:0] cell; input integer row; input integer bit_;
        begin cell = 80'd1 << (row * 8 + bit_); end
    endfunction

    // cell byte as sent by the firmware
    function [7:0] cb; input integer row; input integer bit_;
        begin cb = 8'h80 | (bit_ << 4) | row; end
    endfunction

    integer r;

    initial begin
        rst_n = 1'b0; rx = 1'b1;
        repeat (10) @(posedge clk); rst_n = 1'b1; repeat (10) @(posedge clk);

        check(ext_key === 80'd0, "idle: nothing pressed");

        send(8'h90); send(cb(8,5));                                // MAKE A (row 8 bit 5)
        repeat (4) @(posedge clk);
        check(ext_key === cell(8,5), "MAKE A -> key[8][5]");
        send(8'h90); send(cb(2,5));                                // MAKE Shift
        repeat (4) @(posedge clk);
        check(ext_key === (cell(8,5) | cell(2,5)), "A + Shift");
        send(8'hA0); send(cb(8,5));                                // BREAK A
        repeat (4) @(posedge clk);
        check(ext_key === cell(2,5), "BREAK A leaves Shift");
        send(8'hA0); send(cb(2,5));
        repeat (4) @(posedge clk);
        check(ext_key === 80'd0, "all released");

        send(8'h90); send(cb(9,7));                                // DEL, top row
        repeat (4) @(posedge clk);
        check(ext_key === cell(9,7), "MAKE DEL -> key[9][7]");
        send(8'hA0); send(cb(9,7));

        // row 10 does not exist on a CPC: accepted but never exported
        send(8'h90); send(cb(10,3));
        repeat (4) @(posedge clk);
        check(ext_key === 80'd0, "row 10 ignored");
        send(8'hA0); send(cb(10,3));

        // resync: 0xFE, 11 active-low rows, 0xFF. Row 1 bit 0 and row 5 bit 7 pressed.
        send(8'hFE);
        for (r = 0; r < 11; r = r + 1)
            send(r == 1 ? 8'hFE : r == 5 ? 8'h7F : 8'hFF);          // 0xFE as DATA must load
        send(8'hFF);
        repeat (4) @(posedge clk);
        check(ext_key === (cell(1,0) | cell(5,7)), "resync loads matrix (incl. 0xFE/0xFF data)");

        // version announce and mouse frames are consumed, do not disturb the matrix
        send(8'hC0); send(8'h13);
        send(8'hD0); send(8'h05); send(8'hFB); send(8'h01);
        repeat (4) @(posedge clk);
        check(ext_key === (cell(1,0) | cell(5,7)), "0xC0 / 0xD0 frames consumed");

        // joysticks
        send(8'hB0); send(8'h00); send(8'h21);
        send(8'hB0); send(8'h01); send(8'h08);
        repeat (4) @(posedge clk);
        check(joy0 === 8'h21 && joy1 === 8'h08, "joystick ports 0 and 1");

        check(alive === 1'b1, "link alive while bytes arrive");

        // watchdog: ~1 s of silence releases everything
        repeat (CLK_FREQ + 100) @(posedge clk);
        check(ext_key === 80'd0 && joy0 === 8'h00 && joy1 === 8'h00, "watchdog releases keys and joysticks");
        check(alive === 1'b0, "link_alive drops");

        if (errors == 0) $display("PASS: tb_kbd_uart_cpc");
        else             $display("FAIL: tb_kbd_uart_cpc, %0d error(s)", errors);
        $finish;
    end

    initial begin #3_000_000_000; $display("FAIL: tb_kbd_uart_cpc TIMEOUT"); $finish; end
endmodule

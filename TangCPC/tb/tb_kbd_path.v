// tb_kbd_path.v -- self-checking end-to-end testbench for the BL616 keyboard path:
//   SPI master model -> mcu_spi -> mcu_hid -> hid_cpc_matrix
//
//   iverilog -g2012 -o tb_kbd tb/tb_kbd_path.v src/mcu/mcu_spi.v \
//            src/mcu/mcu_hid.v src/mcu/hid_cpc_matrix.v
//   vvp tb_kbd               -> prints "PASS" or "FAIL ..." lines
//
// SPI is MODE1 like the BL616 companion: data set up on the rising edge of
// SCLK, sampled on the falling edge. Frame = target id, command, data bytes.
//
// NOTE: src/mcu/mcu_spi.v (MiSTeryNano original) toggles spi_data_in_ready
// without initialising it. Real hardware does not care, in simulation it stays
// X forever, so this testbench initialises it at time 0.
`timescale 1ns/1ps

module tb_kbd_path;

    reg clk = 1'b0;
    always #15.625 clk = ~clk;                 // 32 MHz = clk32

    reg  por;                                  // active-high, as in top.sv
    reg  spi_ss, spi_clk, spi_din;
    wire spi_dout;

    wire        mcu_sys_strobe, mcu_hid_strobe, mcu_osd_strobe, mcu_sdc_strobe;
    wire        mcu_start;
    wire [7:0]  mcu_dout;
    wire [7:0]  hid_dout;

    wire [127:0] hid_kb;
    wire [7:0]   joy0, joy1;
    wire [79:0]  matrix;

    mcu_spi u_spi (
        .clk(clk), .reset(por),
        .spi_io_ss(spi_ss), .spi_io_clk(spi_clk), .spi_io_din(spi_din), .spi_io_dout(spi_dout),
        .mcu_sys_strobe(mcu_sys_strobe), .mcu_hid_strobe(mcu_hid_strobe),
        .mcu_osd_strobe(mcu_osd_strobe), .mcu_sdc_strobe(mcu_sdc_strobe),
        .mcu_start(mcu_start),
        .mcu_sys_din(8'h00), .mcu_hid_din(hid_dout), .mcu_osd_din(8'h00), .mcu_sdc_din(8'h00),
        .mcu_dout(mcu_dout)
    );

    mcu_hid u_hid (
        .clk(clk), .rst_n(~por),
        .data_in_strobe(mcu_hid_strobe), .data_in_start(mcu_start), .data_in(mcu_dout),
        .data_out(hid_dout),
        .keyboard(hid_kb), .joystick0(joy0), .joystick1(joy1)
    );

    hid_cpc_matrix u_map (.kb(hid_kb), .key(matrix));

    integer errors = 0;
    integer i;

    task check;
        input        cond;
        input [8*40-1:0] what;
        begin
            if (!cond) begin
                errors = errors + 1;
                $display("FAIL: %0s", what);
            end
        end
    endtask

    task send_byte;
        input [7:0] b;
        integer k;
        begin
            for (k = 7; k >= 0; k = k - 1) begin
                spi_din = b[k];
                #200 spi_clk = 1'b1;           // setup edge
                #200 spi_clk = 1'b0;           // sample edge
            end
            #400;                              // inter-byte gap
        end
    endtask

    task frame_begin; begin spi_ss = 1'b0; #400; end endtask
    task frame_end;   begin #2000; spi_ss = 1'b1; #2000; end endtask

    // one keyboard event: target 1, cmd 1, { release, usb_code }
    task key_event;
        input [6:0] code;
        input       release;
        begin
            frame_begin;
            send_byte(8'd1);                   // target: HID
            send_byte(8'd1);                   // cmd: keyboard
            send_byte({release, code});
            frame_end;
        end
    endtask

    function [79:0] cell;
        input integer row;
        input integer bit_;
        begin cell = 80'd1 << (row * 8 + bit_); end
    endfunction

    initial begin
        u_spi.spi_data_in_ready = 1'b0;        // see note in header
        por = 1'b1; spi_ss = 1'b1; spi_clk = 1'b0; spi_din = 1'b0;
        #200 por = 1'b0;
        #1000;

        check(hid_kb === 128'd0 && matrix === 80'd0, "idle after reset");

        // press and release 'A' (HID 0x04) -> CPC row 8 bit 5
        key_event(7'h04, 1'b0);
        check(hid_kb[7'h04] === 1'b1, "A press seen by mcu_hid");
        check(matrix === cell(8,5),   "A -> CPC key[8][5]");
        key_event(7'h04, 1'b1);
        check(hid_kb[7'h04] === 1'b0, "A release seen by mcu_hid");
        check(matrix === 80'd0,       "matrix cleared after release");

        // Shift (companion-remapped modifier 0x69) + '1' -> two CPC keys at once
        key_event(7'h69, 1'b0);
        key_event(7'h1E, 1'b0);
        check(matrix === (cell(2,5) | cell(8,0)), "Shift + 1");
        key_event(7'h1E, 1'b1);
        key_event(7'h69, 1'b1);
        check(matrix === 80'd0, "Shift + 1 released");

        // frames for another target must not touch the keyboard (target 2 = OSD)
        frame_begin;
        send_byte(8'd2); send_byte(8'd1); send_byte(8'h05);
        frame_end;
        check(hid_kb === 128'd0, "target 2 ignored by mcu_hid");

        // joystick frame: target 1, cmd 3, then 5 data bytes (device, digital, ax, ay, extra)
        frame_begin;
        send_byte(8'd1); send_byte(8'd3);
        send_byte(8'd0); send_byte(8'h31); send_byte(8'h80); send_byte(8'h80); send_byte(8'h00);
        frame_end;
        check(joy0 === 8'h31, "joystick 0 digital byte");
        check(joy1 === 8'h00, "joystick 1 untouched");

        frame_begin;
        send_byte(8'd1); send_byte(8'd3);
        send_byte(8'd1); send_byte(8'h08); send_byte(8'h80); send_byte(8'h80); send_byte(8'h00);
        frame_end;
        check(joy1 === 8'h08, "joystick 1 digital byte");
        check(joy0 === 8'h31, "joystick 0 still held");

        // reset releases everything
        key_event(7'h2C, 1'b0);                // SPACE
        check(matrix === cell(5,7), "SPACE -> CPC key[5][7]");
        por = 1'b1; #200; por = 1'b0; #200;
        check(hid_kb === 128'd0 && joy0 === 8'h00 && joy1 === 8'h00, "reset clears keys and joysticks");

        if (errors == 0) $display("PASS: tb_kbd_path");
        else             $display("FAIL: tb_kbd_path, %0d error(s)", errors);
        $finish;
    end

    initial begin
        #20_000_000;
        $display("FAIL: tb_kbd_path TIMEOUT");
        $finish;
    end

endmodule

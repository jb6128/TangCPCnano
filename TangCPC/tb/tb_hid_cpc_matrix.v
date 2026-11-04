// tb_hid_cpc_matrix.v -- self-checking testbench for src/mcu/hid_cpc_matrix.v
//
//   iverilog -g2012 -o tb_matrix tb/tb_hid_cpc_matrix.v src/mcu/hid_cpc_matrix.v
//   vvp tb_matrix            -> prints "PASS" or "FAIL ..." lines
`timescale 1ns/1ps

module tb_hid_cpc_matrix;

    reg  [127:0] kb;
    wire [79:0]  key;
    integer      errors;

    hid_cpc_matrix dut (.kb(kb), .key(key));

    function [79:0] cell;            // one-hot mask of CPC key (row, bit)
        input integer row;
        input integer bit_;
        begin
            cell = 80'd1 << (row * 8 + bit_);
        end
    endfunction

    // press exactly one HID code; expect exactly `mask` in the matrix
    task press_one;
        input [6:0]  code;
        input [79:0] mask;
        begin
            kb = 128'd0;
            kb[code] = 1'b1;
            #1;
            if (key !== mask) begin
                errors = errors + 1;
                $display("FAIL: HID %h -> key=%h, expected %h", code, key, mask);
            end
        end
    endtask

    initial begin
        errors = 0;
        kb     = 128'd0;
        #1;

        // idle: nothing pressed
        if (key !== 80'd0) begin errors = errors + 1; $display("FAIL: idle matrix not 0"); end

        // letters / digits / space on different rows
        press_one(7'h04, cell(8,5));                    // A
        press_one(7'h1D, cell(8,7));                    // Z
        press_one(7'h13, cell(3,3));                    // P
        press_one(7'h1E, cell(8,0));                    // 1
        press_one(7'h27, cell(4,0));                    // 0
        press_one(7'h2C, cell(5,7));                    // SPACE
        press_one(7'h29, cell(8,2));                    // ESC
        press_one(7'h28, cell(2,2));                    // ENTER -> RETURN
        press_one(7'h58, cell(2,2));                    // KP Enter -> RETURN
        press_one(7'h2A, cell(9,7));                    // Backspace -> DEL
        press_one(7'h39, cell(8,6));                    // CAPS

        // cursors
        press_one(7'h52, cell(0,0));                    // Up
        press_one(7'h4F, cell(0,1));                    // Right
        press_one(7'h51, cell(0,2));                    // Down
        press_one(7'h50, cell(1,0));                    // Left

        // modifiers
        press_one(7'h69, cell(2,5));                    // LShift
        press_one(7'h6D, cell(2,5));                    // RShift
        press_one(7'h68, cell(2,7));                    // LCtrl
        press_one(7'h6C, cell(2,7));                    // RCtrl

        // function keys and CPC keypad
        press_one(7'h3A, cell(1,5));                    // F1
        press_one(7'h59, cell(1,5));                    // KP1 -> F1
        press_one(7'h43, cell(1,7));                    // F10 -> F0
        press_one(7'h62, cell(1,7));                    // KP0 -> F0
        press_one(7'h42, cell(0,3));                    // F9

        // keypad + and * synthesise Shift
        press_one(7'h57, cell(3,4) | cell(2,5));        // KP+  = Shift + ;
        press_one(7'h55, cell(3,5) | cell(2,5));        // KP*  = Shift + :

        // second keys for COPY and ]: left Alt / Mac Option, ISO '#' key
        press_one(7'h6A, cell(1,1));                    // LAlt = COPY
        press_one(7'h49, cell(1,1));                    // Insert = COPY
        press_one(7'h32, cell(2,3));                    // ISO '#' = ]
        press_one(7'h31, cell(2,3));                    // US backslash = ]

        // keys without a CPC counterpart do nothing
        press_one(7'h65, 80'd0);                        // Application
        press_one(7'h6E, 80'd0);                        // RAlt (AltGr)
        press_one(7'h6B, 80'd0);                        // LGUI

        // two keys at once: both appear, nothing else
        kb = 128'd0; kb[7'h04] = 1'b1; kb[7'h05] = 1'b1; #1;   // A + B
        if (key !== (cell(8,5) | cell(6,6))) begin
            errors = errors + 1; $display("FAIL: A+B -> %h", key);
        end

        // release again -> matrix returns to 0
        kb = 128'd0; #1;
        if (key !== 80'd0) begin errors = errors + 1; $display("FAIL: release did not clear"); end

        // all HID codes down: joystick lines of row 9 (bits 0..6) must stay 0, DEL = 1
        kb = {128{1'b1}}; #1;
        if (key[9*8+6 : 9*8] !== 7'd0) begin
            errors = errors + 1; $display("FAIL: row 9 joystick lines not 0: %b", key[9*8+6 : 9*8]);
        end
        if (key[9*8+7] !== 1'b1) begin errors = errors + 1; $display("FAIL: DEL not set"); end

        if (errors == 0) $display("PASS: tb_hid_cpc_matrix");
        else             $display("FAIL: tb_hid_cpc_matrix, %0d error(s)", errors);
        $finish;
    end

endmodule

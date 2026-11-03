//============================================================================
// mcu_hid.v
//   HID target (SPI target id 1) of the BL616 FPGA-Companion.
//
//   Derived from MSXnano/fpga/src/usb/hid.v (BL616 era, working on the same
//   Tang Nano 20K). Differences:
//     - renamed to mcu_hid (src/rtl/hid.sv already defines module `hid`)
//     - mouse / DB9 commands dropped (the CPC core has no use for them yet);
//       CMD 4 (DB9 read) answers 0x00 so the companion sees "nothing pressed"
//     - Verilog-2001, async active-low reset (rst_n), non-blocking everywhere
//
//   Byte protocol (after mcu_spi stripped the target id):
//     data_in_start=1 : this byte is the command
//     CMD 0 (status)  : reads back 0x01, 0x00
//     CMD 1 (keyboard): 1 byte  = { release, usb_hid_code[6:0] }
//                       release = 1 -> key up. keyboard[code] = 1 while down.
//                       USB modifiers E0..E7 arrive remapped to 0x68..0x6F
//                       (code - 0x78), as seen by MSXnano's usb_keyboard_msx.
//     CMD 3 (joystick): byte 0 = device (0/1), byte 1 = digital state,
//                       active-high: bit0 R, bit1 L, bit2 D, bit3 U, bit4.. fire
//     CMD 4 (get DB9) : always 0x00
//
//   Single clock domain: clk is the same clock as mcu_spi's local domain.
//============================================================================
`default_nettype none

module mcu_hid (
    input  wire         clk,
    input  wire         rst_n,

    input  wire         data_in_strobe,
    input  wire         data_in_start,
    input  wire [7:0]   data_in,
    output reg  [7:0]   data_out,

    output reg  [127:0] keyboard,    // [usb_hid_code] = 1 while pressed
    output reg  [7:0]   joystick0,   // active-high, see header
    output reg  [7:0]   joystick1
);

    reg [3:0] state;
    reg [7:0] command;
    reg [7:0] device;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= 4'd0;
            command   <= 8'd0;
            device    <= 8'd0;
            data_out  <= 8'd0;
            keyboard  <= 128'd0;
            joystick0 <= 8'd0;
            joystick1 <= 8'd0;
        end else if (data_in_strobe) begin
            if (data_in_start) begin
                state   <= 4'd0;
                command <= data_in;
            end else begin
                if (state != 4'd15)
                    state <= state + 4'd1;

                case (command)
                    8'd0: begin                       // status
                        if (state == 4'd0) data_out <= 8'h01;
                        if (state == 4'd1) data_out <= 8'h00;
                    end

                    8'd1: begin                       // keyboard
                        if (state == 4'd0)
                            keyboard[data_in[6:0]] <= ~data_in[7];
                    end

                    8'd3: begin                       // joystick
                        if (state == 4'd0) device <= data_in;
                        if (state == 4'd1) begin
                            if (device == 8'd0)      joystick0 <= data_in;
                            else if (device == 8'd1) joystick1 <= data_in;
                        end
                    end

                    8'd4: data_out <= 8'h00;          // DB9: nothing connected

                    default: ;
                endcase
            end
        end
    end

endmodule

`default_nettype wire

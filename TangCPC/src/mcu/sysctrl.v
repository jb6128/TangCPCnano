/*
    sysctrl.v -- TangCPC

    System control target (SPI target id 0) of the BL616 FPGA-Companion.

    Based on the generic sysctrl.v of MiSTle-Dev/C64Nano (GPL-3.0) and the
    MiSTeryNano original (Till Harbaum). The Atari ST / C64 specific
    configuration variables were replaced by the CPC ones below. The OSD menu
    itself is the gzip'ed XML in cpc_xml.hex (source: cpc.xml), read by the
    companion with CMD 8.

    Configuration variables (CMD 4, id character -> value):
      "R"  reset: run(0), reset(1), cold boot(3)
      "P"  floppy write protection: none(0), A(1), B(2), both(3)
      "F"  FDC speed: normal(0), fast(1)
      "L"  external ROM slot being edited (0..31)
  "K"  clear the selected ROM slot (level, rising edge acts)
  "C"  V lock: 0 = Off (HDMI frame 624 rows, free running), 1 = On (row counter locked to the CPC VSync, frame 624 rows +- small corrections), 2 = Follow (On, and the HDMI frame length = CPC frame length while that is stable)
  "E"  V lock / V pos speed: largest change of the HDMI frame length per frame, 1, 2, 4, 8, 16 rows (0..4, default 2)
  "Z"  vertical picture position 0..15 (label -8..+7, 16 HDMI rows per step, default 8 = 0)
  "Q"  remove the ROM set (level, rising edge acts)
  "A"  internal AMSDOS ROM (slot 7) visible: off(0), on(1)
  "O"  ROM0: internal BASIC (0) / external ROM in slot 0 (1)
  "o"  ROM7: internal DOS (0) / external ROM in slot 7 (1)
  "W"  Z80 waits for SDRAM data: off(0, test only), on(1)
  "c"  CRTC type: 0 (HD6845S / UM6845, default) or 1 (UM6845R); a change triggers a reset
  "H"  horizontal picture position 0..8 (h_offset = 192 + 32*value, default 4)
  "T"  invert the tape input signal: normal(0), inverted(1)
  "U"  tape protection: off(0, SAVE records into the mounted tape image), on(1, default)
  "X"  HDMI active width: 768 px (0, default, = RGB output window), 800 px (1, wider border)

    As in the C64Nano the machine is held in reset after power-up until the
    MCU releases it ("R"=0), or after a timeout of about 3 s if no companion
    answers, so the CPC still boots without a BL616 firmware.
*/

module sysctrl (
  input             clk,
  input             reset,          // active high

  input             data_in_strobe,
  input             data_in_start,
  input [7:0]       data_in,
  output reg [7:0]  data_out,

  // interrupt interface
  output            int_out_n,
  input [7:0]       int_in,
  output reg [7:0]  int_ack,

  input [1:0]       buttons,        // S0 and S1 buttons on Tang Nano 20k

  output reg [1:0]  leds,           // two leds can be controlled from the MCU
  output reg [23:0] color,          // 24 bit color e.g. for the ws2812

  // IO port interface (serial port, not implemented by the CPC core)
  input [31:0]      port_status,
  input [7:0]       port_out_available,
  output reg        port_out_strobe,
  input [7:0]       port_out_data,
  input [7:0]       port_in_available,
  output reg        port_in_strobe,
  output reg [7:0]  port_in_data,

  // values that can be configured by the user
  output [1:0]      system_reset,
  output reg [1:0]  system_floppy_wprot,
  output reg        system_fdc_fast,
  output reg [4:0]  system_rom_slot,
  output reg        system_rom_clear,
  output reg        system_rom_clearset,
  output reg  [1:0] system_amsdos,    // OSD id A: 0 = Off, 1 = On, 2 = Auto (On for 6128 / 664, Off for 464; default)
  output reg        system_rom0_ext, // OSD id O: 0 = internal BASIC ROM (slot 0), 1 = external ROM 0 when one is loaded (at the next reset)
  output reg        system_rom7_ext, // OSD id o: 0 = internal DOS ROM (slot 7), 1 = external ROM 7 when one is loaded
  output reg        system_waitgate,
  output reg        system_hlock,
  output reg        system_firesw,   // OSD id J: 1 = swap the two DB9 fire buttons
  output reg        system_numpad,   // OSD id I: 1 = keypad 1-9 act as joystick (8/2/4/6 directions, 7/9/1/3 diagonals, 5 = Fire 2 / X, 0 = Fire 1 / Z)
  output reg        system_rgbout,   // OSD id G: 1 = drive the 15 kHz RGB/CSYNC pins, 0 = high-Z (default)
  output reg  [1:0] system_rgbph,    // OSD id p: RGB phase = delay of the RGB lines against CSYNC, 0 = 0 ns (default), 1 = +16 ns, 2 = +31 ns, 3 = +47 ns
  output reg  [1:0] system_vlock,    // OSD id C: V lock 0 = Off, 1 = On (row counter locked to the CPC VSync), 2 = Follow (On + HDMI frame length = CPC frame length when that is stable)
  output reg  [2:0] system_vrate,    // OSD id E: V lock / V pos: max. change of the HDMI frame length per frame, 1 << value rows (0..4, default 2 = 4 rows)
  output reg  [3:0] system_vpos,     // OSD id Z: vertical position, 0..15 (8 = as before), 16 HDMI rows per step
  output reg  [2:0] system_volume,   // OSD id V: HDMI audio volume 0..4 = 0 / 25 / 50 / 75 / 100 % (default 4)
  output reg        system_tapesnd,  // OSD id N: 1 = tape signal audible on the HDMI audio
  output reg        system_snap,     // OSD id d (Debug / Snapshot): 1 = pin 48 carries the snapshot UART and S1+S2 take a RAM snapshot, 0 = BL616 console bridge (default)
  output reg  [1:0] system_uart,     // OSD id u (Debug / UART): baud rate of the snapshot UART, 0 = 115200, 1 = 460800, 2 = 921600 (default), 3 acts like 2
  output reg        system_mono,     // OSD id M: 0 = colour, 1 = green monitor
  output reg        system_crtc,     // OSD id c: CRTC type 0 / 1 (default 1)
  output reg  [1:0] system_model,    // OSD id Y: 0 = CPC 6128 (OS, BASIC 1.1, 128K), 1 = CPC 664 (OS, BASIC 1.1, 64K), 2 = CPC 464 (OS, BASIC 1.0, 64K); effective at the next reset
  output reg        system_wide,     // OSD id D: 0 = 4:3, 1 = full screen (16:9 flag)
  output reg  [3:0] system_hpos,
  output reg        system_tape_inv,
  output reg        system_tape_prot, // OSD id U: 1 = tape protected (default), 0 = SAVE records into the tape image
  output reg        system_w800      // OSD id X: 0 = 768 px active width (default), 1 = 800 px
);

reg [3:0] state;
reg [7:0] command;
reg [7:0] id;

// reverse data byte for rgb
wire [7:0] data_in_rev = { data_in[0], data_in[1], data_in[2], data_in[3],
                           data_in[4], data_in[5], data_in[6], data_in[7] };

// coldboot flag and system interrupt
reg coldboot = 1'b1;
reg sys_int  = 1'b1;

// registers to report button interrupts
reg [1:0] buttonsD, buttonsD2;
reg       buttons_irq_enable;

// the system control interrupt or any other interrupt (sdc, hid, ...)
// activates the interrupt line to the MCU by pulling it low
assign int_out_n = (int_in != 8'h00 || sys_int) ? 1'b0 : 1'b1;

// by default the system is in reset
reg [1:0] main_reset = 2'b11;
assign system_reset = main_reset;

// 96_000_000 clk32 cycles = 3 s
reg [31:0] main_reset_timeout;

reg       port_out_availableD;
reg [7:0] port_cmd;
reg [7:0] port_index;

// menu rom: gzip'ed cpc.xml
//   gzip -n -c cpc.xml | xxd -c1 -p > cpc_xml.hex   (see tools/gen_menu_hex.py)
reg [11:0] menu_rom_addr;
reg [7:0]  menu_rom_data;
reg [7:0]  cpc_xml [0:2047];
initial $readmemh("cpc_xml.hex", cpc_xml);

always @(posedge clk)
  menu_rom_data <= cpc_xml[menu_rom_addr[10:0]];

always @(posedge clk) begin
   if(reset) begin
      state <= 4'd0;
      leds <= 2'b00;
      color <= 24'h000000;

      main_reset <= 2'b11;
      main_reset_timeout <= 32'd96_000_000;

      buttons_irq_enable <= 1'b1;
      int_ack <= 8'h00;
      coldboot <= 1'b1;
      sys_int <= 1'b1;

      port_out_strobe <= 1'b0;
      port_in_strobe <= 1'b0;

      // sane defaults, the MCU will override these early
      system_floppy_wprot <= 2'b00;
      system_fdc_fast <= 1'b0;
      system_rom_slot <= 5'd0;
      system_rom_clear <= 1'b0;
      system_rom_clearset <= 1'b0;
      system_amsdos <= 2'd2;
      system_rom0_ext <= 1'b0;
      system_rom7_ext <= 1'b0;
      system_waitgate <= 1'b1;
      system_hlock <= 1'b1;
      system_mono <= 1'b0;
      system_snap <= 1'b0;
      system_uart <= 2'd2;
      system_tapesnd <= 1'b0;
      system_volume <= 3'd4;
      system_vpos <= 4'd8;
      system_vlock <= 2'd0;
      system_vrate <= 3'd2;
      system_rgbout <= 1'b0;
      system_rgbph <= 2'd0;
      system_firesw <= 1'b0;
      system_numpad <= 1'b0;
      system_wide <= 1'b0;
      system_model <= 2'd0;
      system_crtc <= 1'b1;
      system_hpos <= 4'd8;
      system_tape_inv <= 1'b0;
      system_tape_prot <= 1'b1;
      system_w800 <= 1'b0;
   end else begin
      buttonsD <= buttons;
      buttonsD2 <= buttonsD;

      int_ack <= 8'h00;
      port_out_strobe <= 1'b0;
      port_in_strobe <= 1'b0;

      // iack bit 0 acknowledges the coldboot notification
      if(int_ack[0]) sys_int <= 1'b0;

      // release main reset after the timeout
      if(main_reset_timeout != 32'd0) begin
         main_reset_timeout <= main_reset_timeout - 32'd1;
         if(main_reset_timeout == 32'd1)
            main_reset <= 2'b00;
      end

      // (further) data has just become available, so raise interrupt
      port_out_availableD <= (port_out_available != 8'd0);
      if((port_out_available != 8'd0) && !port_out_availableD)
         sys_int <= 1'b1;

      // monitor buttons for changes and raise interrupt
      if(buttons_irq_enable && (buttonsD2 != buttonsD)) begin
         sys_int <= 1'b1;
         buttons_irq_enable <= 1'b0;
      end

      if(data_in_strobe) begin
         if(data_in_start) begin
            state <= 4'd0;
            command <= data_in;
            menu_rom_addr <= 12'd0;
            data_out <= 8'h00;
         end else begin
            if(state != 4'd15) state <= state + 4'd1;

            // CMD 0: status data
            if(command == 8'd0) begin
               if(state == 4'd0) data_out <= 8'h5c;
               if(state == 4'd1) data_out <= 8'h42;
               if(state == 4'd2) data_out <= 8'h00;   // core id 0 = generic core
            end

            // CMD 1: two MCU controlled LEDs
            if(command == 8'd1) begin
               if(state == 4'd0) leds <= data_in[1:0];
            end

            // CMD 2: 24 bit color value
            if(command == 8'd2) begin
               if(state == 4'd0) color[15: 8] <= data_in_rev;
               if(state == 4'd1) color[ 7: 0] <= data_in_rev;
               if(state == 4'd2) color[23:16] <= data_in_rev;
            end

            // CMD 3: return button state
            if(command == 8'd3) begin
               data_out <= { 6'b000000, buttons };
               buttons_irq_enable <= 1'b1;
            end

            // CMD 4: config values (set by the user via OSD)
            if(command == 8'd4) begin
               if(state == 4'd0) id <= data_in;

               if(state == 4'd1) begin
                  if(id == "R") begin
                     main_reset <= data_in[1:0];
                     // cancel the power-up timeout once the MCU is active
                     main_reset_timeout <= 32'd0;
                  end
                  if(id == "P") system_floppy_wprot <= data_in[1:0];
                  if(id == "F") system_fdc_fast <= data_in[0];
                  if(id == "L") system_rom_slot <= data_in[4:0];
                  if(id == "K") system_rom_clear <= data_in[0];
                  if(id == "Q") system_rom_clearset <= data_in[0];
                  if(id == "A") system_amsdos <= data_in[1:0];
                  if(id == "O") system_rom0_ext <= data_in[0];
                  if(id == "o") system_rom7_ext <= data_in[0];
                  if(id == "W") system_waitgate <= data_in[0];
                  if(id == "S") system_hlock <= data_in[0];
                  if(id == "M") system_mono <= data_in[0];
                  if(id == "d") system_snap <= data_in[0];
                  if(id == "u") system_uart <= data_in[1:0];
                  if(id == "N") system_tapesnd <= data_in[0];
                  if(id == "V") system_volume <= (data_in[2:0] > 3'd4) ? 3'd4 : data_in[2:0];
                  if(id == "G") system_rgbout <= data_in[0];
                  if(id == "p") system_rgbph <= data_in[1:0];
                  if(id == "J") system_firesw <= data_in[0];
                  if(id == "I") system_numpad <= data_in[0];
                  if(id == "D") system_wide <= data_in[0];
                  if(id == "Y") system_model <= data_in[1:0];
                  if(id == "c") system_crtc <= data_in[0];
                  if(id == "H") system_hpos <= data_in[3:0];
                  if(id == "Z") system_vpos <= data_in[3:0];
                  if(id == "C") system_vlock <= data_in[1:0];   // 3 acts like 2 (Follow)
                  if(id == "E") system_vrate <= (data_in[2:0] > 3'd4) ? 3'd4 : data_in[2:0];
                  if(id == "T") system_tape_inv <= data_in[0];
                  if(id == "U") system_tape_prot <= data_in[0];
                  if(id == "X") system_w800 <= data_in[0];
               end
            end

            // CMD 5: interrupt control
            if(command == 8'd5) begin
               if(state == 4'd0) int_ack <= data_in;
               data_out <= { int_in[7:1], sys_int };
            end

            // CMD 6: read system interrupt source
            if(command == 8'd6) begin
               // bit0: coldboot, bit1: port data, bit2: buttons changed
               data_out <= { 5'b00000, !buttons_irq_enable, (port_out_available != 8'd0), coldboot };
               if(state == 4'd0) coldboot <= 1'b0;
            end

            // CMD 7: port command (serial port, not implemented)
            if(command == 8'd7) begin
               if(state == 4'd0) begin
                  port_cmd <= data_in;
                  data_out <= 8'd1;            // number of ports
               end else if(state == 4'd1) begin
                  port_index <= data_in;
                  data_out <= 8'd0;            // port type 0 = serial
               end else begin
                  if(port_cmd == 8'd0 && port_index == 8'd0) begin
                     if(state == 4'd2)       data_out <= port_out_available;
                     else if(state == 4'd3)  data_out <= port_in_available;
                     else if(state == 4'd4)  data_out <= port_status[31:24];
                     else if(state == 4'd5)  data_out <= port_status[23:16];
                     else if(state == 4'd6)  data_out <= port_status[15:8];
                     else if(state == 4'd7)  data_out <= port_status[7:0];
                     else                    data_out <= 8'h00;
                  end
                  else if(port_cmd == 8'd1 && port_index == 8'd0) begin
                     data_out <= port_out_data;
                     port_out_strobe <= data_in[0];
                  end
                  else if(port_cmd == 8'd2 && port_index == 8'd0) begin
                     port_in_data <= data_in;
                     port_in_strobe <= 1'b1;
                  end
               end
            end

            // CMD 8: read (menu) config
            if(command == 8'd8) begin
               data_out <= menu_rom_data;
               menu_rom_addr <= menu_rom_addr + 12'd1;
            end
         end
      end
   end
end

endmodule

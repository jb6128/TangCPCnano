//============================================================================
// hid_cpc_matrix.v
//   USB HID usage codes (from mcu_hid.keyboard) -> Amstrad CPC keyboard matrix.
//
//   Replaces the MSX translator (MSXnano usb_keyboard_msx.vhd). The CPC matrix
//   has 10 rows (0..9, selected by PPI port C[3:0]) x 8 columns, read back
//   through PSG register 14. Output is ACTIVE-HIGH (1 = key down) and flattened:
//       key[8*row + bit]
//   because src/rtl/hid.sv ORs it into its own `key[Y]` before inverting.
//
//   Mapping is POSITIONAL, i.e. the physical key position of a US PC keyboard,
//   identical to the Amstrad_MiSTer PS/2 table in src/rtl/hid.sv (checked by
//   tools/check_hid_cpc_map.py). So a PC key types what the CPC legend shows,
//   not what the PC legend shows (e.g. US '"' = Shift+' gives CPC Shift+';').
//
//   Numeric keypad follows MiSTer with keypad_mod = 0: KP 0..9 are the CPC's
//   own F0..F9 keypad, KP '+' and '*' synthesise Shift + ';' / ':'.
//
//   Row 9 bits 0..6 are the CPC joystick-0 lines; they are NOT keyboard keys
//   and come through hid.sv's joystick input instead. Only row 9 bit 7 (DEL)
//   is a key.
//
//   Modifier codes follow the BL616 companion remap (HID E0..E7 -> 0x68..0x6F):
//     0x68 LCtrl  0x69 LShift  0x6A LAlt  0x6B LGUI
//     0x6C RCtrl  0x6D RShift  0x6E RAlt  0x6F RGUI
//
//   Purely combinational, no latches (every output bit has one assign).
//
//   Line format below is parsed by tools/check_hid_cpc_map.py:
//     assign key[<row>*8+<bit>] = kb[7'h..] | kb[7'h..];
//============================================================================
`default_nettype none

module hid_cpc_matrix (
    input  wire [127:0] kb,     // [usb_hid_code] = 1 while pressed
    output wire [79:0]  key     // [8*row + bit] = 1 while CPC key pressed
);

    // ---- Row 0 : cursors, small ENTER, FDOT, F9 F6 F3 -----------------------
    assign key[0*8+0] = kb[7'h52];                                   // Up
    assign key[0*8+1] = kb[7'h4F];                                   // Right
    assign key[0*8+2] = kb[7'h51];                                   // Down
    assign key[0*8+3] = kb[7'h42] | kb[7'h61];                       // F9  (F9, KP9)
    assign key[0*8+4] = kb[7'h3F] | kb[7'h5E];                       // F6  (F6, KP6)
    assign key[0*8+5] = kb[7'h3C] | kb[7'h5B];                       // F3  (F3, KP3)
    assign key[0*8+6] = kb[7'h4D];                                   // ENTER small (End)
    assign key[0*8+7] = kb[7'h4E];                                   // FDOT (PgDn)

    // ---- Row 1 : Left, COPY, F7 F8 F5 F1 F2 F0 ------------------------------
    assign key[1*8+0] = kb[7'h50];                                   // Left
    assign key[1*8+1] = kb[7'h49] | kb[7'h6A];                       // COPY (Insert, left Alt / Mac Option)
    assign key[1*8+2] = kb[7'h40] | kb[7'h5F];                       // F7  (F7, KP7)
    assign key[1*8+3] = kb[7'h41] | kb[7'h60];                       // F8  (F8, KP8)
    assign key[1*8+4] = kb[7'h3E] | kb[7'h5D];                       // F5  (F5, KP5)
    assign key[1*8+5] = kb[7'h3A] | kb[7'h59];                       // F1  (F1, KP1)
    assign key[1*8+6] = kb[7'h3B] | kb[7'h5A];                       // F2  (F2, KP2)
    assign key[1*8+7] = kb[7'h43] | kb[7'h62];                       // F0  (F10, KP0)

    // ---- Row 2 : CLR [ RETURN ] F4 SHIFT \ CTRL -----------------------------
    assign key[2*8+0] = kb[7'h4C];                                   // CLR (Delete)
    assign key[2*8+1] = kb[7'h30];                                   // [   (HID ']')
    assign key[2*8+2] = kb[7'h28] | kb[7'h58];                       // RETURN (Enter, KP Enter)
    assign key[2*8+3] = kb[7'h31] | kb[7'h32];                       // ]   (HID '\' US, '#' on ISO DE/UK)
    assign key[2*8+4] = kb[7'h3D] | kb[7'h5C];                       // F4  (F4, KP4)
    assign key[2*8+5] = kb[7'h69] | kb[7'h6D] | kb[7'h57] | kb[7'h55]; // SHIFT (L, R, KP+, KP*)
    assign key[2*8+6] = kb[7'h64];                                   // \   (non-US '<>')
    assign key[2*8+7] = kb[7'h68] | kb[7'h6C];                       // CTRL (L, R)

    // ---- Row 3 : ^ - @ P ; : / . --------------------------------------------
    assign key[3*8+0] = kb[7'h2E];                                   // ^   (HID '=')
    assign key[3*8+1] = kb[7'h2D] | kb[7'h56];                       // -   (HID '-', KP-)
    assign key[3*8+2] = kb[7'h2F];                                   // @   (HID '[')
    assign key[3*8+3] = kb[7'h13];                                   // P
    assign key[3*8+4] = kb[7'h34] | kb[7'h57];                       // ;   (HID ''', KP+ = Shift+;)
    assign key[3*8+5] = kb[7'h33] | kb[7'h55];                       // :   (HID ';', KP* = Shift+:)
    assign key[3*8+6] = kb[7'h38] | kb[7'h54];                       // /   (HID '/', KP/)
    assign key[3*8+7] = kb[7'h37] | kb[7'h63];                       // .   (HID '.', KP.)

    // ---- Row 4 : 0 9 O I L K M , -------------------------------------------
    assign key[4*8+0] = kb[7'h27];                                   // 0
    assign key[4*8+1] = kb[7'h26];                                   // 9
    assign key[4*8+2] = kb[7'h12];                                   // O
    assign key[4*8+3] = kb[7'h0C];                                   // I
    assign key[4*8+4] = kb[7'h0F];                                   // L
    assign key[4*8+5] = kb[7'h0E];                                   // K
    assign key[4*8+6] = kb[7'h10];                                   // M
    assign key[4*8+7] = kb[7'h36];                                   // ,

    // ---- Row 5 : 8 7 U Y H J N SPACE ----------------------------------------
    assign key[5*8+0] = kb[7'h25];                                   // 8
    assign key[5*8+1] = kb[7'h24];                                   // 7
    assign key[5*8+2] = kb[7'h18];                                   // U
    assign key[5*8+3] = kb[7'h1C];                                   // Y
    assign key[5*8+4] = kb[7'h0B];                                   // H
    assign key[5*8+5] = kb[7'h0D];                                   // J
    assign key[5*8+6] = kb[7'h11];                                   // N
    assign key[5*8+7] = kb[7'h2C];                                   // SPACE

    // ---- Row 6 : 6 5 R T G F B V --------------------------------------------
    assign key[6*8+0] = kb[7'h23];                                   // 6
    assign key[6*8+1] = kb[7'h22];                                   // 5
    assign key[6*8+2] = kb[7'h15];                                   // R
    assign key[6*8+3] = kb[7'h17];                                   // T
    assign key[6*8+4] = kb[7'h0A];                                   // G
    assign key[6*8+5] = kb[7'h09];                                   // F
    assign key[6*8+6] = kb[7'h05];                                   // B
    assign key[6*8+7] = kb[7'h19];                                   // V

    // ---- Row 7 : 4 3 E W S D C X --------------------------------------------
    assign key[7*8+0] = kb[7'h21];                                   // 4
    assign key[7*8+1] = kb[7'h20];                                   // 3
    assign key[7*8+2] = kb[7'h08];                                   // E
    assign key[7*8+3] = kb[7'h1A];                                   // W
    assign key[7*8+4] = kb[7'h16];                                   // S
    assign key[7*8+5] = kb[7'h07];                                   // D
    assign key[7*8+6] = kb[7'h06];                                   // C
    assign key[7*8+7] = kb[7'h1B];                                   // X

    // ---- Row 8 : 1 2 ESC Q TAB A CAPS Z -------------------------------------
    assign key[8*8+0] = kb[7'h1E];                                   // 1
    assign key[8*8+1] = kb[7'h1F];                                   // 2
    assign key[8*8+2] = kb[7'h29];                                   // ESC
    assign key[8*8+3] = kb[7'h14];                                   // Q
    assign key[8*8+4] = kb[7'h2B];                                   // TAB
    assign key[8*8+5] = kb[7'h04];                                   // A
    assign key[8*8+6] = kb[7'h39];                                   // CAPS LOCK
    assign key[8*8+7] = kb[7'h1D];                                   // Z

    // ---- Row 9 : joystick 0 lines (not keys) + DEL --------------------------
    assign key[9*8+6:9*8+0] = 7'd0;                                  // joystick lines: via hid.sv
    assign key[9*8+7]       = kb[7'h2A];                             // DEL (Backspace)

endmodule

`default_nettype wire

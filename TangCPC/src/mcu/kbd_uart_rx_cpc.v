//============================================================================
// kbd_uart_rx_cpc.v
//   UART receiver for the RP2040 (Pico) USB companion, CPC flavour.
//
//   Derived from MSXnano/fpga/src/kbd_uart_rx.v. The wire protocol is the
//   UNCHANGED, frozen MSXnano one, so the stock firmware keeps working; only
//   the cell coordinates it sends differ (see firmware/keymaps_cpc.h):
//     0x90 <cell>  MAKE   cell = 0x80 | (bit<<4) | row   (row 0..9 on a CPC)
//     0xA0 <cell>  BREAK
//     0xFE m0..m10 0xFF   resync of the 11-row active-low shadow (every 250 ms)
//     0xC0 <ver>          version announce, consumed and dropped
//     0xB0 <port> <byte>  joystick, active-high: b0 R, b1 L, b2 D, b3 U, b4 A, b5 B
//     0xD0 <dx> <dy> <b>  mouse, consumed and dropped (CPC mouse not wired)
//     0x01..0x04          one-byte commands, ignored
//   Row 10 of the resync is read and discarded (the CPC has rows 0..9).
//
//   Output ext_key[8*row+bit] is ACTIVE-HIGH (1 = pressed), the format
//   hid.sv expects. A 1 s watchdog releases everything if the link goes quiet.
//
//   One clock domain (clk); the RX pin goes through a 2-flop synchroniser.
//   Defaults: clk32 -> CLKS_PER_BIT = 277 (0.3 % baud error, fine for 8N1).
//============================================================================
`default_nettype none

module kbd_uart_rx_cpc #(
    parameter CLK_FREQ = 32_000_000,
    parameter BAUD     = 115_200
)(
    input  wire        clk,
    input  wire        rst_n,        // active-low
    input  wire        rx,           // raw async UART RX pin

    output wire [79:0] ext_key,      // [8*row+bit], 1 = pressed
    output reg  [7:0]  joy_state0,   // active-high as received
    output reg  [7:0]  joy_state1,
    output reg         link_alive    // 1 while bytes arrive (watchdog not expired)
);

    localparam integer CLKS_PER_BIT = CLK_FREQ / BAUD;
    localparam integer WD_LIMIT     = CLK_FREQ;          // ~1 s

    //------------------------------------------------------------------------
    // 2-flop synchroniser + 8N1 receiver
    //------------------------------------------------------------------------
    (* syn_preserve = 1 *) reg rx_s1;
    (* syn_preserve = 1 *) reg rx_s2;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin rx_s1 <= 1'b1; rx_s2 <= 1'b1; end
        else        begin rx_s1 <= rx;   rx_s2 <= rx_s1; end
    end

    localparam [1:0] RX_IDLE = 2'd0, RX_START = 2'd1, RX_DATA = 2'd2, RX_STOP = 2'd3;

    reg [1:0]  rx_state;
    reg [15:0] rx_cnt;
    reg [2:0]  rx_bit;
    reg [7:0]  rx_shift;
    reg [7:0]  rx_byte;
    reg        rx_valid;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_state <= RX_IDLE; rx_cnt <= 16'd0; rx_bit <= 3'd0;
            rx_shift <= 8'd0;    rx_byte <= 8'd0; rx_valid <= 1'b0;
        end else begin
            rx_valid <= 1'b0;
            case (rx_state)
                RX_IDLE: begin
                    rx_cnt <= 16'd0; rx_bit <= 3'd0;
                    if (!rx_s2) rx_state <= RX_START;
                end
                RX_START: begin                           // re-sample mid start bit
                    if (rx_cnt < (CLKS_PER_BIT/2) - 1) rx_cnt <= rx_cnt + 16'd1;
                    else begin
                        rx_cnt   <= 16'd0;
                        rx_state <= rx_s2 ? RX_IDLE : RX_DATA;   // glitch -> back
                    end
                end
                RX_DATA: begin                            // LSB first
                    if (rx_cnt < CLKS_PER_BIT - 1) rx_cnt <= rx_cnt + 16'd1;
                    else begin
                        rx_cnt           <= 16'd0;
                        rx_shift[rx_bit] <= rx_s2;
                        if (rx_bit == 3'd7) begin rx_bit <= 3'd0; rx_state <= RX_STOP; end
                        else                rx_bit <= rx_bit + 3'd1;
                    end
                end
                RX_STOP: begin
                    if (rx_cnt < CLKS_PER_BIT - 1) rx_cnt <= rx_cnt + 16'd1;
                    else begin
                        rx_cnt   <= 16'd0;
                        rx_byte  <= rx_shift;
                        rx_valid <= 1'b1;
                        rx_state <= RX_IDLE;
                    end
                end
                default: rx_state <= RX_IDLE;
            endcase
        end
    end

    //------------------------------------------------------------------------
    // Watchdog: no byte for ~1 s -> release all keys and joysticks
    //------------------------------------------------------------------------
    reg [25:0] wd_cnt;
    wire wd_timeout = (wd_cnt == WD_LIMIT - 1) && !rx_valid;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin wd_cnt <= 26'd0; link_alive <= 1'b0; end
        else if (rx_valid) begin wd_cnt <= 26'd0; link_alive <= 1'b1; end
        else if (wd_cnt < WD_LIMIT) begin
            wd_cnt <= wd_cnt + 26'd1;
            if (wd_cnt == WD_LIMIT - 1) link_alive <= 1'b0;
        end
    end

    //------------------------------------------------------------------------
    // Protocol decoder. Matrix is stored ACTIVE-LOW like the firmware's shadow
    // (1 = released) and inverted on output. 11 rows are kept so the resync
    // length matches the firmware; row 10 is never exported.
    //------------------------------------------------------------------------
    localparam [3:0] D_IDLE = 4'd0, D_CELL = 4'd1, D_LOAD = 4'd2, D_JOY_PORT = 4'd3,
                     D_JOY_BYTE = 4'd4, D_VERSION = 4'd5, D_MOU1 = 4'd6,
                     D_MOU2 = 4'd7, D_MOU3 = 4'd8;

    reg [7:0] mtx [0:10];
    reg [3:0] dstate;
    reg       pending_make;
    reg [3:0] load_idx;
    reg       joy_port;
    integer   i;

    wire [3:0] cell_row = rx_byte[3:0];
    wire [2:0] cell_bit = rx_byte[6:4];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dstate <= D_IDLE; pending_make <= 1'b0; load_idx <= 4'd0; joy_port <= 1'b0;
            joy_state0 <= 8'h00; joy_state1 <= 8'h00;
            for (i = 0; i < 11; i = i + 1) mtx[i] <= 8'hFF;
        end else if (rx_valid) begin
            case (dstate)
                D_IDLE: case (rx_byte)
                    8'h90: begin pending_make <= 1'b1; dstate <= D_CELL;     end
                    8'hA0: begin pending_make <= 1'b0; dstate <= D_CELL;     end
                    8'hB0: dstate <= D_JOY_PORT;
                    8'hC0: dstate <= D_VERSION;
                    8'hD0: dstate <= D_MOU1;
                    8'hFE: begin load_idx <= 4'd0;     dstate <= D_LOAD;     end
                    default: ;                       // 0x01..0x04, stray 0xFF
                endcase
                D_CELL: begin
                    if (cell_row <= 4'd10) mtx[cell_row][cell_bit] <= ~pending_make;
                    dstate <= D_IDLE;
                end
                D_LOAD: begin                        // every byte is data, even 0xFF
                    mtx[load_idx] <= rx_byte;
                    if (load_idx == 4'd10) dstate <= D_IDLE;
                    else                   load_idx <= load_idx + 4'd1;
                end
                D_JOY_PORT: begin joy_port <= rx_byte[0]; dstate <= D_JOY_BYTE; end
                D_JOY_BYTE: begin
                    if (joy_port) joy_state1 <= rx_byte; else joy_state0 <= rx_byte;
                    dstate <= D_IDLE;
                end
                D_VERSION: dstate <= D_IDLE;         // consume, or it decodes as a command
                D_MOU1:    dstate <= D_MOU2;         // dx
                D_MOU2:    dstate <= D_MOU3;         // dy
                D_MOU3:    dstate <= D_IDLE;         // buttons
                default:   dstate <= D_IDLE;
            endcase
        end else if (wd_timeout) begin
            for (i = 0; i < 11; i = i + 1) mtx[i] <= 8'hFF;
            joy_state0 <= 8'h00; joy_state1 <= 8'h00;
            dstate <= D_IDLE; pending_make <= 1'b0; load_idx <= 4'd0; joy_port <= 1'b0;
        end
    end

    genvar r;
    generate
        for (r = 0; r < 10; r = r + 1) begin : g_row
            assign ext_key[8*r +: 8] = ~mtx[r];
        end
    endgenerate

endmodule

`default_nettype wire

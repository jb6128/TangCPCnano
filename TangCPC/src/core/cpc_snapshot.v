// cpc_snapshot.v -- DEBUG: RAM snapshot over a UART (8N1), started with S1 + S2.
//
// Only active while the OSD option "Debug / Snapshot" is S1+S2 (enable = 1). S1 and S2 (both pressed for ~32 ms)
// freeze the Z80 (cpu_hold via busy). The 64 KB base RAM is read through the SDRAM boot port twice: pass 1 only adds
// up the 16-bit sum, then the 256-byte header is sent (the sum is in it), then pass 2 reads the RAM again and sends
// it. The RAM cannot change in between (Z80 frozen, no DMA on the CPC, the boot port is owned by this module).
//
//   256 bytes header (16-bit values little endian, unused bytes 0):
//       0..3    'S' 'N' 'A' 'P'
//       4       version 0x02
//       5..6    PC (address of the last opcode fetch)
//       7       last ROM select (&DF00)
//       8       last RAM configuration (&7Fxx, 11xxxxxx)
//       9       last Gate Array mode / ROM enable (&7Fxx, 10xxxxxx)
//       10..11  16-bit sum of the 65536 RAM bytes
//       12      number of 64 KB RAM pages that follow (1)
//       13      CRTC type (0 / 1)
//       14..15  0
//       16..33  CRTC registers R0..R17 as last written by the Z80 (R16 / R17 are read-only light pen registers: 0)
//       34..50  Gate Array palette as last written, hardware colour numbers 0..31: pens 0..15, then the border
//       51..255 0
//   65536 bytes RAM at BASE .. BASE + 0xFFFF (the 64 KB the video reads), raw, starts at offset 0x100
//
// then releases the Z80. 65792 bytes (a multiple of 256): 0.71 s at 921600 baud, 1.4 s at 460800, 5.7 s at 115200.
// The capture can be done with  cat /dev/ttyUSBx > snap.bin  (stty -F /dev/ttyUSBx 921600 raw first), the data starts
// at the "SNAP" magic. The boot port is shared with cpc_romload: the snapshot only starts when no ROM load is running
// (ok) and stops when one starts.
module cpc_snapshot #(
    parameter [22:0] BASE = 23'h020000        // SDRAM byte address of the base RAM page (CPC RAM page 2)
) (
    input  wire         clk,
    input  wire         rst,
    input  wire         enable,       // OSD Debug / Snapshot = S1+S2
    input  wire   [8:0] bit_div,      // clk32 cycles per UART bit: 278 (115200), 69 (460800), 35 (921600)
    input  wire         btn_a,        // S1, active high, asynchronous
    input  wire         btn_b,        // S2, active high, asynchronous
    input  wire         ok,           // boot done, SDRAM ready, no ROM load running
    input  wire  [15:0] pc,
    input  wire   [7:0] rsel,
    input  wire   [7:0] mmr,
    input  wire   [7:0] gam,
    input  wire         crtc_type,
    input  wire [127:0] crtc_regs,    // R0..R15, 8 bits each (R0 in bits 7:0); stable while the Z80 is frozen
    input  wire  [84:0] pal,          // 17 x 5 bit: pens 0..15, border (pen 0 in bits 4:0)
    output reg          busy,         // 1 while a snapshot runs: freeze the Z80, boot port owned by this module
    output wire  [22:0] addr,
    output reg          req,          // 1 clk pulse: read request
    input  wire         ack,          // 1 clk pulse: data valid in din
    input  wire   [7:0] din,
    output wire         tx
);
    // ---- trigger: S1 and S2 together for 2^20 clk (about 33 ms) ------------------------------
    reg [1:0]  sa, sb;
    reg [19:0] dbc;
    reg        trig;
    always @(posedge clk) begin
        sa <= {sa[0], btn_a};
        sb <= {sb[0], btn_b};
        trig <= 1'b0;
        if (rst || !(sa[1] & sb[1])) dbc <= 20'd0;
        else if (dbc != 20'hFFFFF) begin
            dbc <= dbc + 20'd1;
            if (dbc == 20'hFFFFE) trig <= 1'b1;
        end
    end

    // ---- transmitter (8N1, LSB first) ---------------------------------------------------------
    reg  [8:0] div;
    reg  [3:0] bitn;
    reg  [8:0] shift;
    reg        tx_busy;
    reg        txr;
    reg        tx_go;
    reg  [7:0] tx_d;
    assign tx = txr;

    always @(posedge clk) begin
        if (rst) begin
            tx_busy <= 1'b0; txr <= 1'b1; div <= 9'd0; bitn <= 4'd0; shift <= 9'h1FF;
        end else if (!tx_busy) begin
            if (tx_go) begin
                txr   <= 1'b0;                 // start bit
                shift <= {1'b1, tx_d};
                tx_busy <= 1'b1; bitn <= 4'd0; div <= 9'd0;
            end
        end else begin
            if (div == bit_div - 9'd1) begin
                div <= 9'd0;
                if (bitn == 4'd9) begin        // the stop bit has been sent for a full bit time
                    tx_busy <= 1'b0;
                end else begin
                    txr   <= shift[0];         // bits 1..8: data LSB first, then the stop bit
                    shift <= {1'b1, shift[8:1]};
                    bitn  <= bitn + 4'd1;
                end
            end else div <= div + 9'd1;
        end
    end

    // ---- snapshot sequence --------------------------------------------------------------------
    localparam [2:0] S_IDLE = 3'd0, S_HOLD = 3'd1, S_HDR = 3'd2, S_RD = 3'd3,
                     S_ACK  = 3'd4, S_TX   = 3'd5, S_END = 3'd6;
    reg  [2:0]  state;
    reg         pass2;                 // 0: pass 1 (sum only), 1: pass 2 (send the RAM)
    reg [16:0]  tmr;
    reg [15:0]  idx;
    reg  [7:0]  hi;                    // header byte index
    reg [15:0]  sum;
    reg  [7:0]  byte_r;
    reg [15:0]  s_pc;
    reg  [7:0]  s_rsel, s_mmr, s_gam;
    assign addr = BASE + {7'd0, idx};

    wire can_send = !tx_busy && !tx_go;

    // header byte selection (the CRTC registers and the palette do not change while the Z80 is frozen)
    wire [7:0] pi    = hi - 8'd34;                         // pen number for hi = 34..50
    wire [8:0] pofs  = {4'd0, pi[4:0]} * 9'd5;
    wire [4:0] pal_b = pal[pofs +: 5];
    reg  [7:0] hb;
    always @* begin
        if (hi >= 8'd16 && hi < 8'd32)      hb = crtc_regs[{hi[3:0], 3'b000} +: 8];   // R0..R15
        else if (hi >= 8'd34 && hi < 8'd51) hb = {3'b000, pal_b};
        else case (hi)
            8'd0:    hb = "S";
            8'd1:    hb = "N";
            8'd2:    hb = "A";
            8'd3:    hb = "P";
            8'd4:    hb = 8'h02;
            8'd5:    hb = s_pc[7:0];
            8'd6:    hb = s_pc[15:8];
            8'd7:    hb = s_rsel;
            8'd8:    hb = s_mmr;
            8'd9:    hb = s_gam;
            8'd10:   hb = sum[7:0];
            8'd11:   hb = sum[15:8];
            8'd12:   hb = 8'd1;
            8'd13:   hb = {7'd0, crtc_type};
            default: hb = 8'h00;                           // 14, 15, R16, R17 (32, 33), 51..255
        endcase
    end

    always @(posedge clk) begin
        tx_go <= 1'b0;
        req   <= 1'b0;
        if (rst) begin
            state <= S_IDLE; busy <= 1'b0; tmr <= 17'd0; idx <= 16'd0; hi <= 8'd0; sum <= 16'd0; pass2 <= 1'b0;
            byte_r <= 8'd0; s_pc <= 16'd0; s_rsel <= 8'd0; s_mmr <= 8'd0; s_gam <= 8'd0; tx_d <= 8'd0;
        end else begin
            case (state)
                S_IDLE: begin
                    busy <= 1'b0;
                    if (trig && enable && ok) begin
                        busy <= 1'b1; tmr <= 17'd0; state <= S_HOLD;
                    end
                end
                S_HOLD: begin                              // Z80 frozen (busy): let a running access finish (1 ms)
                    if (!ok || !enable) state <= S_END;
                    else begin
                        tmr <= tmr + 17'd1;
                        if (tmr[15]) begin
                            s_pc <= pc; s_rsel <= rsel; s_mmr <= mmr; s_gam <= gam;
                            hi <= 8'd0; idx <= 16'd0; sum <= 16'd0; pass2 <= 1'b0;
                            state <= S_RD;                 // pass 1: read the RAM, add up the sum
                        end
                    end
                end
                S_HDR: begin
                    if (!ok || !enable) state <= S_END;
                    else if (can_send) begin
                        tx_d <= hb; tx_go <= 1'b1;
                        hi <= hi + 8'd1;
                        if (hi == 8'd255) begin            // last header byte: pass 2, send the RAM
                            idx <= 16'd0; pass2 <= 1'b1; state <= S_RD;
                        end
                    end
                end
                S_RD: begin
                    if (!ok || !enable) state <= S_END;
                    else begin
                        req <= 1'b1; tmr <= 17'd0; state <= S_ACK;
                    end
                end
                S_ACK: begin
                    tmr <= tmr + 17'd1;
                    if (ack) begin
                        if (!pass2) begin                  // pass 1: sum only
                            sum <= sum + {8'd0, din};
                            if (idx == 16'hFFFF) begin hi <= 8'd0; state <= S_HDR; end
                            else begin idx <= idx + 16'd1; state <= S_RD; end
                        end else begin                     // pass 2: send
                            byte_r <= din;
                            state <= S_TX;
                        end
                    end else if (tmr[15] || !ok || !enable) state <= S_END;   // no answer from the SDRAM: give up
                end
                S_TX: begin
                    if (!ok || !enable) state <= S_END;
                    else if (can_send) begin
                        tx_d <= byte_r; tx_go <= 1'b1;
                        if (idx == 16'hFFFF) state <= S_END;
                        else begin idx <= idx + 16'd1; state <= S_RD; end
                    end
                end
                default: begin                              // S_END: release the Z80 and the boot port
                    busy <= 1'b0;
                    state <= S_IDLE;
                end
            endcase
        end
    end
endmodule

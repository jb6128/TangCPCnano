/*
    top.sv - TangCPC : Amstrad CPC 6128 sur Tang Nano 20K
    Phase 0 : Scaffold minimal — PLLs + pins GOWIN

    Cible : GOWIN GW2AR-18C (Tang Nano 20K)
    Horloge entrée : 27 MHz (pin 4)
    Génère : clk32 (32.000 MHz système), flash_clk (96 MHz SPI)
*/

module top (
    input           clk,        // 27 MHz oscillateur Tang Nano 20K

    input           reset,      // bouton S2 (actif haut)
    input           user,       // bouton S1

    output [5:0]    leds_n,     // 6 LEDs intégrées (actif bas)
    output          ws2812,     // LED RGB WS2812

    // UART BL616 intégré
    input           uart_rx,
    output          bl616_mon_tx,

    // Flash SPI
    output          mspi_cs,
    output          mspi_clk,
    inout           mspi_di,
    inout           mspi_hold,
    inout           mspi_wp,
    inout           mspi_do,

    // SDRAM "Magic ports" GOWIN (sur puce GW2AR-18)
    output          O_sdram_clk,
    output          O_sdram_cke,
    output          O_sdram_cs_n,
    output          O_sdram_cas_n,
    output          O_sdram_ras_n,
    output          O_sdram_wen_n,
    inout  [31:0]   IO_sdram_dq,
    output [10:0]   O_sdram_addr,
    output [1:0]    O_sdram_ba,
    output [3:0]    O_sdram_dqm,

    // GPIO — joystick DB9 / souris
    input  [5:0]    io,        // DB9 joystick: BTN0, DOWN, UP, RIGHT, LEFT, BTN1 (pins 27,28,25,26,29,30)
    input           spare_rx,   // pin 31: UART RX from the RP2040 (Pico) keyboard companion

    // Analog-style 15 kHz video out as on the real CPC (2 bits per colour + composite sync).
    // Per colour: pin1 = level>=half, pin0 = level==full; both pins through equal resistors
    // into one node give 0 / 1/2 / 1 of full scale.  csync is active low.
    inout  [1:0]    rgb_r,      // high-Z when the OSD option 'RGB out' is Off (pins can then be used
    inout  [1:0]    rgb_g,      // for something else, e.g. a joystick 2 wired like in other projects)
    inout  [1:0]    rgb_b,
    inout           csync,

    // Interface MCU externe BL616 / M0S Dock
    // m0s[0] = spi_dout (out), m0s[1..3] = din/ss/clk (in)
    // m0s[4] (spi_intn, pin 51) : ajouté en Phase 6 (conflit bank GOWIN)
    output          m0s_0,      // spi_io_dout → MCU
    input           m0s_1,      // spi_io_din  ← MCU
    input           m0s_2,      // spi_io_ss   ← MCU
    input           m0s_3,      // spi_io_clk  ← MCU

    // SPI vers BL616 intégré
    input           spi_sclk,
    input           spi_csn,
    output          spi_dir,
    input           spi_dat,
    output          spi_irqn,

    // Carte SD
    output          sd_clk,
    inout           sd_cmd,
    inout  [3:0]    sd_dat,

    // HDMI TMDS
    output          tmds_clk_n,
    output          tmds_clk_p,
    output [2:0]    tmds_d_n,
    output [2:0]    tmds_d_p
);

// ---------------------------------------------------------------------------
// Pont UART BL616 → console externe
// ---------------------------------------------------------------------------
assign bl616_mon_tx = uart_rx;

// ---------------------------------------------------------------------------
// PLLs
//   flash_pll: 27 MHz * 32 / 9 → 96 MHz (flash SPI; also the input of pll_160m)
//   pll_160m : 96 MHz * 5 / 3 → 160.000 MHz (HDMI TMDS), exact
//              via CLKDIV /5 → 32.000 MHz (clk32 système)
//   (until 0ce5820: 27 MHz * 53 / 9 = 159 MHz → clk32 = 31.8 MHz, 0.625 % too slow)
// ---------------------------------------------------------------------------
wire clk160;
wire clk32;
wire flash_clk;
wire flash_clk_shifted;     // 96 MHz, phase FLASH_PSDA ("0000" = no shift in this build) → mspi_clk
wire pll_lock_hdmi;

// Sorties mémoire Amstrad_motherboard (Phase 3c : connectées à SDRAM)
wire [22:0] core_mem_addr;
wire        core_mem_rd;
wire        core_mem_wr;
wire        core_romen;
wire [14:0] core_vram_addr;

// Bus CPU Amstrad_motherboard (Phase 3c)
wire [15:0] core_cpu_addr;
wire  [7:0] core_cpu_dout;

// Signaux de cpc_core (déclarés ici : utilisés par le mux du port Boot)
wire        boot_busy_w;
wire [3:0]  core_dbg_boot_state;
wire [22:0] core_boot_addr;
wire  [7:0] core_boot_din;
wire  [7:0] core_boot_dout;
wire        core_boot_req;
wire        core_boot_we;
wire        core_boot_ack;
wire [255:0] core_rom_map;

// Sorties vidéo Amstrad_motherboard (Phase 3d : connectées à HDMI)
wire [1:0]  core_red;
wire [1:0]  core_green;
wire [1:0]  core_blue;
wire        core_hsync;
wire        core_vsync;
wire        core_hblank;
wire        core_vblank;
wire        core_field;
wire        core_crtc_vs_raw;
wire        core_dbg_r0_ok, core_dbg_r4_ok, core_dbg_r7_ok, core_dbg_r9_ok;
wire        core_ga_ready;
wire [15:0] core_dbg_pc;
wire [1:0]  core_mode;
wire        core_readback_ok;
wire        core_readback_done;
wire        core_wr_rd_ok;
wire        core_wr_rd_done;
wire [7:0]  core_rb_byte;
wire [15:0] core_dbg_first_flash;
wire        core_flash_retry, core_flash_err;   // boot loader: flash word needed a re-read / never verified
wire [15:0] core_flash_cnt;                      // boot loader: number of flash words whose two reads differed (no LED page since 2026-11-08, see the comment at the LED mux)

wire pll_lock_flash;
wire pll_lock = pll_lock_hdmi && pll_lock_flash;
// Power-On Reset: active while the PLLs are unlocked and for 2^16 clk32 cycles (2 ms)
// after they lock. The lock signals are asynchronous to clk32; using them directly
// released the reset at a different clk32 edge for different registers, and the SDRAM
// needs a stable clock for >= 100-200 us before its init sequence. Release is therefore
// synchronised and delayed. (clk32 itself only runs while pll_lock_hdmi is high.)
reg [1:0]  por_lock_s = 2'b00;
reg [15:0] por_cnt    = 16'd0;
reg        por_r      = 1'b1;
always @(posedge clk32) begin
    por_lock_s <= {por_lock_s[0], pll_lock};
    if (!por_lock_s[1]) begin
        por_cnt <= 16'd0;
        por_r   <= 1'b1;
    end else if (por_cnt != 16'hFFFF) begin
        por_cnt <= por_cnt + 16'd1;
    end else begin
        por_r   <= 1'b0;
    end
end
wire por = por_r;

pll_160m pll_hdmi_inst (
    .clkout ( clk160        ),
    .lock   ( pll_lock_hdmi ),
    .clkin  ( flash_clk     ),     // 96 MHz from flash_pll
    .reset  ( ~pll_lock_flash )    // wait until the 96 MHz input is stable
);

Gowin_CLKDIV clkdiv_inst (
    .clkout ( clk32  ),
    .hclkin ( clk160 ),
    .resetn ( pll_lock_hdmi )
);

// Flash read experiment (S1+S2 LED page shows the number of flash words whose two reads differed):
//   FLASH_PSDA        phase of the SPI pin clock (CLKOUTP of flash_pll), delay in steps of 22.5 deg of 96 MHz:
//                     "1111" = 22.5 deg lead (this core up to 12df7b9), "1110" = 45, "1101" = 67.5,
//                     "1100" = 90 deg lead (NanoMig on the Tang Nano 20K, flash at 85.5 MHz)
//   FLASH_READ_DELAY  sampling point of the read data: 0 = states 24..31 (upstream default, NanoMig),
//                     1 = states 25..32 (one SPI clock later, this core up to 12df7b9)
// Hardware scan 2026-11-06 (READ_DELAY 1): "1101", "1110" fail (many differing reads), "1111", "0000", "0001" boot OK,
// "0010", "0011" fail with counter 0 (stable wrong data). Default = centre of the working steps. 12df7b9 = "1111" / 1 (window edge).
localparam       FLASH_PSDA       = "0000";   // string as PSDA_SEL of the rPLL; measured on hardware 2026-11-06: working "1111", "0000", "0001" (READ_DELAY 1), centre "0000"
localparam       FLASH_READ_DELAY = 1;

flash_pll #(.PSDA(FLASH_PSDA))
pll_flash_inst (
    .clkout  ( flash_clk         ),  // 96 MHz — logique flash
    .clkoutp ( flash_clk_shifted ),  // 96 MHz, phase FLASH_PSDA — horloge SPI physique
    .lock    ( pll_lock_flash    ),
    .clkin   ( clk               )
);

// ---------------------------------------------------------------------------
// Sélection MCU SPI : BL616 intégré ou M0S Dock externe
// Si m0s[2] (SS externe) descend à 0 → basculement vers M0S Dock
// ---------------------------------------------------------------------------
wire spi_io_dout;
wire spi_intn;

assign spi_dir  = spi_io_dout;
assign m0s_0    = spi_io_dout;
// m0s_4 (spi_intn → pin 51) retiré en Phase 0 (conflit bank GOWIN, à résoudre Phase 6)
assign spi_irqn = spi_intn;

// The SPI source starts on the internal BL616 pins and switches (for good, until the next power-on
// reset) to the external pins (RP2040 / M0S Dock) when the external chip select first goes low.
// Until then, and at the moment of the switch, whatever the internal pins do (floating or noisy while
// the BL616 boots) can be taken for an SPI transfer; a stray joystick transfer would leave
// "joystick right" set for good. At the switch (`spi_sw`, one clk32 pulse) the HID state is therefore
// cleared and mcu_spi waits for the chip select to be idle again before it accepts data.
reg spi_ext;
reg spi_sw = 1'b0;
reg [1:0] m0s_2_s = 2'b11;
always @(posedge clk32) begin
    m0s_2_s <= { m0s_2_s[0], m0s_2 };
    spi_sw  <= 1'b0;
    if (por)
        spi_ext <= 1'b0;
    else if (!spi_ext && m0s_2_s == 2'b00) begin
        spi_ext <= 1'b1;
        spi_sw  <= 1'b1;
    end
end

wire spi_io_din = spi_ext ? m0s_1 : spi_dat;
wire spi_io_ss  = spi_ext ? m0s_2 : spi_csn;
wire spi_io_clk = spi_ext ? m0s_3 : spi_sclk;

// ---------------------------------------------------------------------------
// Sorties de statut
// ---------------------------------------------------------------------------
wire sdram_ready;
wire [4:0] sdram_init_cnt;  // debug : compteur init SDRAM

// Deux compteurs indépendants pour diagnostiquer quelle horloge tourne :
//   cnt27 : sur clk (27 MHz brut, sans PLL) → toujours disponible
//   cnt32 : sur clk32 (32 MHz via PLL+CLKDIV) → dépend du CLKDIV
reg [24:0] cnt27;
reg [24:0] cnt32;
always @(posedge clk)   cnt27 <= cnt27 + 1;
always @(posedge clk32) cnt32 <= cnt32 + 1;

// Compteur d'accès mémoire Z80 : s'incrémente à chaque lecture/écriture CPU
// Si Z80 tourne (ROM lue, instructions exécutées) → clignote rapidement
// bit[23] ≈ freq_acces/2^23 → ~1 Hz si Z80 à 4MHz avec accès toutes les 4 cycles
reg [24:0] cpu_rd_cnt;
always @(posedge clk32)
    cpu_rd_cnt <= cpu_rd_cnt + (core_mem_rd ? 25'd1 : 25'd0);

// LED debug : CRTC registers + vsync counter
//   [5] CRTC R0 == 63  (horizontal total)
//   [4] CRTC R4 == 38  (vertical total)
//   [3] CRTC R7 == 30  (vsync position)
//   [2] CRTC R9 == 7   (max raster)
//   [1] vsync counter bit 4  (devrait clignoter ~1.6 Hz si vsync = 50 Hz)
//   [0] readback_ok    (confirmation ROM OK)

reg [24:0] vsync_cnt;
wire vsync_pulse;
reg  vsync_r;
always @(posedge clk32) begin

    vsync_r <= core_crtc_vs_raw;
    if (core_crtc_vs_raw && !vsync_r)
        vsync_cnt <= vsync_cnt + 1;
end

// CPC alive: retriggerable ~100 ms timeout, restarted by every CRTC VSync (50 Hz -> steady on;
// goes off when the CRTC/CPU stops). Used for LED5.
reg [21:0] alive_to = 22'd0;
wire       cpc_alive = (alive_to != 22'd0);
always @(posedge clk32) begin
    if (core_crtc_vs_raw && !vsync_r) alive_to <= 22'd3200000;
    else if (alive_to != 22'd0)       alive_to <= alive_to - 22'd1;
end

// h_offset : décalage horizontal dans le line buffer scandoubler
// Valeur calibrée visuellement via les boutons S1/S2 (voir ci-dessous)
wire  [3:0] sys_hpos;   // OSD "H pos"
wire        sys_w800;   // OSD "Width": 1 = 800 px HDMI active width (80 px wider than 720 = the visible part of a CPC line; start 40 px earlier)
// horizontal picture position: OSD "H pos", v = 0..15 shown as -8..+7 (plain unsigned value, the label is
// only text). offset = 160 + 32*(v-8) = -96 + 32*v (mod 1024); default v=8 = 0 = offset 160, the centred picture.
wire [9:0] h_offset_reg = 10'd928 + {1'b0, sys_hpos, 5'b00000} - (sys_w800 ? 10'd40 : 10'd0);

// --- Mode calibration horizontale (décommenter pour ajuster H_OFFSET) ---
// Utilise S1 (+32 pixels) et S2 (-32 pixels) pour décaler l'image en temps réel.
// Les LEDs affichent h_offset_reg[9:4] en binaire. Une fois centré,
// convertir la valeur LED en décimal × 16 et hardcoder H_OFFSET ci-dessus.
// Exemple : LEDs = 010100 → 20 × 16 = 320.
//
// reg [19:0] s1_deb_cnt, s2_deb_cnt;
// reg s1_stable, s2_stable, s1_prev, s2_prev;
// wire s1_pressed = s1_stable && !s1_prev;
// wire s2_pressed = s2_stable && !s2_prev;
//
// always @(posedge clk32) begin
//     if (por) begin
//         s1_deb_cnt <= 0; s2_deb_cnt <= 0;
//         s1_stable <= 0;  s2_stable <= 0;
//         s1_prev <= 0;    s2_prev <= 0;
//     end else begin
//         if (user != s1_stable) begin
//             s1_deb_cnt <= s1_deb_cnt + 1;
//             if (s1_deb_cnt == 20'hFFFFF) s1_stable <= user;
//         end else s1_deb_cnt <= 0;
//         s1_prev <= s1_stable;
//         if (reset != s2_stable) begin
//             s2_deb_cnt <= s2_deb_cnt + 1;
//             if (s2_deb_cnt == 20'hFFFFF) s2_stable <= reset;
//         end else s2_deb_cnt <= 0;
//         s2_prev <= s2_stable;
//     end
// end
//
// reg [9:0] h_offset_reg;  // remplace le wire ci-dessus
// always @(posedge clk32) begin
//     if (por) h_offset_reg <= 10'd512;
//     else begin
//         if (s1_pressed) h_offset_reg <= h_offset_reg + 10'd32;
//         if (s2_pressed) h_offset_reg <= h_offset_reg - 10'd32;
//     end
// end
// assign leds_n = ~h_offset_reg[9:4];  // remplace l'assign LEDs CRTC

wire [5:0] led_crtc = {
    ~core_dbg_r0_ok,            // [5]
    ~core_dbg_r4_ok,            // [4]
    ~core_dbg_r7_ok,            // [3]
    ~core_dbg_r9_ok,            // [2]
    ~vsync_cnt[4],              // [1] ~1.6 Hz si 50 Hz vsync
    ~core_readback_ok           // [0] ROM OK
};
// S1 + S2 together: address of the last opcode fetch, one snapshot every 4 s:
//   1 s all LEDs on (marker), then PC[15:10], PC[9:4], {PC[3:0],00}, 1 s each (lit = 1)
reg [26:0] pcshow_cnt;
reg [15:0] pcshow_snap;
always @(posedge clk32) begin
    pcshow_cnt <= pcshow_cnt + 27'd1;
    if (pcshow_cnt[26:25] == 2'd0) pcshow_snap <= core_dbg_pc;   // frozen for the other 3 steps
end
reg [5:0] pcshow_leds;
always @* begin
    case (pcshow_cnt[26:25])
        2'd0:    pcshow_leds = 6'b111111;
        2'd1:    pcshow_leds = pcshow_snap[15:10];
        2'd2:    pcshow_leds = pcshow_snap[9:4];
        default: pcshow_leds = {pcshow_snap[3:0], 2'b00};
    endcase
end
reg  disc_led = 1'b0, tape_led = 1'b0, rom_led = 1'b0;   // driven below, after the FDC/tape wires
reg  tape_rec_ind = 1'b0;           // LED4: SAVE recording in progress / failed
wire db9_any = (io != 6'b111111);   // any DB9 line low
assign leds_n = db9_any ? io :        // DB9 test: LED lit = pin pulled low. 0=BTN0 1=DOWN 2=UP 3=RIGHT 4=LEFT 5=BTN1
                        ~{cpc_alive, tape_rec_ind, rom_led, tape_led, disc_led, core_rom_map[7]};   // normal: LED0 ROM7 / DOS active (internal AMSDOS On or external ROM 7 in use; off = tape only), LED1 disc, LED2 tape, LED3 ROM loading, LED4 tape recording (solid) / recording failed (blinking), LED5 CPC alive (steady while CRTC VSync runs)
assign ws2812 = 1'b0;

// ---------------------------------------------------------------------------
// SDRAM — contrôleur multi-port CPC (Phase 3c : ports CPU et VRAM connectés)
// ---------------------------------------------------------------------------
// Port CPU : piloté par Amstrad_motherboard via cpc_core
wire [22:0] cpu_addr_w;     // adresse 23-bit (traduite par MMU)
wire  [7:0] cpu_din_w;      // données Z80 → SDRAM (pour écritures)
wire  [7:0] cpu_dout_w;     // données SDRAM → Z80 (pour lectures)
wire        cpu_req_w;      // requête (impulsion front montant)
wire        cpu_we_w;       // 1 = écriture
wire        cpu_ack_w;
wire        cpu_soon_w;

// Port VRAM : adresse CRTC → mot 16-bit depuis SDRAM
wire [21:0] vram_addr_w;    // adresse 22-bit (mot 16-bit dans SDRAM)
wire [15:0] vram_dout_w;    // données vidéo lues depuis SDRAM

// Conversion adresse VRAM :
//   vram_addr[14:0] = adresse mot 15-bit CRTC
//   La RAM de base CPC (64KB) est en SDRAM bytes 0x020000–0x02FFFF
//   → word address 0x010000 + vram_addr[14:0] = {7'b0000010, vram_addr[14:0]}
//
// Pendant le boot, geler vram_addr à 22'h3FFFFF (= valeur initiale de vram_addr_r
// dans cpc_sdram). Ainsi vram_addr == vram_addr_r → pas de lecture VRAM →
// SDRAM reste libre pour boot_req. Sans ça, le CRTC (non gelé par reset)
// incrémente vram_addr en permanence → SDRAM occupé → boot_req raté → blocage.
assign vram_addr_w = boot_busy_w ? 22'h3FFFFF : {7'b0000010, core_vram_addr};

// CPU SDRAM : connecté aux sorties mem_* de Amstrad_motherboard
assign cpu_addr_w  = core_mem_addr;      // adresse 23-bit MMU
assign cpu_din_w   = core_cpu_dout;      // données du Z80 (→ SDRAM pour écriture)
assign cpu_req_w   = core_mem_rd | core_mem_wr;
assign cpu_we_w    = core_mem_wr;

// VRAM data retournée au motherboard via cpc_core
wire [15:0] core_vram_din = vram_dout_w;

// Signaux port Boot → pilotés par cpc_core (Phase 2)
// Le port Boot est partagé : boot loader de cpc_core jusqu'à la fin de son
// auto-test (wr_rd_done), ensuite cpc_romload (ROMs externes, Étape 3).
wire        rl_sel = core_wr_rd_done;
wire [22:0] rl_addr;
wire  [7:0] rl_din;
wire        rl_req;
wire [22:0] boot_addr_w = rl_sel ? rl_addr : core_boot_addr;
wire  [7:0] boot_din_w  = rl_sel ? rl_din  : core_boot_din;
wire  [7:0] boot_dout_w;
wire        boot_req_w  = rl_sel ? rl_req  : core_boot_req;
wire        boot_we_w   = rl_sel ? 1'b1    : core_boot_we;
wire        boot_ack_w;
assign core_boot_dout = boot_dout_w;
assign core_boot_ack  = boot_ack_w;

cpc_sdram sdram_inst (
    // Interface physique SDRAM
    .O_sdram_clk   ( O_sdram_clk   ),
    .O_sdram_cke   ( O_sdram_cke   ),
    .O_sdram_cs_n  ( O_sdram_cs_n  ),
    .O_sdram_ras_n ( O_sdram_ras_n ),
    .O_sdram_cas_n ( O_sdram_cas_n ),
    .O_sdram_wen_n ( O_sdram_wen_n ),
    .O_sdram_dqm   ( O_sdram_dqm   ),
    .O_sdram_addr  ( O_sdram_addr  ),
    .O_sdram_ba    ( O_sdram_ba    ),
    .IO_sdram_dq   ( IO_sdram_dq   ),

    // Horloge et reset
    .clk           ( clk32         ),
    .reset_n       ( ~por          ),
    .ready         ( sdram_ready   ),
    .init_cnt_dbg  ( sdram_init_cnt ),

    // Port CPU
    .cpu_addr      ( cpu_addr_w    ),
    .cpu_din       ( cpu_din_w     ),
    .cpu_dout      ( cpu_dout_w    ),
    .cpu_req       ( cpu_req_w     ),
    .cpu_we        ( cpu_we_w      ),
    .cpu_ack       ( cpu_ack_w     ),
    .cpu_data_soon ( cpu_soon_w    ),

    // Port VRAM
    .vram_addr     ( vram_addr_w   ),
    .vram_dout     ( vram_dout_w   ),

    // Port Boot
    .boot_addr     ( boot_addr_w   ),
    .boot_din      ( boot_din_w    ),
    .boot_dout     ( boot_dout_w   ),
    .boot_req      ( boot_req_w    ),
    .boot_we       ( boot_we_w     ),
    .boot_ack      ( boot_ack_w    )
);

// ---------------------------------------------------------------------------
// Flash SPI — horloge physique = flash_clk_shifted (phase FLASH_PSDA, voir plus haut)
// Le module flash gère mspi_cs, mspi_di, mspi_hold, mspi_wp, mspi_do
// ---------------------------------------------------------------------------
assign mspi_clk = flash_clk_shifted;

// ---------------------------------------------------------------------------
// cpc_core — Boot loader ROM + Amstrad_motherboard (Phase 3b)
// ---------------------------------------------------------------------------


// Clavier / joysticks BL616 (pilotés plus bas par mcu_hid / hid_cpc_matrix)
wire        rom_busy;
wire  [1:0] sys_reset_w;
// While a ROM file is written to SDRAM the Z80 is only frozen (cpu_hold, via WAIT);
// CRTC and Gate Array keep running, so the HDMI picture and the OSD stay stable.
// When the load is done the CPC gets a 10 ms reset pulse, like the OSD Reset, so
// the firmware rescans the upper ROMs and the video sync (vreset) is re-aligned.
// A ROM load only freezes the Z80 (cpu_hold = rom_busy); there is no reset any more. A new ROM is
// found by the firmware at the next reset, which the user triggers from the OSD when wanted.
// OSD "Model" (6128/664/464) and "CRTC type": a change of the setting triggers a reset of ~32 ms so that the other ROM
// set is used immediately (the core samples the model while it is in reset).
wire  [1:0] sys_model;
wire        sys_crtc;
reg   [1:0] model_d = 2'd0;
reg         crtc_d = 1'b1;
reg  [20:0] model_rst_cnt = 21'd0;
always @(posedge clk32) begin
    model_d <= sys_model;
    crtc_d  <= sys_crtc;
    if (model_d != sys_model || crtc_d != sys_crtc) model_rst_cnt <= 21'h1FFFFF;
    else if (model_rst_cnt != 21'd0) model_rst_cnt <= model_rst_cnt - 21'd1;
end
wire        model_rst = (model_rst_cnt != 21'd0);
// ROM load during start-up: while a load runs before the CPC picture has been aligned once since the
// last reset (vreset_done = 0: cold boot / power-on with a ROM set from cpc.ini, or a load right after
// an OSD reset) the CPC is held in reset instead of only being frozen. Otherwise the Z80 stops in the
// middle of the firmware start-up (garbage screen, PSG/tape tone, CRTC not yet programmed) and the
// HDMI alignment (vreset) is taken with the wrong CRTC timing (vertical offset). The CPC is released
// when the load is done, so the firmware finds the new ROMs at its first start. Loads from the OSD
// while the CPC runs (vreset_done = 1) still only freeze the Z80.
reg         vreset_done;    // declared here; driven in the vreset block below
wire        rom_rst     = rom_busy & ~vreset_done;
wire  [1:0] cpc_reset_w = sys_reset_w | {1'b0, model_rst} | {1'b0, rom_rst};
wire        cpc_in_reset = (|cpc_reset_w) | boot_busy_w;
// The 8255 outputs port C = 0xFF in reset (mode 0x9B, ports are inputs), i.e. tape motor 'on'. Without
// this gate the mounted tape starts playing (pilot tone, tape position moves) during every reset.
wire        tape_motor_run = tape_motor_w & ~cpc_in_reset;
wire  [4:0] rom_prog;
wire        cold_w      = sys_reset_w[1];            // OSD "Cold boot" (R=3) or "Factory boot" (R=2)
wire        factory_w   = (sys_reset_w == 2'b10);   // OSD "Factory boot": also forget the ROM files
wire  [1:0] sys_floppy_wprot;
wire        sys_fdc_fast;
wire  [4:0] sys_rom_slot;
wire        sys_rom_clear;
wire        sys_rom_clearset;
wire  [1:0] sys_amsdos;
// OSD "AMSDOS": 0 = Off, 1 = On, 2 = Auto (On for 6128 / 664, Off for 464). A model change resets the CPC anyway.
wire        amsdos_eff = sys_amsdos[1] ? (sys_model != 2'd2) : sys_amsdos[0];
wire        sys_rom0_ext;
wire        sys_rom7_ext;
wire  [1:0] core_rom_ext;
wire        sys_waitgate;
wire        sys_hlock;
wire        sys_mono;
wire        sys_tapesnd;
wire  [2:0] sys_volume;
wire  [3:0] sys_vpos;
wire  [1:0] sys_vlock;                      // OSD "V lock": 0 Off, 1 On, 2 Follow
wire        sys_lock   = |sys_vlock;        // On or Follow: the row counter is locked to the CPC VSync
wire        sys_follow = sys_vlock[1];      // Follow: the HDMI frame length also follows a stable CPC frame length
wire  [2:0] sys_vrate;                      // OSD "V rate": 1 << value rows per frame
wire        vreset_any;                 // = vreset_pulse (full HDMI re-alignment; 'V pos' only loads the row counter, see below)
wire        sys_rgbout;
wire        sys_firesw;
wire        sys_numpad;
wire        sys_wide;
wire        sys_tape_inv;
wire  [7:0] core_audio_l, core_audio_r;
wire  [1:0] sys_leds;
wire [23:0] sys_color;
wire        sys_int_n;
wire  [7:0] sys_int_ack;
wire        sdc_int;

wire [31:0] sd_img_size;
wire  [7:0] sd_img_mounted;

// ---------------------------------------------------------------------------
// Deferred disk / tape mounts (2026-10-31, experiment)
// While the companion mounts the images of cpc.ini (machine held in cold reset, R=3) the u765 starts
// scanning the disk image directly on the SD card (direct mode) and the tape loader prefetches, i.e. the
// core and the companion use the SD card at the same time. Hardware result 2026-10-31: with only drive4
// in cpc.ini the ROM set was always mounted, with drive0 / drive2 as well it usually was not.
// Mount pulses of images 0..2 that arrive while the cold reset is held are therefore stored (with their
// size) and replayed one after the other when the reset is released. Mounts at any other time (OSD,
// machine running) pass through unchanged.
// (The u765 itself stays in reset until the ROM load is done and serialises the header scans of both images,
// including its own track offset table access, see 2.8 of the doc.)
// ---------------------------------------------------------------------------
wire        mnt_defer   = cold_w;
reg         mnt_defer_d = 1'b1;
reg  [2:0]  mnt_pend    = 3'd0;
reg  [31:0] mnt_sz0 = 32'd0, mnt_sz1 = 32'd0, mnt_sz2 = 32'd0;
reg         rp_act      = 1'b0;
reg  [1:0]  rp_idx      = 2'd0;
reg  [3:0]  rp_cnt      = 4'd0;
reg  [2:0]  rp_pulse    = 3'd0;
reg  [31:0] rp_size     = 32'd0;
wire  [1:0] fdc_mnt     = mnt_defer ? 2'b00 : (rp_act ? rp_pulse[1:0] : sd_img_mounted[1:0]);
wire        tape_mnt    = mnt_defer ? 1'b0  : (rp_act ? rp_pulse[2]   : sd_img_mounted[2]);
wire [31:0] mnt_size    = rp_act ? rp_size : sd_img_size;

always @(posedge clk32) begin
    mnt_defer_d <= mnt_defer;
    rp_pulse    <= 3'd0;
    if (por) begin
        mnt_pend <= 3'd0;
        rp_act   <= 1'b0;
        rp_idx   <= 2'd0;
        rp_cnt   <= 4'd0;
    end else begin
        // store the mounts that arrive while the cold reset is held (the last one of an image wins)
        if (mnt_defer) begin
            if (sd_img_mounted[0]) begin mnt_pend[0] <= 1'b1; mnt_sz0 <= sd_img_size; end
            if (sd_img_mounted[1]) begin mnt_pend[1] <= 1'b1; mnt_sz1 <= sd_img_size; end
            if (sd_img_mounted[2]) begin mnt_pend[2] <= 1'b1; mnt_sz2 <= sd_img_size; end
        end
        // reset released: replay them, one pulse every 9 clk32 cycles
        if (mnt_defer_d && !mnt_defer) begin
            rp_act <= 1'b1;
            rp_idx <= 2'd0;
            rp_cnt <= 4'd0;
        end else if (rp_act) begin
            if (rp_cnt == 4'd0) begin
                if (rp_idx == 2'd3) rp_act <= 1'b0;
                else if (mnt_pend[rp_idx]) begin
                    rp_pulse[rp_idx] <= 1'b1;
                    rp_size          <= (rp_idx == 2'd0) ? mnt_sz0 : (rp_idx == 2'd1) ? mnt_sz1 : mnt_sz2;
                    mnt_pend[rp_idx] <= 1'b0;
                    rp_cnt           <= 4'd1;
                end else
                    rp_idx <= rp_idx + 2'd1;
            end else if (rp_cnt == 4'd8) begin
                rp_idx <= rp_idx + 2'd1;
                rp_cnt <= 4'd0;
            end else
                rp_cnt <= rp_cnt + 4'd1;
        end
    end
end

wire [31:0] fdc_lba;
wire  [1:0] fdc_rd, fdc_wr;
wire  [2:0] sd_rsrc;
wire        sd_rbusy, sd_rdone;
wire        sd_outen;
wire  [8:0] sd_outaddr;
wire  [7:0] sd_outbyte;
wire  [7:0] fdc_wr_data;

wire        tape_motor_w, tape_out_w, tape_in_w, tape_present;
wire        tape_rec_led;
wire  [2:0] tape_rec_err;
wire        tape_rec_rd, tape_rec_wr;
wire [31:0] tape_rec_lba;
wire  [7:0] tape_rec_data;
wire        tape_rd;
wire [31:0] tape_lba;
wire  [7:0] arb_rstart, arb_wstart;
wire [31:0] arb_rsector;
wire        grant_fdc, grant_tape, grant_rec, grant_rom;
wire        sys_tape_prot;

// Activity LEDs (normal display): disc = SD sector transfer for the FDC (stretched ~130 ms),
// tape = tape motor on while a tape image is mounted.
// LED4 error code: <code> blinks (0.2 s on, 0.2 s off each), then 1 s pause
reg [22:0] bl_t = 23'd0;
reg  [4:0] bl_p = 5'd0;
wire [4:0] bl_len = {tape_rec_err, 1'b0} + 5'd5;   // 2 ticks per blink + 5 ticks pause
always @(posedge clk32) begin
    if (tape_rec_err == 3'd0) begin bl_t <= 23'd0; bl_p <= 5'd0; end
    else if (bl_t == 23'd6_399_999) begin
        bl_t <= 23'd0;
        bl_p <= (bl_p + 5'd1 >= bl_len) ? 5'd0 : bl_p + 5'd1;
    end else bl_t <= bl_t + 23'd1;
end
reg [21:0] disc_hold = 22'd0;
reg [21:0] rom_hold  = 22'd0;     // ROM / ROM set load shown on LED4 (stretched ~130 ms)
always @(posedge clk32) begin
    if (rom_busy)                rom_hold <= 22'h3FFFFF;
    else if (rom_hold != 22'd0)  rom_hold <= rom_hold - 22'd1;
end
always @(posedge clk32) begin
    if ((|fdc_rd) | (|fdc_wr) | (sd_rbusy & grant_fdc)) disc_hold <= 22'h3FFFFF;
    else if (disc_hold != 22'd0)                         disc_hold <= disc_hold - 22'd1;
    disc_led <= (disc_hold != 22'd0);
    tape_led <= tape_motor_w & tape_present;
    rom_led  <= (rom_hold != 22'd0);
    tape_rec_ind <= tape_rec_led | ((tape_rec_err != 3'd0) & ~bl_p[0] & (bl_p < {tape_rec_err, 1'b0}));
end
wire        rom_rd;
wire        rom_set;
wire [31:0] rom_lba;
wire [79:0] cpc_ext_key;
wire  [6:0] cpc_joy1;
wire  [6:0] cpc_joy2;

cpc_core #(.FLASH_READ_DELAY(FLASH_READ_DELAY)) core_inst (
    .clk32      ( clk32              ),
    .flash_clk  ( flash_clk         ),
    .por        ( por                ),
    .cold_boot       ( cold_w                ),
    .cpu_hold        ( rom_busy              ),
    .boot_busy       ( boot_busy_w           ),
    .sdram_rdy       ( sdram_ready           ),
    .dbg_boot_state  ( core_dbg_boot_state   ),
    // Clavier / joysticks BL616
    .ext_key         ( cpc_ext_key           ),
    .joy1_in         ( cpc_joy1              ),
    .joy2_in         ( cpc_joy2              ),
    // Disquettes (Étape 1)
    .sys_reset       ( cpc_reset_w           ),
    .model           ( sys_model             ),
    .crtc_type       ( sys_crtc              ),
    .tape_in         ( tape_in_w             ),
    .tape_motor      ( tape_motor_w          ),
    .tape_out        ( tape_out_w            ),
    .audio_l         ( core_audio_l          ),
    .audio_r         ( core_audio_r          ),
    .fdc_img_mounted ( fdc_mnt               ),   // deferred while the cold reset is held (see mnt_defer)
    .fdc_img_size    ( mnt_size              ),
    .fdc_wprot       ( sys_floppy_wprot      ),
    .fdc_fast        ( sys_fdc_fast          ),
    .fdc_sd_lba      ( fdc_lba               ),
    .fdc_sd_rd       ( fdc_rd                ),
    .fdc_sd_wr       ( fdc_wr                ),
    .fdc_sd_ack      ( sd_rbusy & grant_fdc    ),
    .fdc_buff_addr   ( sd_outaddr            ),
    .fdc_buff_dout   ( sd_outbyte            ),
    .fdc_buff_wr     ( sd_outen & grant_fdc    ),
    .fdc_buff_din    ( fdc_wr_data           ),
    // Flash SPI (via MSPI pins)
    .mspi_cs    ( mspi_cs           ),
    .mspi_di    ( mspi_di           ),
    .mspi_hold  ( mspi_hold         ),
    .mspi_wp    ( mspi_wp           ),
    .mspi_do    ( mspi_do           ),
    // Port Boot SDRAM
    .boot_addr  ( core_boot_addr    ),
    .boot_din   ( core_boot_din     ),
    .boot_dout  ( core_boot_dout    ),
    .boot_req   ( core_boot_req     ),
    .boot_we    ( core_boot_we      ),
    .boot_ack   ( core_boot_ack     ),
    // ROM map
    .rom_map    ( core_rom_map      ),
    .rom_ext    ( core_rom_ext      ),
    // Sorties vidéo (Phase 3d)
    .red        ( core_red          ),
    .green      ( core_green        ),
    .blue       ( core_blue         ),
    .hsync      ( core_hsync        ),
    .vsync      ( core_vsync        ),
    .hblank     ( core_hblank       ),
    .vblank     ( core_vblank       ),
    .field      ( core_field        ),
    .crtc_vs_raw( core_crtc_vs_raw  ),
    .dbg_r0_ok  ( core_dbg_r0_ok   ),
    .dbg_r4_ok  ( core_dbg_r4_ok   ),
    .dbg_r7_ok  ( core_dbg_r7_ok   ),
    .dbg_r9_ok  ( core_dbg_r9_ok   ),
    .ga_ready_o ( core_ga_ready    ),
    .dbg_pc     ( core_dbg_pc      ),
    .mode       ( core_mode         ),
    // Sorties mémoire (Phase 3c)
    .mem_addr   ( core_mem_addr     ),
    .mem_rd     ( core_mem_rd       ),
    .mem_wr     ( core_mem_wr       ),
    .romen      ( core_romen        ),
    .vram_addr  ( core_vram_addr    ),
    // Bus CPU (Phase 3c)
    .cpu_addr   ( core_cpu_addr     ),
    .cpu_dout   ( core_cpu_dout     ),
    .cpu_din    ( cpu_dout_w        ),  // données SDRAM → Z80 (lecture)
    .cpu_ack_in ( cpu_ack_w         ),  // SDRAM CPU ack → gating READY Z80
    .cpu_soon_in ( cpu_soon_w       ),
    .wait_gate_en ( sys_waitgate    ),
    // VRAM data (Phase 3c)
    .vram_din_i ( core_vram_din     ),  // données 16-bit SDRAM → CRTC
    // Diagnostic readback
    .readback_ok   ( core_readback_ok   ),
    .readback_done ( core_readback_done ),
    .wr_rd_ok      ( core_wr_rd_ok      ),
    .wr_rd_done    ( core_wr_rd_done    ),
    .rb_byte       ( core_rb_byte       ),
    .dbg_first_flash_word ( core_dbg_first_flash ),
    .flash_retry  ( core_flash_retry ),
    .flash_err    ( core_flash_err   ),
    .flash_retry_cnt ( core_flash_cnt )
);

// ---------------------------------------------------------------------------
// BL616 FPGA-Companion : clavier + joysticks (Phase 4)
//
//   spi_* -> mcu_spi (target id) -> mcu_hid (target 1) -> hid_cpc_matrix
//   -> cpc_core.ext_key (OR dans la matrice de hid.sv, lignes 0..9 du CPC)
//
//   Les cibles sys (0), osd (2) et sdc (3) ne sont pas encore implémentées
//   (Phase 5/6) : leurs strobes sont ignorés et leurs octets de réponse = 0.
// ---------------------------------------------------------------------------
wire        mcu_sys_strobe, mcu_hid_strobe, mcu_osd_strobe, mcu_sdc_strobe;
wire        mcu_start;
wire  [7:0] mcu_dout;
wire  [7:0] mcu_hid_dout;
wire  [7:0] sys_data_out, sdc_data_out;

mcu_spi mcu_spi_inst (
    .clk            ( clk32           ),
    .reset          ( por | spi_sw    ),   // spi_sw: re-arm the start-up guard when the SPI source switches
    .spi_io_ss      ( spi_io_ss       ),
    .spi_io_clk     ( spi_io_clk      ),
    .spi_io_din     ( spi_io_din      ),
    .spi_io_dout    ( spi_io_dout     ),
    .mcu_sys_strobe ( mcu_sys_strobe  ),
    .mcu_hid_strobe ( mcu_hid_strobe  ),
    .mcu_osd_strobe ( mcu_osd_strobe  ),
    .mcu_sdc_strobe ( mcu_sdc_strobe  ),
    .mcu_start      ( mcu_start       ),
    .mcu_sys_din    ( sys_data_out    ),
    .mcu_hid_din    ( mcu_hid_dout    ),
    .mcu_osd_din    ( 8'h00           ),
    .mcu_sdc_din    ( sdc_data_out    ),
    .mcu_dout       ( mcu_dout        )
);

wire [127:0] hid_keyboard;
wire   [7:0] hid_joy0, hid_joy1;

mcu_hid mcu_hid_inst (
    .clk            ( clk32           ),
    .rst_n          ( ~(por | spi_sw) ),   // spi_sw: forget keys / joystick seen before the SPI source switched
    .data_in_strobe ( mcu_hid_strobe  ),
    .data_in_start  ( mcu_start       ),
    .data_in        ( mcu_dout        ),
    .data_out       ( mcu_hid_dout    ),
    .keyboard       ( hid_keyboard    ),
    .joystick0      ( hid_joy0        ),
    .joystick1      ( hid_joy1        )
);

wire [79:0] bl616_key;

// OSD 'Num Joystick' On: keypad 0..9 (HID 0x59..0x62) are the joystick and are removed from the
// keyboard matrix (the PC keys F1..F10 are not affected).
wire [127:0] hid_keyboard_m = sys_numpad ? (hid_keyboard & ~(128'h3FF << 7'h59)) : hid_keyboard;

hid_cpc_matrix hid_cpc_matrix_inst (
    .kb  ( hid_keyboard_m ),
    .key ( bl616_key    )
);

// ---------------------------------------------------------------------------
// RP2040 (Pico) USB companion : UART 115200 8N1 sur spare[3] (pin 31)
// Même protocole que MSXnano ; cellules CPC (firmware/keymaps_cpc.h).
// Peut coexister avec le BL616 : touches et joysticks sont mélangés par OU.
// ---------------------------------------------------------------------------
wire [79:0] pico_key;
wire  [7:0] pico_joy0, pico_joy1;
wire        pico_alive;

kbd_uart_rx_cpc #(.CLK_FREQ(32_000_000), .BAUD(115200)) kbd_uart_inst (
    .clk        ( clk32       ),
    .rst_n      ( ~por        ),
    .rx         ( spare_rx    ),
    .ext_key    ( pico_key    ),
    .joy_state0 ( pico_joy0   ),
    .joy_state1 ( pico_joy1   ),
    .link_alive ( pico_alive  )
);

// Power-on keyboard mask: ignore both keyboard sources for ~2 s after the ROMs are loaded. A stray
// "cursor right" (with a key click) appeared at power-on even without joystick; this keeps any
// start-up glitch of the BL616 / Pico links away from the firmware's first keyboard scans.
reg [26:0] kb_mask_cnt = 27'd0;
always @(posedge clk32) begin
    if (por || boot_busy_w)        kb_mask_cnt <= 27'd0;
    else if (!kb_mask_cnt[26])     kb_mask_cnt <= kb_mask_cnt + 27'd1;
end
assign cpc_ext_key = kb_mask_cnt[26] ? (bl616_key | pico_key) : 80'd0;

// Companion joystick (actif haut) : [0]=R [1]=L [2]=D [3]=U [4]=bouton A [5]=bouton B
// -> format hid.sv : [3:0] idem, [5]=Fire1 (bouton A), [4]=Fire2 (bouton B), [6]=Fire3 (inutilisé)
// DB9 joystick on the Tang Nano header (switches to GND, pull-ups in the .cst = active low):
// io[0]=BTN0 io[1]=DOWN io[2]=UP io[3]=RIGHT io[4]=LEFT io[5]=BTN1  -> CPC joystick 1
reg [5:0] db9_s1, db9_s2;
// Start-up/noise protection: the pins are ignored (released) for ~0.26 s after power-on and a
// change is only accepted when it is stable for two samples 1 ms apart (the CPC firmware shows
// a cursor key if a direction is seen while it boots).
reg [5:0]  db9_last = 6'h3F, db9_ok = 6'h3F;
reg [14:0] db9_t = 15'd0;
reg [23:0] db9_w = 24'd0;
always @(posedge clk32) begin
    db9_s1 <= io; db9_s2 <= db9_s1;
    db9_t  <= db9_t + 15'd1;
    if (por) begin
        db9_w  <= 24'd0;
        db9_ok <= 6'h3F;
    end else begin
        if (!db9_w[23]) db9_w <= db9_w + 24'd1;
        if (db9_t == 15'd0) begin
            db9_last <= db9_s2;
            if (db9_w[23] && db9_last == db9_s2) db9_ok <= db9_s2;
        end
    end
end
// OSD 'Fire swap': exchange BTN0 (io[0], Fire 1) and BTN1 (io[5], Fire 2)
wire db9_b0 = sys_firesw ? ~db9_ok[5] : ~db9_ok[0];
wire db9_b1 = sys_firesw ? ~db9_ok[0] : ~db9_ok[5];
wire [7:0] db9_joy = { 2'b00, db9_b1, db9_b0, ~db9_ok[2], ~db9_ok[1], ~db9_ok[4], ~db9_ok[3] };

// OSD 'Num Joystick': keypad 8/2/4/6 = up/down/left/right, 7/9/1/3 = diagonals (two directions),
// 5 = BTN1 (X, like FPGA pin 30), 0 = BTN0 (Z, like pin 27); not affected by the DB9 fire swap.
// Format as joy_a: [0]=R [1]=L [2]=D [3]=U [4]=BTN0 [5]=BTN1.
wire np_on = sys_numpad & kb_mask_cnt[26];
wire np_u  = np_on & (hid_keyboard[7'h60] | hid_keyboard[7'h5F] | hid_keyboard[7'h61]);   // 8 7 9
wire np_d  = np_on & (hid_keyboard[7'h5A] | hid_keyboard[7'h59] | hid_keyboard[7'h5B]);   // 2 1 3
wire np_l  = np_on & (hid_keyboard[7'h5C] | hid_keyboard[7'h5F] | hid_keyboard[7'h59]);   // 4 7 1
wire np_r  = np_on & (hid_keyboard[7'h5E] | hid_keyboard[7'h61] | hid_keyboard[7'h5B]);   // 6 9 3
wire np_b1 = np_on & hid_keyboard[7'h5D];                                                 // 5 = BTN1
wire np_b0 = np_on & hid_keyboard[7'h62];                                                 // 0 = BTN0
wire [7:0] np_joy = { 2'b00, np_b1, np_b0, np_u, np_d, np_l, np_r };

wire [7:0] joy_a = hid_joy0 | pico_joy0 | db9_joy | np_joy;
wire [7:0] joy_b = hid_joy1 | pico_joy1;
assign cpc_joy1 = { 1'b0, joy_a[4], joy_a[5], joy_a[3:0] };
assign cpc_joy2 = { 1'b0, joy_b[4], joy_b[5], joy_b[3:0] };

// ---------------------------------------------------------------------------
// Système + SD card + disquettes (Étape 1)
//
//   sysctrl (cible 0) : menu OSD (cpc_xml.hex), reset, protection, vitesse FDC
//   sd_card (cible 3) : le companion monte les images (.dsk) ; le coeur lit
//                       les secteurs via rstart/rsector (source 0 = lecteur A,
//                       1 = lecteur B), les données arrivent par outen/outbyte.
//   int_in = {sdc_int sur le bit 3} comme dans C64Nano.
// ---------------------------------------------------------------------------
sysctrl sysctrl_inst (
    .clk                 ( clk32            ),
    .reset               ( por              ),
    .data_in_strobe      ( mcu_sys_strobe   ),
    .data_in_start       ( mcu_start        ),
    .data_in             ( mcu_dout         ),
    .data_out            ( sys_data_out     ),
    .int_out_n           ( sys_int_n        ),
    .int_in              ( {4'b0000, sdc_int, 3'b000} ),
    .int_ack             ( sys_int_ack      ),
    .buttons             ( 2'b00            ),
    .leds                ( sys_leds         ),
    .color               ( sys_color        ),
    .port_status         ( 32'd0            ),
    .port_out_available  ( 8'd0             ),
    .port_out_strobe     (                  ),
    .port_out_data       ( 8'd0             ),
    .port_in_available   ( 8'd0             ),
    .port_in_strobe      (                  ),
    .port_in_data        (                  ),
    .system_reset        ( sys_reset_w      ),
    .system_floppy_wprot ( sys_floppy_wprot ),
    .system_fdc_fast     ( sys_fdc_fast     ),
    .system_rom_slot     ( sys_rom_slot     ),
    .system_rom_clear    ( sys_rom_clear    ),
    .system_rom_clearset ( sys_rom_clearset ),
    .system_amsdos       ( sys_amsdos       ),
    .system_rom0_ext     ( sys_rom0_ext     ),
    .system_rom7_ext     ( sys_rom7_ext     ),
    .system_waitgate     ( sys_waitgate     ),
    .system_hlock        ( sys_hlock        ),
    .system_mono         ( sys_mono         ),
    .system_tapesnd      ( sys_tapesnd      ),
    .system_volume       ( sys_volume       ),
    .system_vpos         ( sys_vpos         ),
    .system_vlock        ( sys_vlock        ),
    .system_vrate        ( sys_vrate        ),
    .system_rgbout       ( sys_rgbout       ),
    .system_firesw       ( sys_firesw       ),
    .system_numpad       ( sys_numpad       ),
    .system_wide         ( sys_wide         ),
    .system_model       ( sys_model        ),
    .system_crtc        ( sys_crtc         ),
    .system_hpos         ( sys_hpos         ),
    .system_tape_inv     ( sys_tape_inv     ),
    .system_tape_prot    ( sys_tape_prot    ),
    .system_w800         ( sys_w800         )
);

assign spi_intn = sys_int_n;

sd_arb sd_arb_inst (
    .clk        ( clk32        ),
    .reset      ( por          ),
    .fdc_lba    ( fdc_lba      ),
    .fdc_rd     ( fdc_rd       ),
    .fdc_wr     ( fdc_wr       ),
    .tape_lba   ( tape_lba     ),
    .tape_rd    ( tape_rd      ),
    .rec_lba    ( tape_rec_lba ),
    .rec_rd     ( tape_rec_rd  ),
    .rec_wr     ( tape_rec_wr  ),
    .rom_lba    ( rom_lba      ),
    .rom_rd     ( rom_rd       ),
    .rom_set    ( rom_set      ),
    .rbusy      ( sd_rbusy     ),
    .rstart     ( arb_rstart   ),
    .wstart     ( arb_wstart   ),
    .rsector    ( arb_rsector  ),
    .grant_fdc  ( grant_fdc    ),
    .grant_tape ( grant_tape   ),
    .grant_rec  ( grant_rec    ),
    .grant_rom  ( grant_rom    )
);

// Cassette (Étape 2) : image 2 de l'OSD, lecture CDT/TZX, signal -> PPI port B bit 7
cpc_tape cpc_tape_inst (
    .clk          ( clk32                    ),
    .reset        ( por                      ),
    .img_mounted  ( tape_mnt                 ),   // deferred while the cold reset is held (see mnt_defer)
    .img_size     ( mnt_size                 ),
    .motor        ( tape_motor_run           ),
    .wr_en        ( ~sys_tape_prot           ),
    .tape_out     ( tape_out_w               ),
    .rec_active   ( tape_rec_led             ),
    .rec_err      ( tape_rec_err             ),
    .invert       ( sys_tape_inv             ),
    .tape_in      ( tape_in_w                ),
    .present      ( tape_present             ),
    .sd_rd        ( tape_rd                  ),
    .sd_lba       ( tape_lba                 ),
    .sd_busy      ( sd_rbusy & grant_tape    ),
    .sd_buff_addr ( sd_outaddr               ),
    .sd_buff_dout ( sd_outbyte               ),
    .sd_buff_wr   ( sd_outen & grant_tape    ),
    .rec_sd_rd    ( tape_rec_rd              ),
    .rec_sd_wr    ( tape_rec_wr              ),
    .rec_sd_lba   ( tape_rec_lba             ),
    .rec_sd_busy  ( sd_rbusy & grant_rec     ),
    .rec_buff_wr  ( sd_outen & grant_rec     ),
    .rec_wr_data  ( tape_rec_data            )
);

// ROMs externes (Étape 3) : image 3 de l'OSD -> slot choisi (0..31)
cpc_romload cpc_romload_inst (
    .clk          ( clk32                    ),
    .reset        ( por | cold_w            ),
    .img_mounted  ( sd_img_mounted[3]        ),
    .set_mounted  ( sd_img_mounted[4]        ),
    .img_size     ( sd_img_size              ),
    .slot         ( sys_rom_slot             ),
    .clear        ( sys_rom_clear            ),
    .clear_set    ( sys_rom_clearset         ),
    .amsdos_en    ( amsdos_eff               ),
    .rom0_ext     ( sys_rom0_ext             ),
    .rom7_ext     ( sys_rom7_ext             ),
    .boot_idle    ( core_wr_rd_done          ),
    .prog         ( rom_prog                 ),
    .forget       ( factory_w                ),
    .sd_rd        ( rom_rd                   ),
    .sd_set       ( rom_set                  ),
    .sd_lba       ( rom_lba                  ),
    .sd_busy      ( sd_rbusy & grant_rom     ),
    .sd_buff_addr ( sd_outaddr               ),
    .sd_buff_dout ( sd_outbyte               ),
    .sd_buff_wr   ( sd_outen & grant_rom     ),
    .ram_addr     ( rl_addr                  ),
    .ram_din      ( rl_din                   ),
    .ram_req      ( rl_req                   ),
    .ram_ack      ( boot_ack_w               ),
    .rom_map      ( core_rom_map             ),
    .ext_sel      ( core_rom_ext             ),
    .busy         ( rom_busy                 )
);

sd_card #(.CLK_DIV(3'd2), .SIMULATE(0), .IMAGE_FIFO_BITS(1)) sd_card_inst (
    .rstn                       ( ~por             ),
    .clk                        ( clk32            ),
    .sdclk                      ( sd_clk           ),
    .sdcmd                      ( sd_cmd           ),
    .sddat                      ( sd_dat           ),
    .data_strobe                ( mcu_sdc_strobe   ),
    .data_start                 ( mcu_start        ),
    .data_in                    ( mcu_dout         ),
    .data_out                   ( sdc_data_out     ),
    .irq                        ( sdc_int          ),
    .iack                       ( sys_int_ack[3]   ),
    .image_size                 ( sd_img_size      ),
    .image_mounted              ( sd_img_mounted   ),
    .rom_image_selection_strobe (                  ),
    .rom_image_selected         (                  ),
    .rom_image_accepted         ( 1'b0             ),
    .rom_image_data_available   (                  ),
    .rom_image_data             (                  ),
    .rom_image_data_strobe      ( 1'b0             ),
    .rstart                     ( arb_rstart       ),
    .wstart                     ( arb_wstart       ),
    .rsector                    ( arb_rsector      ),
    .rsrc                       ( sd_rsrc          ),
    .rbusy                      ( sd_rbusy         ),
    .rdone                      ( sd_rdone         ),
    .inbyte                     ( grant_rec ? tape_rec_data : fdc_wr_data ),
    .outen                      ( sd_outen         ),
    .outaddr                    ( sd_outaddr       ),
    .outbyte                    ( sd_outbyte       )
);

// ---------------------------------------------------------------------------
// Pipeline vidéo CPC → HDMI (Phase 3d)
//
//   GA40010  : red[1:0], green[1:0], blue[1:0], hsync, vsync, hblank, vblank
//      ↓ color_mix (2-bit → 8-bit palette CPC)
//   R[7:0], G[7:0], B[7:0]
//      ↓ scandoubler (15kHz → 31kHz, 4-bit input = R[7:4] G[7:4] B[7:4])
//   r_sd[5:0], g_sd[5:0], b_sd[5:0], hs_sd, vs_sd
//      ↓ video2hdmi (6-bit → TMDS)
//   tmds_clk_p/n, tmds_d_p/n
// ---------------------------------------------------------------------------

// Pixel clock enable : 16 MHz depuis clk32 (même logique que ce_16_r dans cpc_core)
reg ce_pix_r;
always @(posedge clk32) ce_pix_r <= ~ce_pix_r;

// -- Étage 1 : color_mix — palette CPC (2-bit RGB → 8-bit RGB) --
wire [7:0] cm_r, cm_g, cm_b;
wire       cm_hs, cm_vs, cm_hb, cm_vb;

color_mix color_mix_inst (
    .clk_vid    ( clk32         ),
    .ce_pix     ( ce_pix_r      ),
    .mix        ( {1'b0, sys_mono, 1'b0} ),    // 0 = couleurs GA40010, 2 = green monitor (OSD)

    .R_in       ( core_red      ),
    .G_in       ( core_green    ),
    .B_in       ( core_blue     ),
    .HSync_in   ( core_hsync    ),
    .VSync_in   ( core_vsync    ),
    .HBlank_in  ( core_hblank   ),
    .VBlank_in  ( core_vblank   ),

    .R_out      ( cm_r          ),
    .G_out      ( cm_g          ),
    .B_out      ( cm_b          ),
    .HSync_out  ( cm_hs         ),
    .VSync_out  ( cm_vs         ),
    .HBlank_out ( cm_hb         ),
    .VBlank_out ( cm_vb         )
);

// -- 15 kHz RGB + CSYNC output (like the original CPC, before the scandoubler) --
// GA colour code per channel {level, OE_n}: 00 = 0, X1 (OE_n=1, high-Z) = 1/2, 10 = full.
reg [1:0] rgb_r_r, rgb_g_r, rgb_b_r;
reg       csync_r;
wire      core_blank = core_hblank | core_vblank;
always @(posedge clk32) begin
    rgb_r_r <= core_blank ? 2'b00 : {core_red[0]   | core_red[1],   core_red[1]   & ~core_red[0]};
    rgb_g_r <= core_blank ? 2'b00 : {core_green[0] | core_green[1], core_green[1] & ~core_green[0]};
    rgb_b_r <= core_blank ? 2'b00 : {core_blue[0]  | core_blue[1],  core_blue[1]  & ~core_blue[0]};
    csync_r <= ~(core_hsync ^ core_vsync);
end
// Fixed RGB phase: the RGB lines are delayed against CSYNC (csync_r, never delayed). A converter that samples
// the RGB lines with its own fixed phase (VGA4CPC) flickers when the exact 16 MHz pixel edges coincide with its
// sampling instant. Measured on hardware (old 6.25 ns chain / half-period steps): VGA4CPC quiet at +16 and
// +31 ns, flicker at 0 ns (left border at +47 ns); RGBtoHDMI quiet at 0, +16 and +47 ns, flicker at +31 ns.
// -> +16 ns suits both and is the value used here.
//
// The delay is set by WHICH register drives the pins (all in the clk32 domain, no clock crossing; one step =
// half a clk32 period = 15.6 ns; the CPC pixel is 62.5 ns):
//     delay (this file)      pins driven by                                          old menu label (e8e7d04)
//       0 ns                 {rgb_r_r, rgb_g_r, rgb_b_r}  (posedge clk32 registers)  "-16 ns"
//     +16 ns  (USED)         rgb_n1   (negedge clk32 copy of the above)              "0 ns"
//     +31 ns                 rgb_p2   (posedge copy of rgb_n1)                       "+16 ns"
//     +47 ns                 rgb_n2   (negedge copy of rgb_p2)                       "+31 ns"
// To change it: add the copy registers, e.g.
//     reg [5:0] rgb_p2; always @(posedge clk32) rgb_p2 <= rgb_n1;      // +31 ns
//     reg [5:0] rgb_n2; always @(negedge clk32) rgb_n2 <= rgb_p2;      // +47 ns
// and use that register in the three assigns below.
// Intermediate values are not possible in this domain (resolution = half a clk32 period). 6.25 ns steps need a
// clk160 delay chain (tried in e760958 / 97de0e7: clk32 -> clk160 crossing, timing violations, boot failures);
// an IODELAY primitive on the output pins would be the other way (not tried).
reg [5:0] rgb_n1;
always @(negedge clk32) rgb_n1 <= {rgb_r_r, rgb_g_r, rgb_b_r};
assign rgb_r = sys_rgbout ? rgb_n1[5:4] : 2'bzz;
assign rgb_g = sys_rgbout ? rgb_n1[3:2] : 2'bzz;
assign rgb_b = sys_rgbout ? rgb_n1[1:0] : 2'bzz;
assign csync = sys_rgbout ? csync_r : 1'bz;

// -- Étage 2 : scandoubler — 15kHz → 31kHz --
// ce_divider=1 : ce_x1 = i_div[0] = 16MHz (sample entrée), ce_x2 = 1'b1 (32MHz sortie)
wire [5:0] sd_r, sd_g, sd_b;
wire       sd_hs, sd_vs;

scandoubler #(.HCNT_WIDTH(10)) scandoubler_inst (
    .clk_sys    ( clk32         ),
    .bypass     ( 1'b0          ),    // activer le doubleur de lignes
    .ce_divider ( 1'b1          ),    // entrée 16 MHz sur clk_sys 32 MHz
    .scanlines  ( 2'b00         ),    // pas de scanlines

    .r_in       ( cm_r[7:4]     ),    // 4 bits MSB du 8-bit
    .g_in       ( cm_g[7:4]     ),
    .b_in       ( cm_b[7:4]     ),
    .hs_in      ( cm_hs         ),
    .vs_in      ( cm_vs         ),

    .h_offset   ( h_offset_reg[9:0] ),  // décalage horizontal ajustable
    .grid_rst   ( vreset_any    ),      // HDMI row grid reference (see scandoubler)
    .hcomp_en   ( sys_hlock     ),      // OSD 'H lock'
    .pixel_ena  (               ),    // non utilisé
    .r_out      ( sd_r          ),
    .g_out      ( sd_g          ),
    .b_out      ( sd_b          ),
    .hs_out     ( sd_hs         ),
    .vs_out     ( sd_vs         )
);

// -- Étage 3 : video2hdmi — TMDS --
// Audio : YM2149 (8 bits non signés, ~0..190) -> 16 bits signés, composante continue retirée
// OSD 'Tape sound': the tape signal the CPC reads (square wave) is mixed into both channels while
// the motor is on. The base range is -12000..+20640, so +-2500 cannot overflow.
wire signed [15:0] tape_snd = (sys_tapesnd && tape_motor_run) ? ((tape_in_w ^ tape_out_w) ? 16'sd2500 : -16'sd2500) : 16'sd0;
wire signed [15:0] mix_l16 = $signed({1'b0, core_audio_l, 7'd0}) - 16'sd12000 + tape_snd;
wire signed [15:0] mix_r16 = $signed({1'b0, core_audio_r, 7'd0}) - 16'sd12000 + tape_snd;
// OSD 'Volume' (id V): 0 / 25 / 50 / 75 / 100 % of the mixed signal (arithmetic shifts, no multiplier)
function automatic signed [15:0] vol_scale(input signed [15:0] x, input [2:0] v);
    case (v)
        3'd0:    vol_scale = 16'sd0;
        3'd1:    vol_scale = x >>> 2;
        3'd2:    vol_scale = x >>> 1;
        3'd3:    vol_scale = (x >>> 1) + (x >>> 2);
        default: vol_scale = x;
    endcase
endfunction
wire signed [15:0] audio_l16 = vol_scale(mix_l16, sys_volume);
wire signed [15:0] audio_r16 = vol_scale(mix_r16, sys_volume);
wire [15:0] audio_zero [1:0];
assign audio_zero[0] = audio_l16;
assign audio_zero[1] = audio_r16;

// Vidéo CPC vers HDMI
wire [5:0] vid_r, vid_g, vid_b;
wire  [9:0] hcx, hcy;                         // HDMI column / row counter (declared here: used by the OSD and the V controller)

osd_u8g2 osd_inst (
    .clk            ( clk32           ),
    .reset          ( por             ),
    .data_in_strobe ( mcu_osd_strobe  ),
    .data_in_start  ( mcu_start       ),
    .data_in        ( mcu_dout        ),
    .hs             ( sd_hs           ),
    .vs             ( sd_vs           ),
    .grid_en        ( 1'b1            ),   // OSD fixed to the HDMI raster (not to the CPC HSync / VSync)
    .gx             ( hcx             ),
    .gy             ( hcy             ),
    .gx_off         ( sys_w800 ? 10'd112 : 10'd152 ),   // 512 - window centre column (400 resp. 360)
    .r_in           ( sd_r            ),
    .g_in           ( sd_g            ),
    .b_in           ( sd_b            ),
    .r_out          ( vid_r           ),
    .g_out          ( vid_g           ),
    .b_out          ( vid_b           )
);

// vreset: the HDMI frame counters (cx, cy) are forced to 0 once so that the HDMI frame
// matches the CPC picture. It used to fire at the first vsync after any reset, i.e. while
// the CRTC still ran its start-up frames (cold boot: even before the CPC was released),
// and at an arbitrary horizontal phase, so the picture sat too far left and the HDMI row
// boundary fell inside the visible line (a 1-line step at a fixed column). Now:
//   - re-armed when the CPC is released from reset AND from the boot loader (boot_busy),
//   - skips 16 frames (0.3 s) so the firmware has programmed the CRTC,
//   - then fires at the next scandoubler line boundary (falling edge of sd_hs), so the
//     HDMI row start always coincides with the start of a scandoubler line.
// The horizontal picture position is set by h_offset (OSD "H pos").
reg sd_vs_d, sd_hs_d;
reg vreset_wait_hs;
reg [4:0] vreset_frames;
reg vreset_pulse;
reg sys_rst_d;
always @(posedge clk32) begin
    if (por) begin
        sys_rst_d       <= 1'b0;
        sd_vs_d         <= 1'b0;
        sd_hs_d         <= 1'b0;
        vreset_done     <= 1'b0;
        vreset_wait_hs  <= 1'b0;
        vreset_frames   <= 5'd0;
        vreset_pulse    <= 1'b0;
    end else begin
        sd_vs_d <= sd_vs;
        sd_hs_d <= sd_hs;
        vreset_pulse <= 1'b0;
        sys_rst_d <= cpc_in_reset;
        if (cpc_in_reset || (sys_rst_d && !cpc_in_reset)) begin
            vreset_done    <= 1'b0;               // (re-)arm; counting starts after the release
            vreset_wait_hs <= 1'b0;
            vreset_frames  <= 5'd0;
        end else if (!vreset_done) begin
            if (sd_vs_d && !sd_vs) begin
                if (vreset_frames == 5'd16) vreset_wait_hs <= 1'b1;
                else                         vreset_frames <= vreset_frames + 5'd1;
            end
            if (vreset_wait_hs && sd_hs_d && !sd_hs) begin
                vreset_pulse   <= 1'b1;
                vreset_done    <= 1'b1;
                vreset_wait_hs <= 1'b0;
            end
        end
    end
end

// OSD 'V pos' (id Z): the HDMI row counter is set to v_start when the HDMI frame is aligned to the CPC
// VSync (vreset). Like 'H pos' it moves the whole picture (blue area, border, text): one step = 16 HDMI
// rows (= 4 CPC lines = half a character row; 2026-10-29: was 16 rows); the row counter grows downwards, so a
// larger value at the CPC VSync puts the picture lower. v_start = 568 + 8*(v - 8) mod 624, v = 8 (label 0) = 568
// (the old label-0 position 560 plus half a character: the picture sat ~4 CPC lines too high compared with a real CPC).
wire [10:0] v_raw   = 11'd504 + {4'd0, sys_vpos, 3'd0};                  // 568 + 8*(v-8)
wire  [9:0] v_start = (v_raw >= 11'd624) ? (v_raw - 11'd624) : v_raw[9:0];
// HDMI frame height in rows: 624, or with V lock 'Follow' the CPC frame length (see below). The V controller works
// modulo vtot; v_tgt = the V pos row position reduced to the current frame (= v_start for vtot 624).
reg   [9:0] vtot = 10'd624;
wire [10:0] vt11  = {1'b0, vtot};
wire [10:0] vt22  = {vtot, 1'b0};
wire  [9:0] v_tgt = (v_raw >= vt11) ? (v_raw - vt11) : v_raw[9:0];   // v_raw = 504..624 (V pos step 8 rows)
// V pos / V lock controller. The picture is moved WITHOUT a full HDMI reset (which also restarts the
// horizontal counter and the TMDS serializers, so the monitor had to re-sync): after the CPC VSync (and the
// next line boundary) the phase of the HDMI row counter is measured, and the row counter is loaded once
// (cy_arm, applied at the next row boundary) with its natural next value plus a correction of at most 16 rows
// (= one frame of 624+-16 rows). Natural next value = hcy + 1, so the load only adds the correction.
//   V lock Off : free running HDMI frame; the correction only walks the picture from the old to the new V pos
//                (v_cur = the phase the frame is supposed to have; measured drift is NOT corrected).
//   V lock On  : the HDMI row counter is locked to the CPC VSync like a monitor: the measured phase A (row
//                index at the row boundary nearest to the VSync/hs event) is moved towards v_start by at most
//                16 rows per frame, so frame length changes, R7 changes and the irregular frames when a demo
//                reprograms the CRTC are followed (a large error scrolls back within a few frames).
// Distance is taken the short way round the frame (vtot rows). A full vreset (start-up/reset) sets v_cur.
//   V lock Follow : as On, plus the HDMI frame length: when the CPC frame length has been identical for 16 frames
//                -- an exact multiple of 1024 clk32 (= 2 HDMI rows = one CPC line is 2048 clk32), 600..650 rows -- the
//                HDMI frame is set to exactly that many rows (Jet Set Willy: 310 CPC lines = 620 rows, 50.40 Hz), so the
//                picture no longer rolls against the HDMI frame. If the length no longer matches for 8 frames the
//                frame goes back to 624 rows; a different stable length is taken over directly.
//                Every change of the frame length makes the sink re-sync (picture + sound drop out, always the same
//                time), so Follow keeps the number of changes small: irregular frames are never chased, and the
//                alignment to V pos is ONE jump (no 'V rate' walk): right at a change of vtot (vp_one, in the same
//                frame as the length change) and later after 3 regular frames, when the error is >= 4 rows.
reg   [9:0] v_cur = 10'd568;
reg   [9:0] vtot_d = 10'd624;                // vtot of the previous cycle
reg         vp_one = 1'b0;                   // vtot just changed: the next pass aligns at once (no regular-frame gate, deadband 1 row)
reg   [3:0] vp_st = 4'd0;
reg         vp_wait_hs = 1'b0, vp_arm = 1'b0;
reg   [9:0] cap_cx = 10'd0, cap_cy = 10'd0, vp_val = 10'd0, vp_add = 10'd0;
reg  [10:0] vp_a = 11'd0, vp_ref = 11'd0, vp_dfw = 11'd0, vp_mag = 11'd0, vp_tmp = 11'd0;
reg         vp_fwd = 1'b0;
reg  [10:0] vp_e = 11'd0;                    // measured phase minus expected phase (V lock On), mod 624
reg   [2:0] vp_cnt = 3'd0;                   // consecutive frames with no irregularity (V lock On)
wire [10:0] vp_rate = 11'd1 << sys_vrate;     // largest correction per frame (rows)
always @(posedge clk32) begin
    vp_arm <= 1'b0;
    vtot_d <= vtot;
    if (vtot != vtot_d) vp_one <= 1'b1;
    if (por) begin
        v_cur <= 10'd568; vp_wait_hs <= 1'b0; vp_st <= 4'd0; vp_cnt <= 3'd0;
    end else if (vreset_pulse) begin
        v_cur <= v_tgt; vp_wait_hs <= 1'b0; vp_st <= 4'd0; vp_cnt <= 3'd0;
    end else if (cpc_in_reset) begin
        vp_wait_hs <= 1'b0; vp_st <= 4'd0;
    end else begin
        case (vp_st)
        4'd0: begin                              // wait for a CPC VSync, then for the next line boundary
            if ((sys_lock || v_cur != v_tgt) && sd_vs_d && !sd_vs) vp_wait_hs <= 1'b1;
            if (vp_wait_hs && sd_hs_d && !sd_hs) begin
                cap_cx <= hcx; cap_cy <= hcy; vp_wait_hs <= 1'b0; vp_st <= 4'd1;
            end
        end
        4'd1: begin                              // phase A = row index of the row boundary nearest to the event
            vp_tmp <= (cap_cx >= 10'd512) ? ({1'b0, cap_cy} + 11'd1) : {1'b0, cap_cy};
            vp_st <= 4'd2;
        end
        4'd2: begin
            vp_a   <= (vp_tmp >= vt11) ? (vp_tmp - vt11) : vp_tmp;
            vp_st <= 4'd3;
        end
        4'd3: begin                              // reference: measured phase (lock) or the assumed one
            vp_ref <= sys_lock ? vp_a : {1'b0, v_cur};
            // V lock On: error of the measured phase against the phase expected from the previous frame
            // (v_cur already contains our own correction): 0 or +-1 row = regular frame
            vp_e   <= (vp_a >= {1'b0, v_cur}) ? (vp_a - {1'b0, v_cur}) : (vp_a + vt11 - {1'b0, v_cur});
            vp_st <= 4'd4;
        end
        4'd4: begin                              // distance forwards from ref to v_start (0..623)
            vp_cnt <= (vp_e <= 11'd1 || vp_e + 11'd1 >= vt11) ? ((vp_cnt == 3'd7) ? 3'd7 : vp_cnt + 3'd1) : 3'd0;
            vp_dfw <= ({1'b0, v_tgt} >= vp_ref) ? ({1'b0, v_tgt} - vp_ref)
                                                : ({1'b0, v_tgt} + vt11 - vp_ref);
            vp_st <= 4'd5;
        end
        4'd5: begin                              // shortest way round, at most 16 rows
            vp_fwd <= (vp_dfw <= {1'b0, vtot[9:1]});
            vp_tmp <= (vp_dfw <= {1'b0, vtot[9:1]}) ? vp_dfw : (vt11 - vp_dfw);
            vp_st <= 4'd6;
        end
        4'd6: begin
            // V lock On: only after 3 regular frames in a row (no chasing of the irregular frames while a
            // demo reprograms the CRTC: every correction makes a frame 624+-n rows long and the monitor
            // lost the signal), and only 4 rows per frame; V lock Off (V pos walk): 16 rows per frame
            // (rate = OSD 'V rate', 1 << 0..4 rows per frame, default 4; before it was 4 for V lock and 16 for the V pos walk)
            // V lock Follow: one jump (see above); deadband 1 row right after a vtot change, else 4 rows (a V pos step is 8, rounding +-1)
            if (sys_follow)
                           vp_mag <= (vp_one ? vp_tmp <= 11'd1 : (vp_cnt < 3'd3 || vp_tmp < 11'd4)) ? 11'd0 : vp_tmp;
            else if (sys_lock) vp_mag <= (vp_cnt < 3'd3) ? 11'd0 : ((vp_tmp > vp_rate) ? vp_rate : vp_tmp);
            else           vp_mag <= (vp_tmp > vp_rate) ? vp_rate : vp_tmp;
            vp_st <= 4'd7;
        end
        4'd7: begin                              // correction as a forwards value 0..623, new v_cur = ref + correction
            vp_add <= vp_fwd ? vp_mag[9:0] : (vtot - vp_mag[9:0]);
            vp_st <= 4'd8;
        end
        4'd8: begin
            vp_tmp <= vp_ref + {1'b0, vp_add};
            vp_st <= 4'd9;
        end
        4'd9: begin
            v_cur <= (vp_tmp >= vt11) ? (vp_tmp - vt11) : vp_tmp[9:0];
            vp_one <= 1'b0;
            vp_st <= (vp_mag == 11'd0) ? 4'd0 : 4'd10;     // nothing to correct: done
        end
        4'd10: if (hcx >= 10'd64 && hcx < 10'd960) begin    // safely inside a row: no boundary in the next cycles
            vp_tmp <= {1'b0, hcy} + 11'd1 + {1'b0, vp_add};
            vp_st <= 4'd11;
        end
        4'd11: begin
            vp_val <= (vp_tmp >= vt22) ? (vp_tmp - vt22) :
                      (vp_tmp >= vt11) ? (vp_tmp - vt11) : vp_tmp[9:0];
            vp_st <= 4'd12;
        end
        4'd12: begin vp_arm <= 1'b1; vp_st <= 4'd0; end
        default: vp_st <= 4'd0;
        endcase
    end
end
assign vreset_any = vreset_pulse;

// V lock Follow: frame length measurement and the choice of the HDMI frame height (vtot).
// The CPC frame length is measured in clk32 cycles between two CPC VSync events (sd_vs falling edge, the event the V
// controller uses). A CPC line is 2048 clk32 (R0 = 63) = 2 HDMI rows of 1024, so a frame with constant R0 is an exact
// multiple of 1024 clk32 = whole HDMI rows; anything else (odd lengths, R0 changes) is 'not stable'.
//   fl_same  consecutive frames with the same valid length (candidate fl_cand, rows), saturates at 31
//   fl_bad   consecutive frames whose length differs from the current HDMI frame (vtot), saturates at 15
// Follow takes over a candidate after 16 identical frames; it goes back to 624 after 8 non-matching frames; with
// V lock Off / On vtot is 624 (the next frame after the switch, the sink sees one frame length change).
reg [19:0] fl_cnt  = 20'd0;
reg [19:0] fl_len  = 20'd0;
reg        fl_new  = 1'b0, fl_new2 = 1'b0;
reg  [9:0] fl_cand = 10'd624;
reg  [4:0] fl_same = 5'd0;
reg  [3:0] fl_bad  = 4'd0;
wire [9:0] fl_rows = fl_len[19:10];
wire       fl_ok   = (fl_len[9:0] == 10'd0) && (fl_rows >= 10'd600) && (fl_rows <= 10'd650);
always @(posedge clk32) begin
    fl_new  <= 1'b0;
    fl_new2 <= fl_new;
    if (por) begin
        fl_cnt <= 20'd0; fl_same <= 5'd0; fl_bad <= 4'd0; vtot <= 10'd624;
    end else if (cpc_in_reset) begin
        fl_cnt <= 20'd0; fl_same <= 5'd0; fl_bad <= 4'd0;
    end else begin
        if (!(&fl_cnt)) fl_cnt <= fl_cnt + 20'd1;
        if (sd_vs_d && !sd_vs) begin
            fl_len <= fl_cnt + 20'd1;               // 0 on overflow = invalid
            fl_cnt <= 20'd0;
            fl_new <= 1'b1;
        end
        if (fl_new) begin                           // fl_len is valid
            if (fl_ok && fl_rows == fl_cand) fl_same <= (&fl_same) ? fl_same : fl_same + 5'd1;
            else if (fl_ok)                  begin fl_cand <= fl_rows; fl_same <= 5'd1; end
            else                             fl_same <= 5'd0;
            if (fl_ok && fl_rows == vtot)    fl_bad <= 4'd0;
            else if (!(&fl_bad))             fl_bad <= fl_bad + 4'd1;
        end
        if (fl_new2) begin                          // decide with the updated counters
            if (!sys_follow) begin
                if (vtot != 10'd624) vtot <= 10'd624;
            end else if (fl_same >= 5'd16 && fl_cand != vtot) vtot <= fl_cand;
            else if (fl_bad >= 4'd8 && vtot != 10'd624)       vtot <= 10'd624;
        end
    end
end

video2hdmi video2hdmi_inst (
    .clk_pixel_x5 ( clk160       ),  // 160 MHz TMDS
    .clk_pixel    ( clk32        ),  // 32 MHz pixel
    .vreset       ( vreset_any   ),  // sync HDMI au CPC, aligné hsync
    .v_start      ( v_start      ),  // row counter value at a full vreset (OSD 'V pos')
    .cy_arm       ( vp_arm       ),  // V pos / V lock: row counter load at the next row boundary
    .cy_val       ( vp_val       ),
    .frame_len    ( vtot         ),  // HDMI frame height: 624, or the CPC frame length with V lock Follow
    .cx_o         ( hcx          ),
    .cy_o         ( hcy          ),
    .load_en      ( rom_busy     ),  // ROM load progress bar: start-up and also loads while the CPC runs (was rom_rst = start-up only)
    .load_blk     ( rom_prog     ),
    .vmode        ( 2'd1         ),  // PAL 50 Hz hardcodé (évite bascule NTSC/PAL au boot)
    .screen       ( {1'b0, sys_w800} ),  // 0 = standard 720 px, 1 = 800 px active width (OSD 'Width')
    .fullscreen   ( sys_wide     ),  // AVI InfoFrame: 16:9 (full screen) instead of 4:3
    .r            ( vid_r        ),
    .g            ( vid_g        ),
    .b            ( vid_b        ),
    .audio        ( audio_zero   ),
    .tmds_clk_n   ( tmds_clk_n   ),
    .tmds_clk_p   ( tmds_clk_p   ),
    .tmds_d_n     ( tmds_d_n     ),
    .tmds_d_p     ( tmds_d_p     )
);

endmodule

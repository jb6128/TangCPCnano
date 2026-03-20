/*
    top.sv - TangCPC : Amstrad CPC 6128 sur Tang Nano 20K
    Phase 0 : Scaffold minimal — PLLs + pins GOWIN

    Cible : GOWIN GW2AR-18C (Tang Nano 20K)
    Horloge entrée : 27 MHz (pin 4)
    Génère : clk32 (32 MHz système), flash_clk (100 MHz SPI)
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
    input           bl616_mon_rx,

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
    input  [5:0]    io,
    input  [5:0]    spare,

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
//   pll_160m : 27 MHz → 160 MHz (HDMI TMDS)
//              via CLKDIV /5 → 32 MHz (clk32 système)
//   flash_pll: 27 MHz → 100 MHz (flash SPI)
// ---------------------------------------------------------------------------
wire clk160;
wire clk32;
wire flash_clk;
wire flash_clk_shifted;     // 100 MHz + 22.5° → mspi_clk (timing SPI correct)
wire pll_lock_hdmi;
wire pll_lock_flash;
wire pll_lock = pll_lock_hdmi && pll_lock_flash;
wire por      = !pll_lock;   // Power-On Reset : actif tant que PLLs non verrouillées

pll_160m pll_hdmi_inst (
    .clkout ( clk160        ),
    .lock   ( pll_lock_hdmi ),
    .clkin  ( clk           )
);

Gowin_CLKDIV clkdiv_inst (
    .clkout ( clk32  ),
    .hclkin ( clk160 ),
    .resetn ( pll_lock_hdmi )
);

flash_pll pll_flash_inst (
    .clkout  ( flash_clk         ),  // 100 MHz — logique flash
    .clkoutp ( flash_clk_shifted ),  // 100 MHz +22.5° — horloge SPI physique
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

reg spi_ext;
always @(posedge clk32) begin
    if (por)
        spi_ext <= 1'b0;
    else if (m0s_2 == 1'b0)
        spi_ext <= 1'b1;
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

// h_offset : décalage horizontal dans le line buffer scandoubler
// Valeur calibrée visuellement via les boutons S1/S2 (voir ci-dessous)
localparam [9:0] H_OFFSET = 10'd320;
wire [9:0] h_offset_reg = H_OFFSET;

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

assign leds_n = {
    ~core_dbg_r0_ok,            // [5]
    ~core_dbg_r4_ok,            // [4]
    ~core_dbg_r7_ok,            // [3]
    ~core_dbg_r9_ok,            // [2]
    ~vsync_cnt[4],              // [1] ~1.6 Hz si 50 Hz vsync
    ~core_readback_ok           // [0] ROM OK
};
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
// core_boot_dout et core_boot_ack déclarés ici pour éviter usage avant déclaration
wire  [7:0] core_boot_dout;
wire        core_boot_ack;
wire [22:0] boot_addr_w = core_boot_addr;
wire  [7:0] boot_din_w  = core_boot_din;
wire  [7:0] boot_dout_w;
wire        boot_req_w  = core_boot_req;
wire        boot_we_w   = core_boot_we;
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
// Flash SPI — horloge physique = flash_clk_shifted (22.5° de décalage)
// Le module flash gère mspi_cs, mspi_di, mspi_hold, mspi_wp, mspi_do
// ---------------------------------------------------------------------------
assign mspi_clk = flash_clk_shifted;

// ---------------------------------------------------------------------------
// cpc_core — Boot loader ROM + Amstrad_motherboard (Phase 3b)
// ---------------------------------------------------------------------------
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
wire [1:0]  core_mode;
wire        core_readback_ok;
wire        core_readback_done;
wire        core_wr_rd_ok;
wire        core_wr_rd_done;
wire [7:0]  core_rb_byte;
wire [15:0] core_dbg_first_flash;

// Sorties mémoire Amstrad_motherboard (Phase 3c : connectées à SDRAM)
wire [22:0] core_mem_addr;
wire        core_mem_rd;
wire        core_mem_wr;
wire        core_romen;
wire [14:0] core_vram_addr;

// Bus CPU Amstrad_motherboard (Phase 3c)
wire [15:0] core_cpu_addr;
wire  [7:0] core_cpu_dout;

cpc_core core_inst (
    .clk32      ( clk32              ),
    .flash_clk  ( flash_clk         ),
    .por        ( por                ),
    .boot_busy       ( boot_busy_w           ),
    .sdram_rdy       ( sdram_ready           ),
    .dbg_boot_state  ( core_dbg_boot_state   ),
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
    // VRAM data (Phase 3c)
    .vram_din_i ( core_vram_din     ),  // données 16-bit SDRAM → CRTC
    // Diagnostic readback
    .readback_ok   ( core_readback_ok   ),
    .readback_done ( core_readback_done ),
    .wr_rd_ok      ( core_wr_rd_ok      ),
    .wr_rd_done    ( core_wr_rd_done    ),
    .rb_byte       ( core_rb_byte       ),
    .dbg_first_flash_word ( core_dbg_first_flash )
);

// SPI MCU → sortie inactive
assign spi_io_dout   = 1'b1;
assign spi_intn      = 1'b1;

// SD card désactivé
assign sd_clk        = 1'b0;
assign sd_cmd        = 1'bz;
assign sd_dat        = 4'bz;

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
    .mix        ( 3'd0          ),    // 0 = couleurs GA40010

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
    .pixel_ena  (               ),    // non utilisé
    .r_out      ( sd_r          ),
    .g_out      ( sd_g          ),
    .b_out      ( sd_b          ),
    .hs_out     ( sd_hs         ),
    .vs_out     ( sd_vs         )
);

// -- Étage 3 : video2hdmi — TMDS --
// audio = zéro en Phase 3d (Phase 7 : YM2149)
wire [15:0] audio_zero [1:0];
assign audio_zero[0] = 16'h0;
assign audio_zero[1] = 16'h0;

// Vidéo CPC vers HDMI
wire [5:0] vid_r = sd_r;
wire [5:0] vid_g = sd_g;
wire [5:0] vid_b = sd_b;

// vreset : one-shot au boot uniquement (sync verticale HDMI → CPC)
// L'alignement horizontal est géré par h_offset dans le scandoubler
reg sd_vs_d;
reg vreset_pending;
reg vreset_done;
reg vreset_pulse;
always @(posedge clk32) begin
    if (por) begin
        sd_vs_d         <= 1'b0;
        vreset_pending  <= 1'b0;
        vreset_done     <= 1'b0;
        vreset_pulse    <= 1'b0;
    end else begin
        sd_vs_d <= sd_vs;
        vreset_pulse <= 1'b0;
        // front descendant vsync → tirer vreset (une seule fois)
        if (!vreset_done && sd_vs_d && !sd_vs) begin
            vreset_pulse <= 1'b1;
            vreset_done  <= 1'b1;
        end
    end
end

video2hdmi video2hdmi_inst (
    .clk_pixel_x5 ( clk160       ),  // 160 MHz TMDS
    .clk_pixel    ( clk32        ),  // 32 MHz pixel
    .vreset       ( vreset_pulse ),  // sync HDMI au CPC, aligné hsync
    .vmode        ( 2'd1         ),  // PAL 50 Hz hardcodé (évite bascule NTSC/PAL au boot)
    .screen       ( 2'd0         ),  // standard (pas d'overscan)
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

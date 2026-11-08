/*
    cpc_core.sv — TangCPC : Noyau CPC sur Tang Nano 20K
    Phase 3b : Instanciation Amstrad_motherboard (signaux neutres, always reset)

    Rôle :
      - Lit les ROMs CPC depuis la flash SPI au démarrage
      - Les écrit dans la SDRAM via le port Boot
      - Libère le reset CPC quand boot_done
      - (Phase 3b) Amstrad_motherboard instancié, reset=1, sorties exposées

    Layout flash (word addresses 16-bit) :
      0x100000 : OS ROM    CPC 6128 (16 KB = 0x2000 mots)
      0x102000 : BASIC ROM CPC 6128 (16 KB)
      0x104000 : AMSDOS ROM        (16 KB)

    Layout SDRAM (adresses byte 23-bit, bank encodé dans [22:21]) :
      0x000000..0x003FFF : ROM OS    (lower ROM, SDRAM bank 0)
      0x400000..0x403FFF : ROM BASIC (upper ROM bank 0, SDRAM bank 2)
      0x41C000..0x41FFFF : ROM AMSDOS(upper ROM bank 7, SDRAM bank 2)

    Domaines d'horloge :
      clk32     : logique principale, SDRAM
      flash_clk : module flash SPI (100 MHz)
      CDC       : 2-FF synchroniseurs pour busy/ready (flash_clk → clk32)
*/

module cpc_core #(
    parameter FLASH_READ_DELAY = 0   // sampling point of the flash data, see cpc_flash.v (0 = upstream default, 1 = one SPI clock later)
) (
    // Horloges
    input           clk32,          // 32 MHz système
    input           flash_clk,      // 100 MHz flash SPI
    input           por,            // Power-on reset (PLL non verrouillée)
    input           cpu_hold,       // freeze the Z80 (WAIT) while a ROM is written to SDRAM
    input           cold_boot,      // OSD cold boot: reload the ROMs from flash like at power-up

    // Sortie reset CPC
    output          boot_busy,      // 1 = ROMs pas encore chargées → CPU en reset

    // SDRAM prête (boot loader attend que SDRAM init soit terminée)
    input           sdram_rdy,

    // Clavier / joysticks depuis le BL616 (mcu_hid -> hid_cpc_matrix)
    //   ext_key[8*row+bit] = 1 : touche pressée (10 lignes CPC)
    //   joy1_in/joy2_in    : format hid.sv, actif HAUT (0 = repos)
    //                        [0]=Right [1]=Left [2]=Down [3]=Up [4]=Fire2 [5]=Fire1 [6]=Fire3
    input  [79:0]   ext_key,
    input   [6:0]   joy1_in,
    input   [6:0]   joy2_in,

    // Reset demandé par l'OSD (sysctrl "R"): 0 = run, 1 = reset, 3 = cold boot
    input   [1:0]   sys_reset,
    // OSD "Model": 0 = CPC 6128, 1 = CPC 664 (664 OS + BASIC 1.1, 64 KB RAM), 2 = CPC 464 (464 OS + BASIC 1.0, 64 KB RAM).
    // Sampled while the CPC is in reset (3 is treated as 6128).
    input     [1:0] model,

    // Cassette : signal lu par le PPI (port B bit 7) et relais moteur (port C bit 4)
    input           tape_in,
    output          tape_motor,
    output          tape_out,       // PPI port C bit 5: cassette write data

    // Audio YM2149 (8 bits non signés)
    output    [7:0] audio_l,
    output    [7:0] audio_r,

    // Contrôleur de disquette (u765) <-> couche SD
    input   [1:0]   fdc_img_mounted,
    input  [31:0]   fdc_img_size,
    input   [1:0]   fdc_wprot,
    input           fdc_fast,
    output [31:0]   fdc_sd_lba,
    output  [1:0]   fdc_sd_rd,
    output  [1:0]   fdc_sd_wr,
    input           fdc_sd_ack,
    input   [8:0]   fdc_buff_addr,
    input   [7:0]   fdc_buff_dout,
    input           fdc_buff_wr,
    output  [7:0]   fdc_buff_din,

    // Debug : état maximal atteint par le boot loader FSM
    output reg [4:0] dbg_boot_state,

    // -----------------------------------------------------------------------
    // Interface flash SPI (vers top.sv → pins MSPI)
    // -----------------------------------------------------------------------
    output          mspi_cs,
    inout           mspi_di,
    inout           mspi_hold,
    inout           mspi_wp,
    inout           mspi_do,

    // -----------------------------------------------------------------------
    // Port Boot SDRAM (vers cpc_sdram)
    // -----------------------------------------------------------------------
    output reg [22:0] boot_addr,
    output reg  [7:0] boot_din,
    input        [7:0] boot_dout,   // non utilisé en Phase 2/3b
    output reg         boot_req,
    output             boot_we,
    input              boot_ack,

    // -----------------------------------------------------------------------
    // ROM map vers Amstrad_motherboard
    // rom_map[N] = 1 si ROM extension N est présente en SDRAM
    // -----------------------------------------------------------------------
    input  [255:0]  rom_map,
    input  [1:0]    rom_ext,        // {7, 0}: external upper ROM 7 / 0 in use (OSD ROM7/ROM0 = EXT and a ROM loaded)

    // -----------------------------------------------------------------------
    // Sorties vidéo Amstrad_motherboard (Phase 3b — exposées, Phase 3d actives)
    // -----------------------------------------------------------------------
    output [1:0]    red,
    output [1:0]    green,
    output [1:0]    blue,
    output          hsync,
    output          vsync,
    output          hblank,
    output          vblank,
    output          field,
    output [1:0]    mode,
    output          crtc_vs_raw,  // VSYNC brut CRTC pour diagnostic
    output          dbg_r0_ok,    // CRTC R0 == 63
    output          dbg_r4_ok,    // CRTC R4 == 38
    output          dbg_r7_ok,    // CRTC R7 == 30
    output          dbg_r9_ok,    // CRTC R9 == 7
    output          ga_ready_o,   // GA40010 READY (diagnostic)
    output reg [15:0] dbg_pc,     // address of the last Z80 opcode fetch (diagnostic)

    // -----------------------------------------------------------------------
    // Sorties mémoire Amstrad_motherboard (Phase 3b — exposées, Phase 3c actives)
    // -----------------------------------------------------------------------
    output [22:0]   mem_addr,
    output          mem_rd,
    output          mem_wr,
    output          romen,

    // VRAM : adresse pour port SDRAM (Phase 3c)
    output [14:0]   vram_addr,

    // CPU bus (Phase 3c)
    output [15:0]   cpu_addr,
    output  [7:0]   cpu_dout,
    input   [7:0]   cpu_din,        // données SDRAM → CPU (Phase 3c)
    input           cpu_ack_in,     // SDRAM CPU read/write ack (pour gating READY)
    input           cpu_soon_in,    // SDRAM read data ready within <= 4 cycles
    input           wait_gate_en,   // OSD: 1 = Z80 waits for SDRAM data, 0 = never (test only)

    // VRAM data : données 16 bits SDRAM → Amstrad_motherboard (Phase 3c)
    input  [15:0]   vram_din_i,

    // -----------------------------------------------------------------------
    // Diagnostic : readback SDRAM après boot
    // -----------------------------------------------------------------------
    output reg      readback_ok,    // 1 si test ROM readback == 0x01
    output reg      readback_done,  // 1 quand le test ROM est terminé
    output reg      wr_rd_ok,       // 1 si test write/read 0xAB réussi
    output reg      wr_rd_done,     // 1 quand le test write/read est terminé
    output reg [7:0] rb_byte,       // valeur brute lue lors du readback ROM
    output reg [15:0] dbg_first_flash_word, // premier mot flash lu (doit être 0x0189)
    output reg      flash_retry,    // sticky: a flash word differed between two reads (re-read at least once)
    output reg      flash_err,      // sticky: a flash word never gave two equal reads (gave up)
    output reg [15:0] flash_retry_cnt // number of flash words whose two reads differed during the boot copy (saturating)
);

// ---------------------------------------------------------------------------
// Constantes de la table des segments ROM
// ---------------------------------------------------------------------------
// 3 segments, chacun 0x2000 mots (16 KB)
localparam        N_SEGS     = 7;   // 6128 OS, BASIC, AMSDOS + 664 OS, BASIC + 464 OS, BASIC
localparam [12:0] LAST_WORD  = 13'd8191; // index du dernier mot (16KB = 8192 mots, indices 0..8191)

// Flash word addresses de début de chaque segment
localparam [21:0] FLASH_BASE_0 = 22'h100000; // OS ROM    CPC 6128
localparam [21:0] FLASH_BASE_1 = 22'h102000; // BASIC ROM CPC 6128
localparam [21:0] FLASH_BASE_2 = 22'h104000; // AMSDOS ROM
localparam [21:0] FLASH_BASE_3 = 22'h108000; // OS ROM    CPC 664  (boot.rom offset 0x10000)
localparam [21:0] FLASH_BASE_4 = 22'h10A000; // BASIC ROM CPC 664  (boot.rom offset 0x14000)
localparam [21:0] FLASH_BASE_5 = 22'h10C000; // OS ROM    CPC 464  (boot.rom offset 0x18000)
localparam [21:0] FLASH_BASE_6 = 22'h10E000; // BASIC ROM CPC 464 (BASIC 1.0, boot.rom offset 0x1C000)

// SDRAM byte addresses de début (bank encodé dans [22:21])
localparam [22:0] SDRAM_BASE_0 = 23'h000000; // lower ROM OS
localparam [22:0] SDRAM_BASE_1 = 23'h400000; // upper ROM bank 0 (BASIC)
localparam [22:0] SDRAM_BASE_2 = 23'h41C000; // upper ROM bank 7 (AMSDOS)
// 664 set in the free RAM pages 0/1 (0x000000..0x01FFFF; CPC RAM starts at 0x020000, see Amstrad_MMU)
localparam [22:0] SDRAM_BASE_3 = 23'h004000; // 664 lower ROM (OS)
localparam [22:0] SDRAM_BASE_4 = 23'h008000; // 664 upper ROM 0 (BASIC)
localparam [22:0] SDRAM_BASE_5 = 23'h00C000; // 464 lower ROM (OS)
localparam [22:0] SDRAM_BASE_6 = 23'h010000; // 464 upper ROM 0 (BASIC 1.0)

// ROM map : fournie par cpc_romload (slots 0 et 7 = flash, 1..31 = ROMs OSD)

// Boot en écriture pendant le chargement, en lecture pendant le readback
reg boot_we_r;
assign boot_we = boot_we_r;

// ---------------------------------------------------------------------------
// Module Flash SPI (cpc_flash)
// Tourne à flash_clk (100 MHz)
// ---------------------------------------------------------------------------
wire         flash_ready;
wire         flash_busy;
wire [15:0]  flash_dout;
// por (clk32 domain) re-synchronised to the flash clock
reg [1:0] por_fs = 2'b11;
always @(posedge flash_clk) por_fs <= {por_fs[0], por};
wire por_f = por_fs[1];

reg  [21:0]  flash_addr;
reg          flash_cs;

// Flash read experiment: sampling point of the flash data (FLASH_READ_DELAY: 0 = upstream default,
// 1 = one SPI clock later = this core up to 12df7b9); the phase of the SPI pin clock is the PSDA of flash_pll.
// Both are set in top.sv (FLASH_PSDA / FLASH_READ_DELAY).

flash #(.READ_DELAY(FLASH_READ_DELAY)) flash_inst (
    .clk        ( flash_clk   ),
    .resetn     ( ~por_f      ),
    .ready      ( flash_ready ),
    .address    ( flash_addr  ),
    .cs         ( flash_cs    ),
    .dout       ( flash_dout  ),
    .mspi_cs    ( mspi_cs     ),
    .mspi_di    ( mspi_di     ),
    .mspi_hold  ( mspi_hold   ),
    .mspi_wp    ( mspi_wp     ),
    .mspi_do    ( mspi_do     ),
    .busy       ( flash_busy  )
);

// ---------------------------------------------------------------------------
// CDC : synchronisation flash_busy et flash_ready → clk32
// (2-FF synchroniseurs)
// ---------------------------------------------------------------------------
reg [1:0] busy_sync;
reg [1:0] ready_sync;

always @(posedge clk32) begin
    busy_sync  <= { busy_sync[0],  flash_busy  };
    ready_sync <= { ready_sync[0], flash_ready };
end

wire busy_s  = busy_sync[1];
wire ready_s = ready_sync[1];

// ---------------------------------------------------------------------------
// Machine d'état du boot loader (domaine clk32)
// ---------------------------------------------------------------------------
localparam ST_IDLE      = 4'd0;  // attente PLL + flash ready
localparam ST_SEG_INIT  = 4'd1;  // init adresses du segment courant
localparam ST_FLASH_CS  = 4'd2;  // impulsion cs (1 cycle clk32)
localparam ST_WAIT_HI   = 4'd3;  // attente busy=1 (flash a démarré)
localparam ST_WAIT_LO   = 4'd4;  // attente busy=0 (lecture terminée)
localparam ST_LATCH     = 4'd5;  // capturer flash_dout avant d'écrire
localparam ST_WR_LO     = 4'd6;  // écriture octet bas → SDRAM
localparam ST_WR_LO_ACK = 4'd7;  // attente ack octet bas
localparam ST_WR_HI     = 4'd8;  // écriture octet haut → SDRAM
localparam ST_WR_HI_ACK = 4'd9;  // attente ack octet haut
localparam ST_NEXT_WORD = 4'd10; // avancer d'un mot (2 octets)
localparam ST_NEXT_SEG  = 4'd11; // segment suivant ou fin
localparam ST_DONE      = 5'd12; // boot terminé
localparam ST_RB_REQ    = 5'd13; // readback ROM : lancer lecture SDRAM addr 0
localparam ST_RB_WAIT   = 5'd14; // readback ROM : attendre boot_ack
localparam ST_RB_CHECK  = 5'd15; // readback ROM : comparer et terminer
localparam ST_WR_TEST   = 5'd16; // test W/R : écrire 0xAB à adresse 0x7FF000
localparam ST_WR_T_ACK  = 5'd17; // test W/R : attendre ack écriture
localparam ST_RD_TEST   = 5'd18; // test W/R : relire la même adresse
localparam ST_RD_T_ACK  = 5'd19; // test W/R : attendre ack lecture
localparam ST_RD_T_CHK  = 5'd20; // test W/R : comparer
localparam ST_V_CS      = 5'd21; // verify: read the same flash word again (cs pulse)
localparam ST_V_HI      = 5'd22; // verify: wait busy=1
localparam ST_V_LO      = 5'd23; // verify: wait busy=0
localparam ST_V_CMP     = 5'd24; // verify: two equal reads in a row -> accept, else read again

reg [4:0]  state;
reg [2:0]  seg;          // 0..6 : index du segment ROM en cours
reg [12:0] word_cnt;     // compteur de mots dans le segment courant
reg [21:0] cur_flash;    // adresse flash courante (mots)
reg [22:0] cur_sdram;    // adresse SDRAM courante (bytes)
reg [15:0] word_latch;   // mot flash capturé

reg [3:0] v_cnt;          // re-reads of the current flash word
reg boot_done_r;
assign boot_busy = ~boot_done_r;

reg first_latch_captured;  // 1 après capture du premier flash_dout

// Model selection takes effect at a reset only (the ROMs must not change under a running CPU)
reg [1:0] model_r = 2'd0;
always @(posedge clk32) if (~boot_done_r | (|sys_reset)) model_r <= model;

// Likewise the choice internal / external ROM 0 and 7 (the upper ROM must not change under a running CPU)
reg [1:0] rom_ext_r = 2'b00;
always @(posedge clk32) if (~boot_done_r | (|sys_reset)) rom_ext_r <= rom_ext;

wire boot_rst = por | cold_boot;
always @(posedge clk32) begin
    flash_cs <= 1'b0;   // défaut
    boot_req <= 1'b0;   // défaut (impulsion 1 cycle)

    if (boot_rst) begin
        state           <= ST_IDLE;
        boot_done_r     <= 1'b0;
        seg             <= 3'd0;
        word_cnt        <= 13'd0;
        dbg_boot_state  <= 5'd0;
        boot_we_r       <= 1'b1;
        readback_ok     <= 1'b0;
        readback_done   <= 1'b0;
        wr_rd_ok        <= 1'b0;
        wr_rd_done      <= 1'b0;
        rb_byte         <= 8'h00;
        dbg_first_flash_word <= 16'h0000;
        first_latch_captured <= 1'b0;
        flash_retry     <= 1'b0;
        flash_err       <= 1'b0;
        flash_retry_cnt <= 16'd0;
        v_cnt           <= 4'd0;
    end else begin
        // Latch état maximum atteint par la FSM (debug)
        if (state > dbg_boot_state)
            dbg_boot_state <= state;
        case (state)

            // ----------------------------------------------------------------
            // ST_IDLE : attendre que PLL et flash soient prêts
            // ----------------------------------------------------------------
            ST_IDLE: begin
                if (ready_s && sdram_rdy)
                    state <= ST_SEG_INIT;
            end

            // ----------------------------------------------------------------
            // ST_SEG_INIT : charger les adresses de début du segment
            // ----------------------------------------------------------------
            ST_SEG_INIT: begin
                word_cnt <= 13'd0;
                case (seg)
                    3'd0: begin cur_flash <= FLASH_BASE_0; cur_sdram <= SDRAM_BASE_0; end
                    3'd1: begin cur_flash <= FLASH_BASE_1; cur_sdram <= SDRAM_BASE_1; end
                    3'd2: begin cur_flash <= FLASH_BASE_2; cur_sdram <= SDRAM_BASE_2; end
                    3'd3: begin cur_flash <= FLASH_BASE_3; cur_sdram <= SDRAM_BASE_3; end
                    3'd4: begin cur_flash <= FLASH_BASE_4; cur_sdram <= SDRAM_BASE_4; end
                    3'd5: begin cur_flash <= FLASH_BASE_5; cur_sdram <= SDRAM_BASE_5; end
                    3'd6: begin cur_flash <= FLASH_BASE_6; cur_sdram <= SDRAM_BASE_6; end
                    default: ;
                endcase
                state <= ST_FLASH_CS;
            end

            // ----------------------------------------------------------------
            // ST_FLASH_CS : impulsion cs → déclenche lecture flash
            // ----------------------------------------------------------------
            ST_FLASH_CS: begin
                flash_addr <= cur_flash;
                flash_cs   <= 1'b1;   // impulsion 1 cycle clk32
                state      <= ST_WAIT_HI;
            end

            // ----------------------------------------------------------------
            // ST_WAIT_HI : attendre que flash_busy passe à 1
            // ----------------------------------------------------------------
            ST_WAIT_HI: begin
                if (busy_s)
                    state <= ST_WAIT_LO;
            end

            // ----------------------------------------------------------------
            // ST_WAIT_LO : attendre que flash_busy repasse à 0
            // ----------------------------------------------------------------
            ST_WAIT_LO: begin
                if (!busy_s)
                    state <= ST_LATCH;
            end

            // ----------------------------------------------------------------
            // ST_LATCH : capturer flash_dout
            // ----------------------------------------------------------------
            ST_LATCH: begin
                word_latch <= flash_dout;
                // Capturer le tout premier mot flash (diagnostic)
                if (!first_latch_captured) begin
                    dbg_first_flash_word <= flash_dout;
                    first_latch_captured <= 1'b1;
                end
                v_cnt      <= 4'd0;
                state      <= ST_V_CS;   // every word is read until two reads in a row agree
            end

            // ST_V_CS / ST_V_HI / ST_V_LO / ST_V_CMP: the flash data is sampled straight from the pin; with a
            // marginal SPI timing single reads can be wrong. A word is accepted when two consecutive reads
            // are equal (at most 15 re-reads, then the last value is used and flash_err is set).
            ST_V_CS: begin
                flash_addr <= cur_flash;
                flash_cs   <= 1'b1;
                state      <= ST_V_HI;
            end
            ST_V_HI: if (busy_s) state <= ST_V_LO;
            ST_V_LO: if (!busy_s) state <= ST_V_CMP;
            ST_V_CMP: begin
                if (flash_dout == word_latch) begin
                    state <= ST_WR_LO;
                end else begin
                    flash_retry <= 1'b1;
                    if (flash_retry_cnt != 16'hFFFF) flash_retry_cnt <= flash_retry_cnt + 16'd1;
                    word_latch  <= flash_dout;
                    if (v_cnt == 4'd15) begin
                        flash_err <= 1'b1;
                        state     <= ST_WR_LO;
                    end else begin
                        v_cnt <= v_cnt + 4'd1;
                        state <= ST_V_CS;
                    end
                end
            end

            // ----------------------------------------------------------------
            // ST_WR_LO : écriture du premier byte ROM en SDRAM
            // dout[15:8] = premier byte lu depuis le flash (adresse 2N)
            // → doit aller à cur_sdram+0 (adresse basse)
            // ----------------------------------------------------------------
            ST_WR_LO: begin
                boot_addr <= cur_sdram;
                boot_din  <= word_latch[15:8];  // premier byte flash → adresse basse
                boot_req  <= 1'b1;
                state     <= ST_WR_LO_ACK;
            end

            // ----------------------------------------------------------------
            // ST_WR_LO_ACK : attente du ack SDRAM pour l'octet bas
            // ----------------------------------------------------------------
            ST_WR_LO_ACK: begin
                if (boot_ack)
                    state <= ST_WR_HI;
            end

            // ----------------------------------------------------------------
            // ST_WR_HI : écriture de l'octet haut en SDRAM (adresse + 1)
            // ----------------------------------------------------------------
            ST_WR_HI: begin
                boot_addr <= cur_sdram + 23'd1;
                boot_din  <= word_latch[7:0];   // deuxième byte flash → adresse haute
                boot_req  <= 1'b1;
                state     <= ST_WR_HI_ACK;
            end

            // ----------------------------------------------------------------
            // ST_WR_HI_ACK : attente du ack SDRAM pour l'octet haut
            // ----------------------------------------------------------------
            ST_WR_HI_ACK: begin
                if (boot_ack)
                    state <= ST_NEXT_WORD;
            end

            // ----------------------------------------------------------------
            // ST_NEXT_WORD : avancer d'un mot (2 octets)
            // ----------------------------------------------------------------
            ST_NEXT_WORD: begin
                cur_flash <= cur_flash + 22'd1;
                cur_sdram <= cur_sdram + 23'd2;
                word_cnt  <= word_cnt  + 13'd1;

                if (word_cnt == LAST_WORD)
                    state <= ST_NEXT_SEG;
                else
                    state <= ST_FLASH_CS;
            end

            // ----------------------------------------------------------------
            // ST_NEXT_SEG : passer au segment suivant, ou boot terminé
            // ----------------------------------------------------------------
            ST_NEXT_SEG: begin
                if (seg == N_SEGS - 1)
                    state <= ST_DONE;
                else begin
                    seg   <= seg + 3'd1;
                    state <= ST_SEG_INIT;
                end
            end

            // ----------------------------------------------------------------
            // ST_DONE : tous les segments chargés — libérer le CPC
            // ----------------------------------------------------------------
            ST_DONE: begin
                boot_done_r <= 1'b1;
                state       <= ST_RB_REQ;
            end

            // ----------------------------------------------------------------
            // ST_RB_REQ : lire l'octet 0 de l'OS ROM depuis SDRAM
            // ----------------------------------------------------------------
            ST_RB_REQ: begin
                boot_addr  <= 23'h000000;  // adresse OS ROM byte 0
                boot_we_r  <= 1'b0;        // mode lecture
                boot_req   <= 1'b1;
                state      <= ST_RB_WAIT;
            end

            // ----------------------------------------------------------------
            // ST_RB_WAIT : attendre boot_ack du SDRAM
            // ----------------------------------------------------------------
            ST_RB_WAIT: begin
                if (boot_ack)
                    state <= ST_RB_CHECK;
            end

            // ----------------------------------------------------------------
            // ST_RB_CHECK : comparer l'octet lu avec la valeur attendue
            // Premier octet de l'OS ROM CPC 6128 = 0x01
            // ----------------------------------------------------------------
            ST_RB_CHECK: begin
                rb_byte       <= boot_dout;
                readback_ok   <= (boot_dout == 8'h01);
                readback_done <= 1'b1;
                state         <= ST_WR_TEST;
            end

            // ================================================================
            // Test Write/Read : écrire 0xAB à 0x7FF000, relire, comparer
            // (adresse haute dans bank 3, loin des ROMs et RAM CPC)
            // ================================================================

            // ----------------------------------------------------------------
            // ST_WR_TEST : écrire 0xAB
            // ----------------------------------------------------------------
            ST_WR_TEST: begin
                boot_addr  <= 23'h7FF000;  // zone libre : ne pas écraser l'OS ROM
                boot_din   <= 8'hAB;
                boot_we_r  <= 1'b1;
                boot_req   <= 1'b1;
                state      <= ST_WR_T_ACK;
            end

            ST_WR_T_ACK: begin
                if (boot_ack)
                    state <= ST_RD_TEST;
            end

            // ----------------------------------------------------------------
            // ST_RD_TEST : relire la même adresse
            // ----------------------------------------------------------------
            ST_RD_TEST: begin
                boot_addr  <= 23'h7FF000;
                boot_we_r  <= 1'b0;       // mode lecture
                boot_req   <= 1'b1;
                state      <= ST_RD_T_ACK;
            end

            ST_RD_T_ACK: begin
                if (boot_ack)
                    state <= ST_RD_T_CHK;
            end

            // ----------------------------------------------------------------
            // ST_RD_T_CHK : comparer
            // ----------------------------------------------------------------
            ST_RD_T_CHK: begin
                wr_rd_ok    <= (boot_dout == 8'hAB);
                wr_rd_done  <= 1'b1;
                boot_we_r   <= 1'b1;
                // terminal
            end

            default: state <= ST_IDLE;
        endcase
    end
end

// ---------------------------------------------------------------------------
// Génération ce_16 : enable 16 MHz depuis clk32 (toggle chaque cycle)
// Le Gate Array CPC tourne à 16 MHz (div-by-2 de clk32)
// ---------------------------------------------------------------------------
reg ce_16_r;
always @(posedge clk32) ce_16_r <= ~ce_16_r;

// ---------------------------------------------------------------------------
// Signal sdram_data_ready : indique que cpu_dout SDRAM est valide
// Passe à 0 au front montant de mem_rd, repasse à 1 quand cpu_ack arrive.
// cpu_ack a priorité (si ack et nouveau read simultanés, data est prête).
// ---------------------------------------------------------------------------
reg sdram_data_ready;
reg mem_rd_prev;
always @(posedge clk32) begin
    if (por) begin
        sdram_data_ready <= 1'b1;
        mem_rd_prev      <= 1'b0;
    end else begin
        mem_rd_prev <= mem_rd;
        if (cpu_ack_in)
            sdram_data_ready <= 1'b1;   // données prêtes
        else if (mem_rd && !mem_rd_prev)
            sdram_data_ready <= 1'b0;   // nouvelle lecture → invalide
    end
end

// ---------------------------------------------------------------------------
// Phase 3b : Amstrad_motherboard — signaux neutres, toujours en reset
// Phase 3c : connecter SDRAM (cpu_din réel, vram_din réel, reset = ~boot_busy)
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// Bus de données du Z80 pendant les cycles I/O
//
// cpu_din est la sortie du port CPU de la SDRAM : un registre qui garde la
// dernière lecture mémoire. Le motherboard fait di = crtc_dout & ppi_dout &
// cpu_din ; pendant un cycle I/O cette valeur périmée polluerait donc la lecture
// (clavier via PPI/PSG, FDC). Pendant IORQ on remplace cpu_din par le bus
// d'extension : 0xFF (bus flottant) ou la sortie du FDC.
// ---------------------------------------------------------------------------
wire        mb_iorq, mb_rd, mb_wr, mb_m1;
// last opcode fetch address (shown on the LEDs with S1+S2, to see where a hung Z80 sits)
always @(posedge clk32) if (mb_m1 & mem_rd) dbg_pc <= cpu_addr;
wire  [7:0] fdc_dout;
wire  [7:0] cpu_din_bus = mb_iorq ? fdc_dout : cpu_din;

cpc_fdc fdc_inst (
    .clk          ( clk32           ),
    .reset        ( ~boot_done_r | (|sys_reset) ),
    .iorq         ( mb_iorq         ),
    .rd           ( mb_rd           ),
    .wr           ( mb_wr           ),
    .addr         ( cpu_addr        ),
    .din          ( cpu_dout        ),
    .dout         ( fdc_dout        ),
    .wprot        ( fdc_wprot       ),
    .fast         ( fdc_fast        ),
    .img_mounted  ( fdc_img_mounted ),
    .img_size     ( fdc_img_size    ),
    .sd_lba       ( fdc_sd_lba      ),
    .sd_rd        ( fdc_sd_rd       ),
    .sd_wr        ( fdc_sd_wr       ),
    .sd_ack       ( fdc_sd_ack      ),
    .sd_buff_addr ( fdc_buff_addr   ),
    .sd_buff_dout ( fdc_buff_dout   ),
    .sd_buff_wr   ( fdc_buff_wr     ),
    .sd_buff_din  ( fdc_buff_din    ),
    .motor_o      (                 ),
    .ready_o      (                 )
);

Amstrad_motherboard mb (
    // reset : libéré après chargement ROMs (boot_done_r=1 → reset=0)
    .reset          ( ~boot_done_r | (|sys_reset) ),

    .clk            ( clk32         ),
    .ce_16          ( ce_16_r       ),

    // Joysticks / clavier — entrées neutres
    .joy1           ( joy1_in       ),  // actif HAUT (0 = repos), cf. hid.sv
    .joy2           ( joy2_in       ),
    .right_shift_mod( 1'b0          ),
    .keypad_mod     ( 1'b0          ),
    .ps2_key        ( 11'h0         ),
    .ps2_mouse      ( 25'h0         ),
    .ext_key        ( ext_key       ),

    // Configuration matérielle
    .ppi_jumpers    ( 4'hF          ),  // jumpers standards CPC 6128
    .crtc_type      ( 1'b0          ),  // CRTC type 0
    .sync_filter    ( 1'b1          ),  // crt_filter (robust vsync/hsync). Test 5a97f8d with 1'b0 (raw GA sync): CSYNC ok, HDMI unchanged, no RGB flicker gain -> kept at 1
    .no_wait        ( 1'b0          ),  // wait states GA40010 actifs
    .cpu_hold       ( cpu_hold      ),
    .sdram_data_ready( sdram_data_ready | cpu_soon_in | ~wait_gate_en ), // gating READY sur données SDRAM

    // Cassette
    .tape_in        ( tape_in       ),

    // VRAM — données depuis SDRAM port VRAM (Phase 3c)
    .vram_din       ( vram_din_i    ),

    // ROM map — depuis boot loader
    .rom_map        ( rom_map       ),
    .rom_ext        ( rom_ext_r     ),
    .ram64k         ( model_r == 2'd1 || model_r == 2'd2 ),  // 664 / 464: 64 KB (RAM bank switching ignored)
    .model          ( model_r       ),  // 664 / 464: OS/BASIC from the 664 / 464 set

    // Bus CPU — données bus tirées haut (tri-state CPC convention)
    .cpu_din        ( cpu_din_bus   ),

    // IRQ/NMI — inactifs
    .irq            ( 1'b0          ),
    .nmi            ( 1'b0          ),

    // Sorties vidéo → ports cpc_core
    .red            ( red           ),
    .green          ( green         ),
    .blue           ( blue          ),
    .hsync          ( hsync         ),
    .vsync          ( vsync         ),
    .hblank         ( hblank        ),
    .vblank         ( vblank        ),
    .field          ( field         ),
    .crtc_vs_raw    ( crtc_vs_raw   ),
    .dbg_r0_ok      ( dbg_r0_ok     ),
    .dbg_r4_ok      ( dbg_r4_ok     ),
    .dbg_r7_ok      ( dbg_r7_ok     ),
    .dbg_r9_ok      ( dbg_r9_ok     ),
    .mode           ( mode          ),

    // Sorties mémoire → ports cpc_core
    .mem_addr       ( mem_addr      ),
    .mem_rd         ( mem_rd        ),
    .mem_wr         ( mem_wr        ),
    .romen          ( romen         ),
    .vram_addr      ( vram_addr     ),

    // CPU bus
    .cpu_addr       ( cpu_addr      ),
    .cpu_dout       ( cpu_dout      ),

    // Sorties ignorées en Phase 3b
    .joy1_sel       (               ),
    .joy2_sel       (               ),
    .key_nmi        (               ),
    .key_reset      (               ),
    .Fn             (               ),
    .tape_out       ( tape_out      ),
    .tape_motor     ( tape_motor    ),
    .audio_l        ( audio_l       ),
    .audio_r        ( audio_r       ),
    .phi_n          (               ),
    .phi_en_n       (               ),
    .phi_en_p       (               ),
    .iorq           ( mb_iorq       ),
    .mreq           (               ),
    .rd             ( mb_rd         ),
    .wr             ( mb_wr         ),
    .m1             ( mb_m1         ),
    .ga_ready       ( ga_ready_o    ),
    .cursor         (               )
);

endmodule

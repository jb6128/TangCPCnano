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

module cpc_core (
    // Horloges
    input           clk32,          // 32 MHz système
    input           flash_clk,      // 100 MHz flash SPI
    input           por,            // Power-on reset (PLL non verrouillée)

    // Sortie reset CPC
    output          boot_busy,      // 1 = ROMs pas encore chargées → CPU en reset

    // SDRAM prête (boot loader attend que SDRAM init soit terminée)
    input           sdram_rdy,

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
    output [255:0]  rom_map,

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
    output reg [15:0] dbg_first_flash_word  // premier mot flash lu (doit être 0x0189)
);

// ---------------------------------------------------------------------------
// Constantes de la table des segments ROM
// ---------------------------------------------------------------------------
// 3 segments, chacun 0x2000 mots (16 KB)
localparam        N_SEGS     = 3;
localparam [12:0] LAST_WORD  = 13'd8191; // index du dernier mot (16KB = 8192 mots, indices 0..8191)

// Flash word addresses de début de chaque segment
localparam [21:0] FLASH_BASE_0 = 22'h100000; // OS ROM    CPC 6128
localparam [21:0] FLASH_BASE_1 = 22'h102000; // BASIC ROM CPC 6128
localparam [21:0] FLASH_BASE_2 = 22'h104000; // AMSDOS ROM

// SDRAM byte addresses de début (bank encodé dans [22:21])
localparam [22:0] SDRAM_BASE_0 = 23'h000000; // lower ROM OS
localparam [22:0] SDRAM_BASE_1 = 23'h400000; // upper ROM bank 0 (BASIC)
localparam [22:0] SDRAM_BASE_2 = 23'h41C000; // upper ROM bank 7 (AMSDOS)

// ---------------------------------------------------------------------------
// ROM map : AMSDOS présent en slot 7
// ---------------------------------------------------------------------------
assign rom_map = 256'h1 << 7; // bit 7 = AMSDOS

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
reg  [21:0]  flash_addr;
reg          flash_cs;

flash flash_inst (
    .clk        ( flash_clk   ),
    .resetn     ( ~por        ),
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

reg [4:0]  state;
reg [1:0]  seg;          // 0..2 : index du segment ROM en cours
reg [12:0] word_cnt;     // compteur de mots dans le segment courant
reg [21:0] cur_flash;    // adresse flash courante (mots)
reg [22:0] cur_sdram;    // adresse SDRAM courante (bytes)
reg [15:0] word_latch;   // mot flash capturé

reg boot_done_r;
assign boot_busy = ~boot_done_r;

reg first_latch_captured;  // 1 après capture du premier flash_dout

always @(posedge clk32) begin
    flash_cs <= 1'b0;   // défaut
    boot_req <= 1'b0;   // défaut (impulsion 1 cycle)

    if (por) begin
        state           <= ST_IDLE;
        boot_done_r     <= 1'b0;
        seg             <= 2'd0;
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
                    2'd0: begin cur_flash <= FLASH_BASE_0; cur_sdram <= SDRAM_BASE_0; end
                    2'd1: begin cur_flash <= FLASH_BASE_1; cur_sdram <= SDRAM_BASE_1; end
                    2'd2: begin cur_flash <= FLASH_BASE_2; cur_sdram <= SDRAM_BASE_2; end
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
                state      <= ST_WR_LO;
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
                    seg   <= seg + 2'd1;
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
                boot_addr  <= 23'h000000;  // même adresse que la ROM byte 0
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
                boot_addr  <= 23'h000000;
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
Amstrad_motherboard mb (
    // reset : libéré après chargement ROMs (boot_done_r=1 → reset=0)
    .reset          ( ~boot_done_r  ),

    .clk            ( clk32         ),
    .ce_16          ( ce_16_r       ),

    // Joysticks / clavier — entrées neutres
    .joy1           ( 7'h7F         ),  // aucun bouton pressé
    .joy2           ( 7'h7F         ),
    .right_shift_mod( 1'b0          ),
    .keypad_mod     ( 1'b0          ),
    .ps2_key        ( 11'h0         ),
    .ps2_mouse      ( 25'h0         ),

    // Configuration matérielle
    .ppi_jumpers    ( 4'hF          ),  // jumpers standards CPC 6128
    .crtc_type      ( 1'b0          ),  // CRTC type 0
    .sync_filter    ( 1'b1          ),  // utilise crt_filter (vsync/hsync robuste)
    .no_wait        ( 1'b0          ),  // wait states GA40010 actifs
    .sdram_data_ready( sdram_data_ready ), // gating READY sur données SDRAM

    // Cassette — inactive
    .tape_in        ( 1'b0          ),

    // VRAM — données depuis SDRAM port VRAM (Phase 3c)
    .vram_din       ( vram_din_i    ),

    // ROM map — depuis boot loader
    .rom_map        ( rom_map       ),
    .ram64k         ( 1'b0          ),  // mode 128 KB

    // Bus CPU — données bus tirées haut (tri-state CPC convention)
    .cpu_din        ( cpu_din       ),

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
    .tape_out       (               ),
    .tape_motor     (               ),
    .audio_l        (               ),
    .audio_r        (               ),
    .phi_n          (               ),
    .phi_en_n       (               ),
    .phi_en_p       (               ),
    .iorq           (               ),
    .mreq           (               ),
    .rd             (               ),
    .wr             (               ),
    .m1             (               ),
    .ga_ready       ( ga_ready_o    ),
    .cursor         (               )
);

endmodule

//
// cpc_sdram.v — Contrôleur SDRAM multi-port pour TangCPC
//
// Cible : GOWIN GW2AR-18C (Tang Nano 20K)
//   SDRAM interne : 32 bits, 11 bits de ligne, 9 bits de colonne, 4 banques
//   Horloge      : 32 MHz (clk32)
//   Timing       : CAS Latency = 2, tRCD = 1 cycle (31 ns @ 32 MHz)
//
// 3 ports :
//   Port 0 (CPU)  : 8 bits R/W, adresse 23 bits, handshake req/ack
//   Port 1 (VRAM) : 16 bits lecture seule, adresse 22 bits, rafraîchi
//                   automatiquement dès que l'adresse change
//   Port 2 (Boot) : 8 bits R/W (chargement ROMs depuis flash → SDRAM)
//                   handshake req/ack
//
// Arbitrage : Boot > VRAM > CPU (priorité fixe)
//   Le port Boot est actif uniquement pendant la phase de démarrage.
//   En fonctionnement normal, VRAM > CPU.
//   Un rafraîchissement auto-refresh est inséré quand aucun port n'est actif.
//
// Layout SDRAM :
//   Banque 0 : RAM CPC 6128  (512 KB) + ROMs OS/BASIC/AMSDOS (3×16 KB)
//   Banque 1 : RAM CPC 664   (128 KB) + ROMs
//   Banque 2 : Buffer tape / Dandanator EEPROM
//   Banque 3 : ROMs extension (256 pages × 16 KB)
//
// Adressage physique (23 bits → banque 2 + ligne 11 + colonne 9 + byte) :
//   addr[22:21] = banque SDRAM
//   addr[20:10] = ligne  (11 bits)
//   addr[ 9: 1] = colonne (9 bits, bit 0 sélectionne octet haut/bas)
//   addr[    0] = sélection octet (0=bas/D[7:0], 1=haut/D[15:8])
//
// Copyright (c) 2025 — TangCPC project
// Basé sur sdram.v de MiSTeryNano (Till Harbaum) et sdram.v MiSTer (Sorgelig)
// SPDX-License-Identifier: GPL-3.0-or-later
//

module cpc_sdram (

    // -----------------------------------------------------------------------
    // Interface SDRAM physique (GW2AR-18 « magic ports »)
    // -----------------------------------------------------------------------
    output              O_sdram_clk,
    output              O_sdram_cke,
    output              O_sdram_cs_n,
    output              O_sdram_ras_n,
    output              O_sdram_cas_n,
    output              O_sdram_wen_n,
    output       [3:0]  O_sdram_dqm,
    output reg  [10:0]  O_sdram_addr,   // GW2AR-18 : 11 bits de ligne
    output reg   [1:0]  O_sdram_ba,
    inout        [31:0] IO_sdram_dq,

    // -----------------------------------------------------------------------
    // Horloge et reset
    // -----------------------------------------------------------------------
    input               clk,            // 32 MHz
    input               reset_n,        // actif bas — init SDRAM quand 0

    // Contrôleur prêt (init terminée)
    output              ready,
    output       [4:0]  init_cnt_dbg,  // debug : valeur courante de init_cnt

    // -----------------------------------------------------------------------
    // Port 0 — CPU (R/W 8 bits)
    // -----------------------------------------------------------------------
    input       [22:0]  cpu_addr,       // adresse 23 bits (byte)
    input        [7:0]  cpu_din,        // données CPU → SDRAM (écriture)
    output reg   [7:0]  cpu_dout,       // données SDRAM → CPU (lecture)
    input               cpu_req,        // impulsion : requête transaction
    input               cpu_we,         // 1 = écriture, 0 = lecture
    output reg          cpu_ack,        // impulsion : transaction terminée

    // -----------------------------------------------------------------------
    // Port 1 — VRAM (lecture 16 bits, rafraîchissement automatique)
    // -----------------------------------------------------------------------
    input       [21:0]  vram_addr,      // adresse 22 bits (mots de 16 bits)
    output reg  [15:0]  vram_dout,      // données 16 bits lues

    // -----------------------------------------------------------------------
    // Port 2 — Boot/Tape (R/W 8 bits)
    // -----------------------------------------------------------------------
    input       [22:0]  boot_addr,      // adresse 23 bits (byte)
    input        [7:0]  boot_din,       // données → SDRAM (écriture)
    output reg   [7:0]  boot_dout,      // données ← SDRAM (lecture)
    input               boot_req,       // impulsion : requête transaction
    input               boot_we,        // 1 = écriture, 0 = lecture
    output reg          boot_ack        // impulsion : transaction terminée
);

// ---------------------------------------------------------------------------
// Paramètres de timing SDRAM @ 32 MHz
// ---------------------------------------------------------------------------
// tCK = 31.25 ns
// tRCD (RAS→CAS) ≥ 15 ns  → 1 cycle suffisant
// CAS Latency   = 2 cycles
// tRP (Precharge) ≥ 15 ns → 1 cycle suffisant
// Un cycle lecture : IDLE(0)→ACTIVE(1)→CAS(2)→NOP1(3)→NOP2(4)→LATCH(5)→IDLE
// Un cycle écriture : IDLE(0)→ACTIVE(1)→CAS(2)→NOP1(3)→IDLE (ACK dans NOP1)
// En mode auto-refresh : IDLE→AREF→NOP→...→LAST(6)→IDLE

localparam RASCAS_DELAY  = 3'd1;   // tRCD : 1 cycle @ 32 MHz
localparam CAS_LATENCY   = 3'd2;   // CL = 2

// États de la machine principale
localparam ST_IDLE   = 3'd0;       // choix de la prochaine opération
localparam ST_RAS    = 3'd1;       // CMD_ACTIVE (RAS)
localparam ST_CAS    = 3'd2;       // CMD_READ ou CMD_WRITE (CAS)
localparam ST_NOP1   = 3'd3;       // NOP après CAS (CL=2, cycle 1)
localparam ST_NOP2   = 3'd4;       // NOP après CAS (O_sdram_clk=~clk → +0.5 cycle)
localparam ST_LATCH  = 3'd5;       // capture données lues
localparam ST_LAST   = 3'd6;       // dernier état du cycle complet (auto-refresh)

// Mode Register : burst=1, CL=2, séquentiel, pas de write burst
localparam BURST_LEN     = 3'b000; // 1 mot
localparam ACCESS_TYPE   = 1'b0;   // séquentiel
localparam NO_WR_BURST   = 1'b1;
localparam OP_MODE       = 2'b00;
localparam MODE_REG = {3'b000, NO_WR_BURST, OP_MODE, CAS_LATENCY, ACCESS_TYPE, BURST_LEN};

// ---------------------------------------------------------------------------
// Commandes SDRAM (CS_n, RAS_n, CAS_n, WE_n)
// ---------------------------------------------------------------------------
localparam CMD_INHIBIT       = 4'b1111;
localparam CMD_NOP           = 4'b0111;
localparam CMD_ACTIVE        = 4'b0011;
localparam CMD_READ          = 4'b0101;
localparam CMD_WRITE         = 4'b0100;
localparam CMD_PRECHARGE     = 4'b0010;
localparam CMD_AUTO_REFRESH  = 4'b0001;
localparam CMD_LOAD_MODE     = 4'b0000;

reg [3:0] sd_cmd;
assign O_sdram_cs_n  = sd_cmd[3];
assign O_sdram_ras_n = sd_cmd[2];
assign O_sdram_cas_n = sd_cmd[1];
assign O_sdram_wen_n = sd_cmd[0];

// Horloge SDRAM = ~clk (données stables sur front montant SDRAM)
assign O_sdram_clk = ~clk;
assign O_sdram_cke = 1'b1;

// ---------------------------------------------------------------------------
// Registre de données bidirectionnel
// ---------------------------------------------------------------------------
reg  [31:0] sd_dq_out;
reg         sd_dq_oe;     // 1 = drive (écriture), 0 = haute impédance

// DQM actifs pendant l'écriture pour sélectionner l'octet (bus 32 bits)
// Lors d'une écriture 8 bits : dupliquer l'octet sur les 4 lanes,
// et activer uniquement le DQM correspondant au byte lane voulu.
// addr[1:0] sélectionne le lane 0-3 dans le mot 32 bits.
reg [3:0] sd_dqm_r;
assign O_sdram_dqm   = sd_dqm_r;
assign IO_sdram_dq   = sd_dq_oe ? sd_dq_out : 32'bz;

// ---------------------------------------------------------------------------
// Initialisation (séquence SDRAM standard)
// ---------------------------------------------------------------------------
reg [4:0] init_cnt;      // compte à rebours d'initialisation (31→0)
assign ready       = (init_cnt == 5'd0);
assign init_cnt_dbg = init_cnt;

// ---------------------------------------------------------------------------
// Machine d'état principale
// ---------------------------------------------------------------------------
reg  [2:0]  state;

// Signaux d'arbitrage interne
reg         do_cpu;       // transaction CPU en cours
reg         do_boot;      // transaction Boot en cours
reg         do_vram;      // transaction VRAM en cours
reg         do_aref;      // auto-refresh en cours
reg         cur_we;       // direction de la transaction courante
reg  [22:0] cur_addr;     // adresse courante (23 bits byte)
reg   [7:0] cur_din;      // donnée à écrire

// Détection du changement d'adresse VRAM
reg  [21:0] vram_addr_r;  // dernière adresse VRAM servie

// Compteur de rafraîchissement (auto-refresh toutes les ~7.8 µs @ 32 MHz)
// 32 MHz × 7.8 µs = 249 cycles → compteur 8 bits avec seuil 230
localparam REFRESH_PERIOD = 9'd230;
reg  [8:0]  refresh_cnt;
reg         refresh_req;  // rafraîchissement en attente

// Latches requêtes entrantes (edge detect)
reg         cpu_req_r;
reg         boot_req_r;

// Latch boot_req : capture l'impulsion même si le SDRAM est occupé (refresh, etc.)
// Sans ce latch, une impulsion de 1 cycle coincidant avec un refresh (4 cycles)
// est perdue → blocage éternel en ST_WR_LO_ACK / ST_WR_HI_ACK.
reg         boot_req_lat;    // requête boot en attente (persistent)
reg  [22:0] boot_addr_lat;   // adresse sauvegardée à la détection du front
reg  [7:0]  boot_din_lat;    // donnée sauvegardée
reg         boot_we_lat;     // direction sauvegardée

// Latch cpu_req : même mécanisme que boot — capture le front montant
// de cpu_req pour ne pas perdre la requête si le SDRAM est occupé.
reg         cpu_req_lat;
reg  [22:0] cpu_addr_lat;
reg   [7:0] cpu_din_lat;
reg         cpu_we_lat;

always @(posedge clk) begin
    // Valeurs par défaut
    sd_cmd    <= CMD_NOP;
    sd_dq_oe  <= 1'b0;
    sd_dqm_r  <= 4'b1111;  // masque tout pendant les cycles non-écriture
    cpu_ack   <= 1'b0;
    boot_ack  <= 1'b0;

    // --------------------------------------------------------------------
    // Compteur de rafraîchissement
    // --------------------------------------------------------------------
    if (!reset_n) begin
        refresh_cnt <= 9'd0;
        refresh_req <= 1'b0;
    end else if (refresh_cnt >= REFRESH_PERIOD) begin
        refresh_cnt <= 9'd0;
        refresh_req <= 1'b1;
    end else begin
        refresh_cnt <= refresh_cnt + 9'd1;
    end

    // --------------------------------------------------------------------
    // Séquence d'initialisation
    // --------------------------------------------------------------------
    if (!reset_n) begin
        init_cnt     <= 5'd31;
        state        <= ST_IDLE;
        do_cpu       <= 1'b0;
        do_boot      <= 1'b0;
        do_vram      <= 1'b0;
        do_aref      <= 1'b0;
        cpu_req_r    <= 1'b0;
        cpu_req_lat  <= 1'b0;
        boot_req_r   <= 1'b0;
        boot_req_lat <= 1'b0;
        vram_addr_r  <= 22'h3FFFFF; // forcer un fetch VRAM au démarrage (toutes bits à 1)
        sd_cmd    <= CMD_INHIBIT;

    end else if (init_cnt != 5'd0) begin
        // Phase d'initialisation : cycler les états, envoyer PRECHARGE puis
        // deux AUTO_REFRESH puis LOAD_MODE_REG
        state <= state + 3'd1;

        if (state == ST_LAST)
            init_cnt <= init_cnt - 5'd1;

        if (state == ST_IDLE) begin
            case (init_cnt)
                5'd14: begin
                    // PRECHARGE ALL
                    sd_cmd        <= CMD_PRECHARGE;
                    O_sdram_addr  <= 11'b10000000000; // A10=1 = precharge all
                    O_sdram_ba    <= 2'b00;
                end
                5'd4, 5'd2: begin
                    // AUTO REFRESH (×2 requis)
                    sd_cmd <= CMD_AUTO_REFRESH;
                end
                5'd1: begin
                    // LOAD MODE REGISTER
                    sd_cmd       <= CMD_LOAD_MODE;
                    O_sdram_addr <= MODE_REG[10:0];
                    O_sdram_ba   <= 2'b00;
                end
                default: ;
            endcase
        end

    end else begin
        // ----------------------------------------------------------------
        // Fonctionnement normal
        // ----------------------------------------------------------------

        // Détection front montant sur cpu_req et boot_req
        cpu_req_r  <= cpu_req;
        boot_req_r <= boot_req;

        // Latch boot_req : capture le front même si le SDRAM est occupé
        if (boot_req && !boot_req_r) begin
            boot_req_lat  <= 1'b1;
            boot_addr_lat <= boot_addr;
            boot_din_lat  <= boot_din;
            boot_we_lat   <= boot_we;
        end else if (boot_ack) begin
            boot_req_lat  <= 1'b0;
        end

        // Latch cpu_req : même principe — capture le front montant
        if (cpu_req && !cpu_req_r) begin
            cpu_req_lat  <= 1'b1;
            cpu_addr_lat <= cpu_addr;
            cpu_din_lat  <= cpu_din;
            cpu_we_lat   <= cpu_we;
        end else if (cpu_ack) begin
            cpu_req_lat  <= 1'b0;
        end

        // Machine d'état
        case (state)

            // --------------------------------------------------------
            // ST_IDLE : arbitrage et démarrage d'un cycle
            // --------------------------------------------------------
            ST_IDLE: begin
                do_cpu  <= 1'b0;
                do_boot <= 1'b0;
                do_vram <= 1'b0;
                do_aref <= 1'b0;

                // Priorité : Boot > VRAM > CPU > Auto-Refresh
                if (boot_req_lat) begin
                    // Transaction Boot (ROM loading / tape) — via latch persistant
                    do_boot  <= 1'b1;
                    cur_we   <= boot_we_lat;
                    cur_addr <= boot_addr_lat;
                    cur_din  <= boot_din_lat;
                    sd_cmd   <= CMD_ACTIVE;
                    O_sdram_addr <= boot_addr_lat[20:10]; // ligne
                    O_sdram_ba   <= boot_addr_lat[22:21]; // banque
                    state    <= ST_RAS;

                end else if (vram_addr != vram_addr_r) begin
                    // Lecture VRAM : l'adresse a changé
                    do_vram  <= 1'b1;
                    cur_we   <= 1'b0;
                    cur_addr <= {vram_addr, 1'b0};    // adresse 23 bits (mot→byte LSB)
                    vram_addr_r <= vram_addr;
                    sd_cmd   <= CMD_ACTIVE;
                    O_sdram_addr <= vram_addr[19:9];  // ligne (bits [20:10] de l'adresse byte)
                    O_sdram_ba   <= vram_addr[21:20]; // banque
                    state    <= ST_RAS;

                end else if (cpu_req_lat) begin
                    // Transaction CPU — via latch persistant
                    do_cpu   <= 1'b1;
                    cur_we   <= cpu_we_lat;
                    cur_addr <= cpu_addr_lat;
                    cur_din  <= cpu_din_lat;
                    sd_cmd   <= CMD_ACTIVE;
                    O_sdram_addr <= cpu_addr_lat[20:10];  // ligne
                    O_sdram_ba   <= cpu_addr_lat[22:21];  // banque
                    state    <= ST_RAS;

                end else if (refresh_req) begin
                    // Auto-refresh
                    do_aref      <= 1'b1;
                    refresh_req  <= 1'b0;
                    sd_cmd       <= CMD_AUTO_REFRESH;
                    state        <= ST_RAS;
                end
                // sinon on reste en IDLE
            end

            // --------------------------------------------------------
            // ST_RAS : après CMD_ACTIVE, attendre tRCD=1 cycle
            // --------------------------------------------------------
            ST_RAS: begin
                state <= ST_CAS;
            end

            // --------------------------------------------------------
            // ST_CAS : envoi de la commande READ ou WRITE
            // --------------------------------------------------------
            ST_CAS: begin
                state <= ST_NOP1;

                if (do_aref) begin
                    // Refresh : pas de CAS — retour direct à IDLE après attente
                    state <= ST_NOP1;
                end else if (cur_we) begin
                    // WRITE
                    sd_cmd <= CMD_WRITE;
                    // Colonne 8 bits : A10=1 (auto-precharge), A[9:8]=0, A[7:0]=col
                    // Bus 32 bits : chaque colonne = 4 octets → col = addr[9:2]
                    O_sdram_addr <= {1'b1, 2'b00, cur_addr[9:2]};  // A10=AP, col 8-bit
                    // Dupliquer l'octet sur les 4 lanes, sélectionner le bon DQM
                    sd_dq_out <= {4{cur_din}};
                    sd_dq_oe  <= 1'b1;
                    // DQM : 0 = actif (laisse passer), 1 = masqué
                    // cur_addr[1:0] sélectionne le byte lane dans les 32 bits
                    case (cur_addr[1:0])
                        2'b00: sd_dqm_r <= 4'b1110;
                        2'b01: sd_dqm_r <= 4'b1101;
                        2'b10: sd_dqm_r <= 4'b1011;
                        2'b11: sd_dqm_r <= 4'b0111;
                    endcase
                    O_sdram_ba <= cur_addr[22:21];
                end else begin
                    // READ (CPU ou VRAM)
                    sd_cmd <= CMD_READ;
                    O_sdram_addr <= {1'b1, 2'b00, cur_addr[9:2]};  // A10=AP, col 8-bit
                    sd_dqm_r  <= 4'b0000; // lire tous les octets
                    O_sdram_ba <= cur_addr[22:21];
                end
            end

            // --------------------------------------------------------
            // ST_NOP1 : NOP, attente CAS Latency cycle 1
            //   Écriture : ACK immédiat (pas de CL pour writes)
            //   Lecture   : continuer vers NOP2 (CL=2 nécessite 2 NOPs
            //               car O_sdram_clk = ~clk décale d'un demi-cycle)
            //   Refresh   : attente tRC
            // --------------------------------------------------------
            ST_NOP1: begin
                sd_dq_oe  <= 1'b0;
                if (do_aref) begin
                    // Refresh : attente tRC
                    state <= ST_LAST;
                end else if (cur_we) begin
                    // Écriture terminée (auto-precharge), ACK immédiat
                    if (do_cpu)  cpu_ack  <= 1'b1;
                    if (do_boot) boot_ack <= 1'b1;
                    state <= ST_IDLE;
                end else begin
                    // Lecture : attendre un cycle supplémentaire (CL=2)
                    sd_dqm_r <= 4'b0000;   // garder DQ output activé
                    state <= ST_NOP2;
                end
            end

            // --------------------------------------------------------
            // ST_NOP2 : NOP, attente CL=2 + demi-cycle O_sdram_clk=~clk
            //   Données SDRAM apparaissent à negedge clk (= posedge SDRAM)
            //   après CL=2. On les échantillonne au prochain posedge (LATCH).
            // --------------------------------------------------------
            ST_NOP2: begin
                state <= ST_LATCH;
            end

            // --------------------------------------------------------
            // ST_LATCH : capture des données lues
            //   Les données sont stables depuis le falling edge précédent.
            // --------------------------------------------------------
            ST_LATCH: begin
                // Les données sont présentes sur IO_sdram_dq
                if (do_vram) begin
                    // Lecture 16 bits : prendre D[15:0] ou D[31:16] selon lane
                    // vram_addr[0] = 0 → D[15:0], = 1 → D[31:16]
                    vram_dout <= cur_addr[1] ? IO_sdram_dq[31:16] : IO_sdram_dq[15:0];
                end else if (do_cpu) begin
                    case (cur_addr[1:0])
                        2'b00: cpu_dout <= IO_sdram_dq[ 7: 0];
                        2'b01: cpu_dout <= IO_sdram_dq[15: 8];
                        2'b10: cpu_dout <= IO_sdram_dq[23:16];
                        2'b11: cpu_dout <= IO_sdram_dq[31:24];
                    endcase
                    cpu_ack <= 1'b1;
                end else if (do_boot) begin
                    case (cur_addr[1:0])
                        2'b00: boot_dout <= IO_sdram_dq[ 7: 0];
                        2'b01: boot_dout <= IO_sdram_dq[15: 8];
                        2'b10: boot_dout <= IO_sdram_dq[23:16];
                        2'b11: boot_dout <= IO_sdram_dq[31:24];
                    endcase
                    boot_ack <= 1'b1;
                end
                state <= ST_IDLE;
            end

            // --------------------------------------------------------
            // ST_LAST : fin du cycle refresh (tRC ≥ 63 ns = 2 cycles)
            // --------------------------------------------------------
            ST_LAST: begin
                state <= ST_IDLE;
            end

            default: state <= ST_IDLE;
        endcase
    end
end

endmodule

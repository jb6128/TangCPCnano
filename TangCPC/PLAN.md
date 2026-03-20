# TangCPC : Port Amstrad CPC 6128 → Tang Nano 20K

## Architecture cible

```
Oscillateur 27 MHz (Tang Nano 20K, pin 4)
         |
    [rPLL #1: 160 MHz] --> /5 CLKDIV --> 32 MHz  (clk_sys + clk_pixel)
    [rPLL #2: 100 MHz] --> flash SPI
         |
    [cpc_core.sv]  <--SPI--> BL616 MCU (OSD / clavier / joystick / SD)
         |
    [Amstrad_motherboard.v]  <-- inchangé du source MiSTer
         |-- GA40010 (timing vidéo, arbitrage mémoire)
         |-- T80pa   (VHDL Z80 CPU)
         |-- UM6845R (CRTC)
         |-- YM2149  (PSG)
         |-- u765    (FDC)
         |-- i8255   (PPI)
         |
    [cpc_sdram.v]  <----> SDRAM 32 MB
    [cpc_flash.v]  <----> Flash SPI 16 MB (ROMs CPC)
    [cpc_video.sv] ------> HDMI (color_mix → scandoubler → OSD → TMDS)
```

## Structure de fichiers

```
TangCPC/
├── TangCPC.gprj
├── impl/pnr/TangCPC.fs
├── src/
│   ├── top/
│   │   └── top.sv              NOUVEAU  (depuis nano20k/top.sv)
│   ├── core/
│   │   └── cpc_core.sv         NOUVEAU  (depuis misterynano.sv)
│   ├── rtl/                    COPIÉ tel quel depuis Amstrad_MiSTer-master/rtl/
│   │   ├── Amstrad_motherboard.v
│   │   ├── Amstrad_MMU.v
│   │   ├── GA40010/
│   │   ├── T80/                (VHDL — supporté par GOWIN)
│   │   ├── UM6845R.v
│   │   ├── YM2149.sv
│   │   ├── u765/u765.sv
│   │   ├── i8255.v
│   │   ├── color_mix.sv
│   │   ├── crt_filter.v
│   │   ├── progressbar.v
│   │   ├── dandanator/         (optionnel)
│   │   └── playcity/           (optionnel)
│   ├── sdram/
│   │   └── cpc_sdram.v         NOUVEAU/ADAPTÉ (depuis MiSTeryNano sdram.v)
│   ├── flash/
│   │   └── cpc_flash.v         COPIÉ (flash_dspi.v)
│   ├── video/
│   │   ├── cpc_video.sv        NOUVEAU
│   │   └── video2hdmi.v        COPIÉ
│   ├── mcu/
│   │   ├── mcu_spi.v           COPIÉ
│   │   ├── sysctrl.v           ADAPTÉ (options CPC)
│   │   ├── hid_cpc.v           NOUVEAU (matrice 10×8 CPC)
│   │   ├── hid_to_ps2.v        NOUVEAU (shim HID→PS/2)
│   │   ├── osd_u8g2.v          COPIÉ
│   │   ├── sd_card.v           COPIÉ
│   │   └── sd_rw.v             COPIÉ
│   └── pll/
│       ├── pll_160m.v          COPIÉ
│       ├── pll_flash.v         COPIÉ
│       └── gowin_clkdiv.v      COPIÉ
├── constraint/
│   ├── TangCPC.cst             NOUVEAU (depuis nano20k/atarist.cst)
│   └── TangCPC.sdc             NOUVEAU (depuis nano20k/atarist.sdc)
└── firmware/                   BL616 — adapter depuis MiSTeryNano firmware/
```

---

## Phases d'implémentation

### Phase 0 — Scaffolding ✅ COMPLÉTÉ
**Objectif :** Projet GOWIN qui synthétise.

**Réalisé :**
1. Structure de répertoires créée (`src/top`, `src/pll`, `src/video`, `src/mcu`, `src/flash`, `src/sdram`, `constraint`, `impl/pnr`)
2. PLLs, HDMI, MCU SPI, scandoubler, flash, SDRAM copiés depuis MiSTeryNano
3. `top.sv` minimal — PLLs + pins, SDRAM/flash/SPI en mode sûr
4. `TangCPC.cst` — adapté depuis `atarist.cst`
5. `TangCPC.sdc` — contrainte horloge 27 MHz
6. `build.tcl` — script `gw_sh` pour synthèse en ligne de commande

**Livrable :** ✅ Synthèse + placement + routage + bitstream OK en ~25 secondes

**Leçons apprises (important pour la suite) :**
- Module CLKDIV s'appelle `Gowin_CLKDIV` (avec majuscules) — pas `gowin_clkdiv`
- Banks 4 et 5 du GW2AR-18 sont à **1.8V** (verrouillés par SDRAM interne) → pas de `IO_TYPE=LVCMOS33` sur les pins 25-31, 41-42, 51
- Pins flash (59-63) = MSPI dédié, pins SPI2 (54-56) = SSPI dédié → nécessite `set_option -use_mspi_as_gpio 1` et `-use_sspi_as_gpio 1` dans le TCL
- GOWIN SDC ne supporte pas la continuation de ligne par backslash `\`
- `flash_pll` optimisée (swept) si non connectée — normal en Phase 0
- `m0s[4]` (pin 51, bank 4) : problème de routing tristate entre bank 4 et bank 3 → signaux m0s séparés en ports individuels, `m0s_4` reporté à Phase 6

---

### Phase 1 — Contrôleur SDRAM multi-port ✅ COMPLÉTÉ
**Objectif :** Adapter le contrôleur MiSTeryNano (1 port, 16-bit) pour le CPC (3 ports, 8-bit).

**Réalisé :**
1. `cpc_sdram.v` écrit (~290 lignes) — 3 ports avec arbitrage fixe Boot > VRAM > CPU > Auto-Refresh
2. Machine d'état 5 états @ 32 MHz : ST_IDLE → ST_RAS → ST_CAS → ST_NOP1 → ST_LATCH → ST_IDLE
3. Bus 32 bits → sélection octet par `addr[1:0]` + DQM
4. Auto-refresh toutes les ~230 cycles (7.2 µs, dans les 7.8 µs requis)
5. Initialisation standard : PRECHARGE ALL → AUTO_REFRESH ×2 → LOAD_MODE_REG
6. Intégré dans `top.sv` avec placeholders pour les 3 ports (Phase 3 les connectera)

**Layout SDRAM :**
```
Bank 0 : RAM CPC 6128 (512 KB) + ROMs (OS/BASIC/AMSDOS)
Bank 1 : RAM CPC 664 (128 KB) + ROMs
Bank 2 : Buffer tape / Dandanator EEPROM
Bank 3 : ROMs d'extension (256 pages × 16 KB)
```

**Livrable :** ✅ 53 LUTs, 22 FF — bitstream OK, aucune violation timing

**Leçons apprises :**
- Bus 32 bits GW2AR-18 : dupliquer l'octet sur les 4 lanes (`{4{din}}`), sélectionner avec DQM[3:0]
- `addr[1:0]` sélectionne le byte lane dans le mot 32 bits (pas `addr[0]` seul comme en 16 bits)
- VRAM port : comparaison `vram_addr != vram_addr_r` pour détecter les changements d'adresse
- Arbitrage Boot > VRAM > CPU : en phase de boot, Boot monopolise le bus, VRAM et CPU attendent

---

### Phase 2 — Chargeur ROM depuis Flash SPI (2 jours)
**Objectif :** Remplacer le chargement HPS par un boot depuis la flash.

**Layout flash (offset depuis fin du bitstream) :**
```
0x200000 : CPC 6128 OS ROM    (16 KB)
0x204000 : CPC 6128 BASIC ROM (16 KB)
0x208000 : AMSDOS ROM         (16 KB)
0x20C000 : MF2 ROM            (16 KB)
0x210000 : CPC 664 OS ROM     (16 KB)
0x214000 : CPC 664 BASIC ROM  (16 KB)
0x218000 : ROMs d'extension   (jusqu'à 4 MB)
```

**Machine d'état de boot dans `cpc_core.sv` :**
```
BOOT_IDLE → BOOT_FLASH_READ → BOOT_SDRAM_WRITE → (boucle) → BOOT_DONE → release reset
```

**Livrable :** SDRAM contient les ROMs valides au démarrage.

---

### Phase 3 — Intégration core + première image

#### Phase 3a — Copie RTL + compilation propre ✅ COMPLÉTÉ
**Objectif :** Copier les fichiers RTL Amstrad et compiler sans erreurs.

**Réalisé :**
1. Copie de tous les fichiers RTL depuis Amstrad_MiSTer-master/rtl/ vers src/rtl/
2. Corrections de compatibilité GOWIN SV2017 :
   - `T80pa.vhd` : port `DO` → `DO_Z80` (mot réservé SV)
   - `Amstrad_motherboard.v` : `.do(D)` → `.do_z80(D)`
   - `Amstrad_MMU.v` : `reg old_wr = 0;` sorti du bloc `always`
   - `hid.sv` : `reg alt`, `mstb` réordonné, `output reg dir`
   - `u765.sv` : fonction `SECTOR_SIZE` réécrite (pas de `reg` local)
3. `build.tcl` mis à jour avec 23 fichiers RTL

**Livrable :** ✅ 295 LUTs, 0 violation, bitstream généré

#### Phase 3b — Amstrad_motherboard instancié (signaux neutres) ✅ COMPLÉTÉ
**Objectif :** Instancier `Amstrad_motherboard` dans `cpc_core.sv` avec `reset=1`, vérifier la synthèse.

**Réalisé :**
1. `cpc_core.sv` : ajout de 20 ports de sortie (vidéo + mémoire + bus CPU)
2. Génération `ce_16` : toggle clk32 → enable 16 MHz
3. `Amstrad_motherboard` instancié avec `reset=1'b1` (constant)
4. Signaux neutres : joy=0x7F, ps2=0, ppi_jumpers=0xF, cpu_din=0xFF, vram_din=0
5. `top.sv` : wires déclarés et connectés pour tous les nouveaux ports

**Note :** 295 LUTs (inchangé) — propagation constante reset=1 élimine tout le RTL Amstrad.
C'est normal : Phase 3c remplacera 1'b1 par ~boot_done_r.

**Livrable :** ✅ 295 LUTs, 0 violation, bitstream généré

#### Phase 3c — Connexion SDRAM (CPU + VRAM) ✅ COMPLÉTÉ
**Objectif :** Connecter les ports CPU et VRAM du contrôleur SDRAM à `Amstrad_motherboard`.

**Réalisé :**
1. `cpc_core.sv` : `reset(~boot_done_r)` — CPU libéré après chargement ROMs
2. `cpc_core.sv` : port `vram_din_i[15:0]` ajouté, connecté à `Amstrad_motherboard.vram_din`
3. `top.sv` : port CPU SDRAM connecté :
   - `cpu_addr_w = core_mem_addr` (adresse 23-bit depuis MMU)
   - `cpu_din_w = core_cpu_dout` (données Z80 → SDRAM pour écriture)
   - `cpu_req_w = core_mem_rd | core_mem_wr`
   - `cpu_we_w = core_mem_wr`
4. `top.sv` : port VRAM SDRAM connecté :
   - `vram_addr_w = {7'b0000010, core_vram_addr}` (conversion 15-bit CRTC → 22-bit word addr)
   - RAM CPC base 64KB en SDRAM bytes 0x020000–0x02FFFF → word 0x010000 + vram_addr

**Livrable :** ✅ 3 255 LUTs (3 063 LUT + 192 ALU), 1 222 FF, 1 BSRAM — 0 violation, bitstream OK

#### Phase 3d — Pipeline vidéo + HDMI ✅ COMPLÉTÉ
**Objectif :** Connecter les sorties GA40010 vers HDMI.

**Réalisé :**
1. `build.tcl` : 12 fichiers vidéo ajoutés (scandoubler, video2hdmi, hdmi.sv + paquets HDMI)
2. `top.sv` : pipeline vidéo instancié :
   - `color_mix` : 2-bit RGB GA40010 → palette CPC 8-bit, ce_pix=16MHz toggle
   - `scandoubler` : HCNT_WIDTH=10, ce_divider=1 (16MHz→32MHz), bypass=0
   - `video2hdmi` : vmode=1 (PAL 50Hz), audio=0 (Phase 7)
3. Correction `TangCPC.cst` : `IO_TYPE=LVCMOS33` ajouté sur io[0-5], spare[3], m0s_0/1
   (ELVDS_OBUF HDMI force une vérification stricte de bank VCCIO)

**Livrable :** ✅ 4 548 LUTs (4 206 LUT + 342 ALU), 2 027 FF, 3 BSRAM — 0 erreur, bitstream OK

**Note :** 22% LUTs utilisés — budget 78% restant pour HID, FDC, OSD, audio.

---

### Phase 4 — HID : Clavier et Joystick (2-3 jours)
**Objectif :** Mapper les codes HID BL616 vers la matrice CPC.

**Approche retenue — shim HID→PS/2 (`hid_to_ps2.v`) :**
- LUT de 128 entrées : HID keycode → AT Set 2 scancode
- Génère `ps2_key[10:0]` au format MiSTer
- `Amstrad_motherboard.v` inchangé (son `hid.sv` interne fait la traduction)

**Coût :** ~1 BRAM9 + ~50 LUTs

**Joystick :**
```
hid_joy[0][5:0] → joy1[6:0]  (format Kempston : fire2/fire1/right/left/down/up/fire3)
hid_joy[1][5:0] → joy2[6:0]
```

**Livrable :** Saisie clavier fonctionnelle à l'invite BASIC.

---

### Phase 5 — FDC et carte SD (3-4 jours)
**Objectif :** Connecter `u765.sv` au MCU BL616 via `sd_card.v`.

**Bridge u765 ↔ sd_card :**
```
u765.sd_rd[1:0]    → sd_card.rstart[1:0]
u765.sd_wr[1:0]    → sd_card.wstart[1:0]
u765.sd_lba[31:0]  → sd_card.rsector[31:0]
sd_card.rdone      → u765.sd_ack
sd_card.image_mounted → u765.img_mounted
sd_card.outbyte    → u765.sd_buff_dout
u765.sd_buff_din   → sd_card.inbyte
```

Petit module de glue (~50 LUTs) pour synchroniser `sd_buff_addr`.

**Livrable :** Images DSK montables, chargement de programmes.

---

### Phase 6 — OSD et sysctrl (1-2 jours)
**Objectif :** Menu de configuration via BL616.

**Options OSD CPC :**
- Drive A/B : monter image DSK
- Tape : monter CDT
- Modèle : CPC 6128 / CPC 664
- CRTC : Type 0 / Type 1
- Affichage : Couleur / Vert / Ambre / Cyan
- Scan lines : Aucune / 25% / 50% / 75%
- Vitesse CPU : Normale / Rapide
- Vitesse FDC : Normale / Rapide / Désactivé
- Mix stéréo, échange joysticks, Multiface 2, PlayCity
- Reset

**Livrable :** OSD fonctionnel, configuration persistante.

---

### Phase 7 — Audio (1 jour)
**Objectif :** YM2149 vers HDMI audio.

```
audio_l[7:0] → sign-extend → 16-bit signé → video2hdmi (48 kHz intégré)
audio_r[7:0] → idem
+ mix tape audio si activé
```

**Livrable :** Son CPC via HDMI.

---

### Phase 8 — Fonctionnalités optionnelles (2-3 jours)
**Si budget LUTs disponible :**
- **Multiface 2** : 8 KB BRAM (1 BRAM36), logique NMI ~300 LUTs
- **Dandanator** : VHDL, EEPROM en SDRAM bank 3, ~600 LUTs
- **PlayCity** : 2e YM2149 + Z80 CTC, ~1000 LUTs
- **Tape CDT** : streaming via MCU SPI dans BRAM tampon

---

## Budget LUTs estimé

| Bloc | LUTs estimés |
|------|-------------|
| Amstrad_motherboard (GA, i8255, PSG, CRTC, MMU) | 8 000–10 000 |
| T80pa (Z80 VHDL) | 1 500–2 000 |
| u765 FDC | 1 500–2 000 |
| cpc_sdram (multi-port) | 400–600 |
| Flash boot loader | 200–300 |
| MCU SPI + sysctrl + HID + OSD | 800–1 200 |
| Pipeline vidéo (color_mix + scandoubler + HDMI) | 600–1 000 |
| sd_card + sd_rw | 600–800 |
| MF2 (logique seule) | 200–300 |
| hid_to_ps2 shim | 100–150 |
| Glue top-level | 200–300 |
| **Total sans optionnels** | **~14 100–18 650** |
| Dandanator + PlayCity (optionnel) | +1 300–1 900 |
| **Total avec optionnels** | **~15 400–20 550** |
| **Capacité GW2AR-18** | **20 736 LUTs** |

---

## Checkpoints de synthèse

| CP | Après phase | Critère | Statut |
|----|-------------|---------|--------|
| CP-1 | 0 | < 200 LUTs, PLL lock, bitstream généré | ✅ OK |
| CP-2 | 1 | 53 LUTs, pas de violation timing sur clk32 | ✅ OK |
| CP-3a | 3a | 295 LUTs, 0 violation, tous RTL compilés | ✅ OK |
| CP-3b | 3b | 295 LUTs (propagation constante), 0 violation | ✅ OK |
| CP-3c | 3c | 3 255 LUTs, CPU+VRAM connectés, timing clean | ✅ OK |
| CP-4 | 3d | 4 548 LUTs, pipeline vidéo complet, bitstream OK | ✅ OK |
| CP-4 | 4 | Clavier fonctionnel | ⏳ |
| CP-5 | 5 | Chargement DSK OK | ⏳ |
| CP-6 | 8 | < 20 500 LUTs, timing clean sur les 3 domaines | ⏳ |

---

## Risques

| Phase | Risque | Mitigation |
|-------|--------|------------|
| SDRAM multi-port | Timing VRAM critique pour GA40010 | Simulation avant synthèse |
| Flash boot | Adressage offset ROMs | Vérifier avec MiSTeryNano flash_prog_cfg |
| Intégration core | Domaines d'horloge, timing bus CPC | Simuler Amstrad_motherboard isolément |
| FDC/SD | Sync sd_buff_addr | Module de glue dédié |
| Fit LUTs | > 85% utilisation → problèmes routage | Désactiver PlayCity + Dandanator en 1ère passe |

---

## Fichiers critiques de référence

| Fichier | Rôle |
|---------|------|
| `MiSTeryNano/src/tang/nano20k/top.sv` | Modèle pour `top.sv` |
| `Amstrad_MiSTer/Amstrad.sv` | Inventaire complet des signaux à remplacer |
| `MiSTeryNano/src/tang/nano20k/sdram.v` | Base de `cpc_sdram.v` |
| `Amstrad_MiSTer/rtl/Amstrad_motherboard.v` | Core inchangé — définit toutes les interfaces |
| `MiSTeryNano/src/misterynano.sv` | Modèle structurel pour `cpc_core.sv` |

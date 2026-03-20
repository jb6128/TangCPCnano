# TangCPC — Journal de debug

## Symptôme initial (Phase 3d)
- **HDMI** : écran bleu avec hachures noires (désynchro)
- **LED mapping original** : CRTC R0/R4/R7/R9 checks + vsync counter + boot_done
- LED0 (boot_done) : allumée fixe → boot loader atteint ST_DONE ✅
- LED1 (vsync/32) : cycle ~30s → vsync CRTC ~2 Hz au lieu de 50 Hz
- LED2-5 (CRTC regs) : toutes éteintes → CRTC jamais programmé par le BASIC
- **Conclusion** : Z80 n'exécute pas l'OS ROM → CRTC reste en config par défaut

## Hypothèse 1 — Timing lecture SDRAM (CAS latency)
- **Théorie** : Avec `O_sdram_clk = ~clk` et CL=2, le contrôleur SDRAM échantillonne
  les données un demi-cycle trop tôt (ST_LATCH au cycle N+2 au lieu de N+3).
- **Fix** : Ajout de ST_NOP2 entre ST_NOP1 et ST_LATCH dans `cpc_sdram.v`
- **Résultat** : ❌ Aucun changement. Comportement identique.
- **Statut** : Fix conservé (probablement correct mais pas suffisant)

## Test 2 — LEDs diagnostiques
- **Mapping LEDs** :
  - [0] boot_done (dbg_boot_state == 12)
  - [1] GA40010 READY vu (latch)
  - [2] cpu_rd_cnt[22] (Z80 mem_rd)
  - [3] SDRAM cpu_ack vu (latch)
  - [4] readback_done
  - [5] readback OK (SDRAM[0] == 0x01)
- **Résultats** :
  - LED0 éteinte — dbg_boot_state != 12 (normal : FSM passe à 13-15 avec readback)
  - LED1 allumée — GA READY fonctionne ✅
  - LED2 clignote ~1s — Z80 fait des mem_rd ✅
  - LED3 allumée — SDRAM répond au CPU ✅
  - LED4 allumée — readback terminé ✅
  - LED5 éteinte — **SDRAM[0x000000] ≠ 0x01** ❌
- **Conclusion** : Le readback via le port Boot SDRAM retourne une mauvaise valeur.
  Soit l'écriture boot a échoué, soit la lecture retourne des données corrompues.
- **HDMI** : toujours écran bleu + hachures noires (inchangé)

## Test 3 — LEDs diagnostiques v2 (W/R test + readback ROM + rb_byte)
- **Mapping LEDs** :
  - [0] FSM terminée (dbg_boot_state >= 15) — NOTE: bug, wire 4 bits tronqué
  - [1] rb_byte[0] — bit 0 de la valeur lue depuis SDRAM[0x000000]
  - [2] ROM readback done
  - [3] ROM readback OK (== 0x01)
  - [4] W/R test done
  - [5] W/R test OK (write 0xAB, read 0xAB)
- **Résultats** :
  - LED0 éteinte — bug affichage (wire tronqué 4 bits vs state 5 bits)
  - LED1 allumée — rb_byte[0] = 1 → valeur lue a bit 0 à 1, mais ≠ 0x01
  - LED2 allumée — readback done ✅
  - LED3 éteinte — **SDRAM[0] ≠ 0x01** ❌
  - LED4 allumée — W/R test done ✅
  - LED5 allumée — **W/R 0xAB test OK** ✅ ← SDRAM fonctionne !
- **Conclusion clé** :
  - **SDRAM fonctionne correctement** (écriture + lecture via boot port)
  - Les données ROM en SDRAM sont **incorrectes**
  - Le problème est soit le contenu flash, soit le module de lecture flash DSPI
  - rb_byte a bit 0 = 1 mais ≠ 0x01 → possiblement 0x89 (byte-swap?) ou autre

## Test 4 — Vérification flash + rb_byte complet
- **Flash verify** : `openFPGALoader --verify --offset 0x200000 boot.rom` → **DONE** ✅
  - Le contenu flash est correct, le problème n'est pas là
- **LED mapping** : rb_byte[7:0] sur LED[0..7] (6 LEDs disponibles)
  - [0] rb_byte[0], [1] rb_byte[1], ... [5] rb_byte[5]
- **Résultats** : LED0 allumée, LED[1-5] éteintes → rb_byte = 0x01 ? Non...
- **Re-test avec autre mapping** : rb_byte = 0x00 (toutes éteintes)
- **Conclusion** : SDRAM[0x000000] = 0x00 après boot

## Test 5 — Capture premier mot flash (dbg_first_flash_word)
- **Mapping LEDs** :
  - [0] W/R SDRAM OK
  - [1] flash_dout[8] (byte0 bit0)
  - [2] flash_dout[12] (byte0 bit4)
  - [3] flash_dout[13] (byte0 bit5)
  - [4] flash_dout[14] (byte0 bit6)
  - [5] flash_dout[15] (byte0 bit7)
- **Résultats** : LED[0,1] allumées, LED[2-5] éteintes
  - W/R test OK à 0x7FF000 ✅
  - flash_dout[15:8] = 0x01 → **Flash DSPI lit correctement le premier octet** ✅
- **Conclusion** :
  - Flash lit 0x01 correctement ✅
  - SDRAM W/R fonctionne à 0x7FF000 ✅
  - Mais SDRAM[0x000000] = 0x00 après le boot ❌
  - Le boot FSM fait 49152 écritures avec acks, mais les données ne persistent pas à l'adresse 0

## Test 6 — W/R test à adresse 0x000000 (en cours)
- **Objectif** : Vérifier si l'écriture/lecture fonctionne à l'adresse 0x000000 spécifiquement
- **Modification** : W/R test changé de 0x7FF000 → 0x000000
- **Statut** : Code modifié, build en attente

## Prochaine étape
- Builder et tester Test 6
- Si W/R OK à 0x000000 → le problème est dans le transfert flash→SDRAM (timing, données)
- Si W/R KO à 0x000000 → problème d'adressage SDRAM (bank 0/row 0)

## Informations de référence
- **boot.rom** : 128 KB, flashé à 0x200000 via openFPGALoader
  - Offset 0x0000 : OS ROM (vérifié : commence par 01 89 7F ED 49)
  - Offset 0x4000 : BASIC ROM (vérifié : contient "BASIC 1.1")
  - Offset 0x8000 : AMSDOS ROM
- **Flash word addresses** : FLASH_BASE_0=0x100000, _1=0x102000, _2=0x104000
- **SDRAM destinations** : 0x000000 (OS), 0x400000 (BASIC), 0x41C000 (AMSDOS)

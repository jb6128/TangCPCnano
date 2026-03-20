//  TangCPC.sdc — Contraintes de timing
//  Tang Nano 20K — GW2AR-18C
//  Note: GOWIN SDC ne supporte pas la continuation de ligne par backslash

// Horloge d'entrée 27 MHz
create_clock -name clk_in -period 37.037 [get_ports {clk}]

// Faux chemins entre domaines d'horloge asynchrones
// (les horloges générées sont définies automatiquement par les PLLs)
set_false_path -from [get_clocks {clk_in}] -to [get_clocks {clk_in}]

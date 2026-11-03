# build.tcl — Script de synthèse GOWIN pour TangCPC
# Usage : gw_sh build.tcl
# Cible : GW2AR-18C (Tang Nano 20K)

set_device GW2AR-LV18QN88C8/I7 -name GW2AR-18C

# Fichiers sources
add_file src/top/top.sv
add_file src/core/cpc_core.sv
add_file src/sdram/cpc_sdram.v
add_file src/flash/cpc_flash.v

# RTL Amstrad CPC (copié depuis Amstrad_MiSTer)
add_file src/rtl/Amstrad_motherboard.v
add_file src/rtl/Amstrad_MMU.v
add_file src/rtl/UM6845R.v
add_file src/rtl/YM2149.sv
add_file src/rtl/i8255.v
add_file src/rtl/hid.sv
add_file src/mcu/mcu_spi.v
add_file src/mcu/mcu_hid.v
add_file src/mcu/hid_cpc_matrix.v
add_file src/mcu/kbd_uart_rx_cpc.v
add_file src/core/cpc_fdc.v
add_file src/mcu/sysctrl.v
add_file src/mcu/sd_arb.v
add_file src/core/cpc_tape.v
add_file src/core/cpc_taprec.v
add_file src/core/cpc_romload.v
add_file src/rtl/tape/tzxplayer.vhd
add_file src/mcu/sd_card.v
add_file src/mcu/sd_rw.v
add_file src/mcu/sdcmd_ctrl.v
add_file src/mcu/osd_u8g2.v
add_file src/mcu/cpc_xml.hex
add_file src/rtl/color_mix.sv
add_file src/rtl/crt_filter.v
add_file src/rtl/GA40010/ga40010.sv
add_file src/rtl/GA40010/casgen.v
add_file src/rtl/GA40010/casgen_sync.v
add_file src/rtl/GA40010/syncgen.v
add_file src/rtl/GA40010/syncgen_sync.v
add_file src/rtl/GA40010/rslatch.v
add_file src/rtl/GA40010/video.sv
add_file src/rtl/T80/T80_Pack.vhd
add_file src/rtl/T80/T80_ALU.vhd
add_file src/rtl/T80/T80_Reg.vhd
add_file src/rtl/T80/T80_MCode.vhd
add_file src/rtl/T80/T80.vhd
add_file src/rtl/T80/T80pa.vhd
add_file src/rtl/u765/u765.sv
add_file src/pll/gowin_rpll/pll_160m.v
add_file src/pll/gowin_rpll/flash_pll.v
add_file src/pll/gowin_clkdiv/gowin_clkdiv.v

# Pipeline vidéo HDMI
add_file src/video/scandoubler.v
add_file src/video/video_analyzer.v
add_file src/video/video2hdmi.v
add_file src/video/hdmi.sv
add_file src/video/tmds_channel.sv
add_file src/video/serializer.sv
add_file src/video/packet_assembler.sv
add_file src/video/packet_picker.sv
add_file src/video/audio_clock_regeneration_packet.sv
add_file src/video/audio_info_frame.sv
add_file src/video/audio_sample_packet.sv
add_file src/video/auxiliary_video_information_info_frame.sv
add_file src/video/source_product_description_info_frame.sv

# Contraintes
add_file constraint/TangCPC.cst
add_file constraint/TangCPC.sdc

# Options de synthèse
set_option -synthesis_tool gowinsynthesis
set_option -output_base_name TangCPC
set_option -top_module top
set_option -verilog_std sysv2017
set_option -vhdl_std vhd2008
set_option -print_all_synthesis_warning 1

# Autoriser l'utilisation des pins dédiés comme GPIO
# (flash SPI = MSPI, SPI secondaire = SSPI, JTAG comme GPIO)
set_option -use_mspi_as_gpio 1
set_option -use_sspi_as_gpio 1
set_option -use_jtag_as_gpio 1

# Lancement
run all

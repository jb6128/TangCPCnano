// tb_cpc_romload.v -- ROM loader test with stub SD and stub SDRAM (not run here:
// no simulator in the authoring sandbox).  Run:
//   iverilog -g2012 -o tb_rom tb/tb_cpc_romload.v src/core/cpc_romload.v && vvp tb_rom
`timescale 1ns/1ps
module tb_cpc_romload;
    reg clk = 0; always #15.6 clk = ~clk;
    reg reset = 1, mounted = 0, clear = 0, boot_idle = 1;
    reg [31:0] size = 0; reg [4:0] slot = 0;
    wire sd_rd; wire [31:0] sd_lba; reg sd_busy = 0;
    reg [8:0] baddr = 0; reg [7:0] bdout = 0; reg bwr = 0;
    wire [22:0] ram_addr; wire [7:0] ram_din; wire ram_req; reg ram_ack = 0;
    wire [255:0] rom_map; wire busy;

    cpc_romload dut (.clk(clk), .reset(reset), .img_mounted(mounted), .img_size(size),
        .slot(slot), .clear(clear), .amsdos_en(1'b1), .boot_idle(boot_idle), .sd_rd(sd_rd), .sd_lba(sd_lba),
        .sd_busy(sd_busy), .sd_buff_addr(baddr), .sd_buff_dout(bdout), .sd_buff_wr(bwr),
        .ram_addr(ram_addr), .ram_din(ram_din), .ram_req(ram_req), .ram_ack(ram_ack),
        .rom_map(rom_map), .busy(busy));

    reg [7:0] sdram [0:8388607];
    integer hdr = 0;                                   // 0 or 128 byte file header
    function [7:0] fbyte(input integer f); fbyte = (f*5 + 1) & 8'hFF; endfunction

    // SD stub (file byte f = hdr-adjusted index)
    integer k; reg [31:0] lba_l;
    always @(posedge clk) if (sd_rd && !sd_busy) begin
        lba_l = sd_lba; sd_busy <= 1;
        for (k = 0; k < 512; k = k + 1) begin
            @(posedge clk); baddr <= k; bdout <= fbyte(lba_l*512 + k); bwr <= 1;
        end
        @(posedge clk); bwr <= 0; repeat (10) @(posedge clk); sd_busy <= 0;
    end
    // SDRAM stub: ack 3 cycles after req
    always @(posedge clk) begin
        ram_ack <= 0;
        if (ram_req) begin sdram[ram_addr] <= ram_din; repeat (3) @(posedge clk); ram_ack <= 1; end
    end

    integer i, errs = 0;
    task load(input [4:0] s, input integer sz, input integer h);
        begin hdr = h; slot = s; size = sz;
            @(posedge clk); mounted = 1; @(posedge clk); mounted = 0;
            repeat (20) @(posedge clk);
            wait (!busy); repeat (5) @(posedge clk); end
    endtask

    initial begin
        repeat (5) @(posedge clk); reset = 0;
        if (rom_map[31:0] !== 32'h0000_0081) begin errs = errs + 1; $display("FAIL: reset map %h", rom_map[31:0]); end

        load(5'd9, 16384, 0);                              // plain 16 KB ROM into slot 9
        if (!rom_map[9]) begin errs = errs + 1; $display("FAIL: slot 9 not set"); end
        for (i = 0; i < 16384; i = i + 1)
            if (sdram[23'h400000 + 9*16384 + i] !== fbyte(i)) begin errs = errs + 1;
                if (errs < 10) $display("FAIL slot9 byte %0d", i); end

        load(5'd12, 16512, 128);                           // with 128 byte AMSDOS header
        for (i = 0; i < 16384; i = i + 1)
            if (sdram[23'h400000 + 12*16384 + i] !== fbyte(i + 128)) begin errs = errs + 1;
                if (errs < 10) $display("FAIL slot12 byte %0d", i); end

        load(5'd9, 0, 0);                                  // empty selection ejects
        if (rom_map[9]) begin errs = errs + 1; $display("FAIL: slot 9 not cleared"); end
        slot = 5'd12; @(posedge clk); clear = 1; repeat (3) @(posedge clk); clear = 0; @(posedge clk);
        if (rom_map[12]) begin errs = errs + 1; $display("FAIL: clear button"); end
        slot = 5'd7; @(posedge clk); clear = 1; repeat (3) @(posedge clk); clear = 0; @(posedge clk);
        if (!rom_map[7]) begin errs = errs + 1; $display("FAIL: AMSDOS slot must be protected"); end

        $display(errs ? "tb_cpc_romload: %0d ERRORS" : "tb_cpc_romload: PASS", errs);
        $finish;
    end
endmodule

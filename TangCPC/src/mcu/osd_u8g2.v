/*
    osd_u8g2.v
 
    on-screen-display using a memory layout that matches the 
    one of 128x64 OLED displays and is thus supported by u8g2
  */

module osd_u8g2 (
  input        clk,
  input        reset,

  input        data_in_strobe,
  input        data_in_start,
  input [7:0]  data_in,
	    
  input        hs,
  input        vs, 
  // Grid mode (grid_en = 1): the OSD position is derived from the HDMI column / row counters (gx, gy) instead of
  // the CPC sync (hs / vs), so it stays at a fixed place on the HDMI screen however a program moves HSync / VSync
  // or changes the line / frame length. gx_off shifts the horizontal centre (512 - window centre column).
  input        grid_en,
  input [9:0]  gx,
  input [9:0]  gy,
  input [9:0]  gx_off,
  input [5:0]  r_in,
  input [5:0]  g_in,
  input [5:0]  b_in,

  output [5:0] r_out,
  output [5:0] g_out,
  output [5:0] b_out
);

// OSD is enabled and visible
reg enabled;

// -------------------------- OSD painting -------------------------------

reg vsD, hsD;
reg [11:0] hcnt;    // signal ranges 0..1023
reg [11:0] hcntL;
reg [9:0] vcnt;    // signal ranges 0..626
reg [9:0] vcntL;

// OSD is active on current pixel, the shadow is active or the text area is active
wire active, sactive, tactive;
   
// draw active osd, add some shadow to those parts outside osd
// that are covered by shadow
assign r_out = !enabled?r_in:active?osd_r:sactive?{1'b0, r_in[5:1]}:r_in;
assign g_out = !enabled?g_in:active?osd_g:sactive?{1'b0, g_in[5:1]}:g_in;
assign b_out = !enabled?b_in:active?osd_b:sactive?{1'b0, b_in[5:1]}:b_in;   

wire	   osd_pix;  
wire [5:0] osd_pix_col;

// background is darker where "shadow" is active
wire [5:0] osd_r = (tactive && osd_pix)?osd_pix_col:sactive?{4'b0000, r_in[5:4]}:{3'b000, r_in[5:3]};
wire [5:0] osd_g = (tactive && osd_pix)?osd_pix_col:sactive?{4'b0100, g_in[5:4]}:{3'b010, g_in[5:3]};
wire [5:0] osd_b = (tactive && osd_pix)?osd_pix_col:sactive?{4'b0000, b_in[5:4]}:{3'b000, b_in[5:3]};  
   
`define BORDER 2
`define SHADOW 4
`define SCALE  2
`define WIDTH 16   // OSD width in characters
`define HEIGHT 8   // OSD height in characters

// position counters: from the CPC sync (measured below) or, in grid mode, from the HDMI counters
wire [10:0] gxs   = {1'b0, gx} + {1'b0, gx_off};            // wraps at 1024 (HDMI row = 1024 clocks)
wire [10:0] gys   = {1'b0, gy} + 11'd24;                    // HDMI frame = 624 rows, window centre row 288 -> 312
wire [9:0]  vc_g  = (gys >= 11'd624) ? (gys - 11'd624) : gys[9:0];
wire [11:0] hcnt_u  = grid_en ? {2'b00, gxs[9:0]} : hcnt;
wire [11:0] hcntL_u = grid_en ? 12'd1024 : hcntL;
wire [9:0]  vcnt_u  = grid_en ? vc_g : vcnt;
wire [9:0]  vcntL_u = grid_en ? 10'd624 : vcntL;

wire [11:0] hstart = (hcntL_u/2)-8*`WIDTH*`SCALE/2;
wire [9:0]  vstart = (vcntL_u/2)-8*`HEIGHT*`SCALE/2;

// entire OSD area incl border
wire	    hactive = hcnt_u >= hstart-`SCALE*`BORDER && hcnt_u < hstart+`SCALE*`BORDER+8*`WIDTH*`SCALE;
wire	    vactive = vcnt_u >= vstart-`SCALE*`BORDER && vcnt_u < vstart+`SCALE*`BORDER+8*`HEIGHT*`SCALE;
assign      active = hactive && vactive;  

// text area of OSD
wire	    thactive = hcnt_u >= hstart && hcnt_u < hstart+8*`WIDTH*`SCALE;
wire	    tvactive = vcnt_u >= vstart && vcnt_u < vstart+8*`HEIGHT*`SCALE;
assign 	    tactive = thactive && tvactive;  

// shadow area of OSD
wire	    shactive = hcnt_u >= hstart-`SCALE*`BORDER+`SCALE*`SHADOW && hcnt_u < hstart+`SCALE*`BORDER+`SCALE*`SHADOW+8*`WIDTH*`SCALE;
wire	    svactive = vcnt_u >= vstart-`SCALE*`BORDER+`SCALE*`SHADOW && vcnt_u < vstart+`SCALE*`BORDER+`SCALE*`SHADOW+8*`HEIGHT*`SCALE;
assign	    sactive = shactive && svactive;  

// 1024 bytes = 8192 pixels = 128 x 64 pixels
reg [7:0] buffer [1024];  

// external data interface to write to buffer
reg [9:0] data_cnt;
reg [7:0] command;
reg data_addr_state;
   
always @(posedge clk) begin
    if(reset) begin
        enabled <= 1'b0;

    end else begin

      if(data_in_strobe) begin
        if(data_in_start) begin
            command <= data_in;
            data_addr_state <= 1'b1;
            data_cnt <= 10'd0;
        end else begin
            data_addr_state <= 1'b0;

            // OSD command 1: enabled (show) or disable (hide) OSD
            if((command == 8'd1) && data_addr_state)
                enabled <= data_in[0];   // en/disable

            // OSD command 2: display data for give tile
            if(command == 8'd2) begin
                if(data_addr_state)
                    data_cnt <= { data_in[6:0], 3'b000 };
                else begin	 
                    buffer[data_cnt] <= data_in;
                    data_cnt <= data_cnt + 10'd1;
                end
            end
         end
      end
   end
end
   
wire [7:0] hpix  = hcnt_u-hstart;  // horizontal pixel position inside OSD   
wire [7:0] hpixD = hpix+1;       // latch byte one pixel in advance
wire [6:0] vpix  = vcnt_u-vstart;  // vertical pixel position inside OSD   

assign osd_pix = buffer_byte[vpix[3:1]];

reg [7:0] buffer_byte;
always @(posedge clk)
   buffer_byte <= buffer[{ vpix[6:4], hpixD[7:1] }];
   
assign osd_pix_col = 6'd63;

// -------------------------- video signal analysis -------------------------
   
// analyze video timing to determine center of screen

always @(posedge clk) begin
   // ---- hsync processing -----
   hsD <= hs;

   // end of hsync, rising edge
   if(hs && !hsD) begin
      hcntL <= hcnt;
      hcnt <= 0;
   end else
     hcnt <= hcnt + 12'd1;
   
   if(hs && !hsD) begin
      // ---- vsync processing -----
      vsD <= vs;
      // begin of vsync, falling edge
      if(!vs && vsD) begin
         vcntL <= vcnt;
         vcnt <= 0;
      end else
        vcnt <= vcnt + 10'd1;
   end
end
   
endmodule

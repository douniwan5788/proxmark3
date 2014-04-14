`timescale 1ns/1ns

// constants for the different modes:
`define SNIFFER     3'b000
`define TAGSIM_LISTEN 3'b001
`define TAGSIM_MOD    3'b010
`define READER_LISTEN 3'b011
`define READER_MOD    3'b100

`define TAGSIM_MOD2   3'b101

module hi_iso14443a_tb  ;

  reg    ssp_dout   ;
  reg    cross_hi   ;
  wire    pwr_lo   ;
  reg  [7:0]  adc_d   ;
  reg    ck_1356meg   ;
  wire    pwr_oe1   ;
  reg    pck0   ;
  wire    pwr_oe2   ;
  wire    pwr_oe3   ;
  wire    ssp_din   ;
  wire    pwr_oe4   ;
  wire  ssp_frame   ;
  reg  [2:0]  mod_type   ;
  wire    pwr_hi   ;
  wire    adc_clk   ;
  reg    cross_lo   ;
  wire    dbg   ;
  wire  ssp_clk   ;
  reg    ck_1356megb   ;

  hi_iso14443a dut  (
       .ssp_dout (ssp_dout ) ,
      .cross_hi (cross_hi ) ,
      .pwr_lo (pwr_lo ) ,
      .adc_d (adc_d ) ,
      .ck_1356meg (ck_1356meg ) ,
      .pwr_oe1 (pwr_oe1 ) ,
      .pck0 (pck0 ) ,
      .pwr_oe2 (pwr_oe2 ) ,
      .pwr_oe3 (pwr_oe3 ) ,
      .ssp_din (ssp_din ) ,
      .pwr_oe4 (pwr_oe4 ) ,
      .ssp_frame (ssp_frame ) ,
      .mod_type (mod_type ) ,
      .pwr_hi (pwr_hi ) ,
      .adc_clk (adc_clk ) ,
      .cross_lo (cross_lo ) ,
      .dbg (dbg ) ,
      .ssp_clk (ssp_clk ) ,
      .ck_1356megb (ck_1356megb ) );

  // main clock
  always #1 begin
    ck_1356megb = !ck_1356megb;
    ck_1356meg = ck_1356megb;
  end

  reg adc;
  always #2 begin
    adc = !adc;
    // adc_d = adc?{$random}%192 +16 : {$random}%7;
    adc_d = adc?{$random}%20 + 192 : 0;  //deep_modultion
  end

  reg [4:0] out_counter;
  always @(posedge ssp_clk) begin
    out_counter = out_counter +1;
    ssp_dout = out_counter[4] ? 1'd1:1'd0;
  end

  initial begin
    //init
    ck_1356megb = 0;
    adc = 0;
    ssp_dout = 0;
    out_counter = 0;

    mod_type = `TAGSIM_LISTEN; //TAGSIM_LISTEN
    #32 mod_type = `TAGSIM_MOD; //TAGSIM_MOD
  end

endmodule


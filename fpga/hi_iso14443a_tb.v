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

  // reg [2:0] out_counter;
  reg [4:0] out_counter;
  reg [7:0] rx;
  reg issend;
  reg [7:0] send_buf;
  reg start;

// sample data at rising edge of ssp_clk - ssp_dout changes at the falling edge.
  always @(negedge ssp_clk) begin
    if(start) begin
      out_counter <= out_counter - 1;

      if(issend) begin
      ssp_dout <= send_buf[out_counter];
      // ssp_dout <= 1;
      // ssp_dout <= out_counter[0];
      end
      else begin
      // ssp_dout <= out_counter[4];
      ssp_dout <= 0;
      end

      rx[7:1] <= rx[6:0];
      rx[0] <= ssp_din;
    end
  end

  always @(posedge ssp_frame)
  begin
    if(start) begin
      if(rx != 8'd0)
      begin
        issend <= 1;
        out_counter <= 7;
        // if(send_buf >= 8'd10) begin
        //   send_buf <= 8'hff;
        // end
        // else
          send_buf <= send_buf +1;
      end
      else
      begin
        issend <= 0;
      end
    end
  end

  initial begin
    //init
    ck_1356megb = 0;
    adc = 0;
    ssp_dout = 0;
    out_counter = 0;
    issend = 0;
    rx=0;
    start = 0;

    send_buf = 8'h01;

    mod_type = `TAGSIM_LISTEN; //TAGSIM_LISTEN
    #32 mod_type = `TAGSIM_MOD; //TAGSIM_MOD
    start = 1;
  end

endmodule


module frame_counter
  #(
     parameter  HMAX = 640,   // max horizontal count
     VMAX = 480    // max vertical count
   )
   (
     input logic clk,
     input logic reset,
     input logic inc,
     input logic sync_clr,
     output logic [10:0] hcount,
     output logic [10:0] vcount,
     output logic        frame_start,
     output logic        frame_end
   )

   // signal declarations
   logic [10:0] hcount_reg, hcount_next;
  logic [10:0] vcount_reg, vcount_next;

  // horizontal and vertical pixel counters
  // register
  always_ff @(posedge clk, posedge reset)
    if (reset)
    begin
      hcount_reg <= 0;
      vcount_reg <= 0;
    end
    else if (sync_clr)
    begin
      hcount_reg <= 0;
      vcount_reg <= 0;
    end
    else
    begin
      hcount_reg <= hcount_next;
      vcount_reg <= vcount_next;
    end
  // next-state logic of horizontal counter
  always_comb
    if (inc)
      if (hcount_reg == (HMAX-1))
        vcount_next = 0;
      else
        vcount_next = vcount_reg + 1;
    else
      vcount_next = vcount_reg;

  // next-state logic of vertical counter
  always_comb
    if (inc && (hcount_reg == (HMAX-1)))
      if (vcount_reg == (VMAX-1))
        vcount_next = 0;
      else
        vcount_next = vcount_reg + 1;
    else
      vcount_next = vcount_reg;
  // output
  assign hcount = hcount_reg;
  assign vcount = vcount_reg;
  assign frame_start = (vcount_reg==0) && (hcount_reg==0);
  assign frame_end = (vcount_reg==(VMAX-1)) && (hcount_reg ==(HMAX-1));
endmodule

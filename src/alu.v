module alu(sub,a,b,bus_out,carry_flag,zero_flag);
  input sub;
  input [7:0] a,b;
  output [7:0] bus_out;
  output carry_flag ,zero_flag;
  wire [8:0] w;
  assign w = (sub)?{1'b0,a}-{1'b0,b}:{1'b0,a}+{1'b0,b};
  assign bus_out = w[7:0] ;
  assign carry_flag = w[8];
  assign zero_flag = ~w[7]&~w[6]&~w[5]&~w[4]&~w[3]&~w[2]&~w[1]&~w[0];
endmodule

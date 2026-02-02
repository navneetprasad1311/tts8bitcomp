module flag_reg(clk,reset,flag_en,cin,zin,cout,zout);
  input clk,reset,flag_en,cin,zin ;
  output reg cout,zout;
  always @(negedge clk or posedge reset)begin
    if(reset)begin
      cout<=1'b0;
      zout<=1'b0;
    end
    else if(flag_en)begin 
      cout<=cin;
      zout<=zin;
    end
  end
endmodule

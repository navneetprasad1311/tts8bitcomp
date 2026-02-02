module ctrl_unit(
	input start,
	input clk,
	input reset,
	input [3:0] opcode,
	input carry_flag,
	input zero_flag,
	output [15:0] out);

localparam FE   = 15;   //Flag Enable	
localparam HLT  = 14; 	//HALT
localparam MI	= 13; 	//MAR in
localparam RI	= 12; 	//RAM in
localparam RO	= 11; 	//RAM out
localparam II 	= 10; 	//IR in
localparam IO 	= 9; 	//IR out 
localparam AI	= 8; 	//A register in
localparam AO 	= 7; 	//A register out 
localparam ALO	= 6; 	//ALU out
localparam SUB	= 5; 	//Addition / Subtraction 
localparam BI	= 4; 	//B register in
localparam OI	= 3; 	//Output register in
localparam CE	= 2; 	//Program Counter Enable
localparam CL	= 1; 	//Program Counter in
localparam CO	= 0; 	//Program Counter out


localparam OP_NOP = 4'b0000; //No-Operation
localparam OP_LDA = 4'b0001; //Load
localparam OP_STA = 4'b0100; //Store
localparam OP_LDI = 4'b0101; //Load Immediate
localparam OP_ADD = 4'b0010; //Addition
localparam OP_SUB = 4'b0011; //Subtraction
localparam OP_JMP = 4'b0110; //Jump
localparam OP_JC  = 4'b0111; //Jump if Carry
localparam OP_JZ  = 4'b1000; //Jump if Zero
localparam OP_OUT = 4'b1110; //Output
localparam OP_HLT = 4'b1111; //Halt

reg [15:0] ctrl_wd;
reg [2:0] stage;
reg running = 1'b0;
reg halt = 1'b0;

always @(posedge clk or posedge reset) begin
	if (reset) begin
		stage <= 0;
		running <= 0;
		halt <= 0;
	end 
	else if (start && !running) begin
		running <= 1;
		stage   <= 0;
	end 
	else if (running && !halt) begin
		if (ctrl_wd[HLT]) begin
			running <= 0;
			halt <= 1;
			stage <= stage;
		end
		else if (stage == 5) begin
			stage <= 0;
		end
		else begin
			stage <= stage + 1;
		end
	end
end

	
always @(*) begin
	ctrl_wd = 16'b0;

	case (stage)
		0: begin
			ctrl_wd[CO] = 1;
			ctrl_wd[MI] = 1;
		end
		1: begin
			ctrl_wd[RO] = 1;
			ctrl_wd[II] = 1; 
		end
		2: begin
		    ctrl_wd[CE] = 1;  
		end
		3: begin
			case (opcode)
				OP_LDA: begin
					ctrl_wd[IO] = 1;
					ctrl_wd[MI] = 1;
				end
				OP_STA: begin
					ctrl_wd[IO] = 1;
					ctrl_wd[MI] = 1;
				end
				OP_LDI: begin
					ctrl_wd[IO] = 1;
					ctrl_wd[AI] = 1;
				end
				OP_ADD: begin
					ctrl_wd[IO] = 1;
					ctrl_wd[MI] = 1;
				end
				OP_SUB: begin
					ctrl_wd[IO] = 1;
					ctrl_wd[MI] = 1;
				end
				OP_JMP: begin
					ctrl_wd[IO] = 1;
					ctrl_wd[CL] = 1;
				end
				OP_JC: begin
					if(carry_flag) begin
						ctrl_wd[IO] = 1;
						ctrl_wd[CL] = 1;
					end
				end
				OP_JZ: begin
					if(zero_flag) begin
						ctrl_wd[IO] = 1;
						ctrl_wd[CL] = 1;
					end
				end
				OP_OUT: begin
					ctrl_wd[AO] = 1;
					ctrl_wd[OI] = 1;
				end
				OP_HLT: begin
					ctrl_wd[HLT] = 1;
				end
			endcase
		end
		4: begin
			case (opcode)
				OP_LDA: begin
					ctrl_wd[RO] = 1;
					ctrl_wd[AI] = 1;
				end
				OP_STA: begin
					ctrl_wd[AO] = 1;
					ctrl_wd[RI] = 1;
				end
				OP_LDI: ctrl_wd[FE] = 1;  // Flag register Enable
				OP_ADD: begin
					ctrl_wd[RO] = 1;
					ctrl_wd[BI] = 1;
				end
				OP_SUB: begin
					ctrl_wd[RO] = 1;
					ctrl_wd[BI] = 1;
				end
			endcase
		end
		5: begin
			case (opcode)
				OP_LDA: ctrl_wd[FE] = 1;  // Flag register Enable 
				OP_ADD: begin
					ctrl_wd[ALO] = 1;
					ctrl_wd[AI] = 1;
					ctrl_wd[FE] = 1;  // Flag register Enable
				end
				OP_SUB: begin
					ctrl_wd[SUB] = 1;
					ctrl_wd[ALO] = 1;
					ctrl_wd[AI] = 1;
					ctrl_wd[FE] = 1;  // Flag register Enable
				end
			endcase
		end
	endcase
end

assign out = ctrl_wd;

endmodule

module tt_um_cpu_top_np (
    input clk,
    input reset,
    input RAM_rst,
    input start,

    input [3:0] sw_addr,
    input [7:0] enter_data,
    input load_btn,
    input inp_loaded,
    output inp_req,
    output [3:0] pc_disp,//PC output
    output [7:0] Out_display;

    output a_h, b_h, c_h, d_h, e_h, f_h, g_h,//h-hundred's place
    output a_t, b_t, c_t, d_t, e_t, f_t, g_t,//t-ten's place
    output a_o, b_o, c_o, d_o, e_o, f_o, g_o//o-one's place
);

    // ============================
    // Interconnect wires
    // ============================
    reg [7:0] sw_data;
    reg [7:0] inp_data;
    wire [3:0] ram_addr;
    wire [7:0] ram_data_in;
    wire [7:0] ram_data_out;
    wire ram_in_en;
    wire inp_sig;
    wire [7:0] out_display;
    
    // ============================
    // Clock Divider 100MHz to 6 Hz 
    // ============================
    assign Out_display = out_display;
    assign inp_req = inp_sig;
    always @(*)begin
        inp_data = 8'b0;
        sw_data = 8'b0;
        if(inp_sig)
            inp_data = enter_data;
        else 
            sw_data = enter_data;
    end
    // ============================
    // Instantiate CPU core
    // ============================

    cpu_core CPU (
        .clk(clk),
        .reset(reset),
        .start(start),
        .pc_disp(pc_disp),
        .out_display(out_display),
        .inp_req(inp_sig),   
        .inp_loaded(inp_loaded), // inp by me 
        .inp_data(inp_data),   //inp reg data
        .ram_data_in(ram_data_in),
        .ram_data_out(ram_data_out),
        .ram_addr(ram_addr),
        .ram_in_en(ram_in_en)
    );

    // ============================
    // Instantiate RAM (external)
    // ============================

    RAM RAM_inst (
        .clk(clk),
        .reset(RAM_rst),
        .ram_in_en(ram_in_en),
        .ram_in_addr(ram_addr),
        .bus_in(ram_data_out),
        .bus_out(ram_data_in),

        .start(start),
        .sw_addr(sw_addr),
        .sw_data(sw_data),
        .load_btn(load_btn)
    );
    
    // ============================
    // Instantiate external display
    // ============================

    binary_to_7segment_display Display (
    	.bin(out_display),
    	.a_h(a_h), .b_h(b_h), .c_h(c_h), .d_h(d_h), .e_h(e_h), .f_h(f_h), .g_h(g_h),
    	.a_t(a_t), .b_t(b_t), .c_t(c_t), .d_t(d_t), .e_t(e_t), .f_t(f_t), .g_t(g_t),
   	    .a_o(a_o), .b_o(b_o), .c_o(c_o), .d_o(d_o), .e_o(e_o), .f_o(f_o), .g_o(g_o)
    );
endmodule

module cpu_core(
    input clk,
    input reset,
    input start,
    input inp_loaded,
    input [7:0] inp_data,
    output inp_req,
    output [3:0] pc_disp,
    output [7:0] out_display,
    input  [7:0] ram_data_in,
    output [3:0] ram_addr,
    output ram_in_en,
    output [7:0] ram_data_out
);

    // ============================
    // Internal signals and wires
    // ============================

    wire [18:0] ctrl;
    wire INP = ctrl[18];
    wire ANA = ctrl[17];
    wire XRA = ctrl[16];
    wire FE  = ctrl[15];
    wire HLT = ctrl[14];
    wire MI  = ctrl[13];
    wire RI  = ctrl[12];
    wire RO  = ctrl[11];
    wire II  = ctrl[10];
    wire IO  = ctrl[9];
    wire AI  = ctrl[8];
    wire AO  = ctrl[7];
    wire ALO = ctrl[6];
    wire SUB = ctrl[5];
    wire BI  = ctrl[4];
    wire OI  = ctrl[3];
    wire CE  = ctrl[2];
    wire CL  = ctrl[1];
    wire CO  = ctrl[0];

    // ============================
    // Datapath internal wires
    // ============================
    wire [7:0] alu_out;
    wire carry_flag,cout;
    wire zero_flag,zout;
    wire [7:0] a_alu_in, b_alu_in ,data_out;
    wire [3:0] mar_addr;
    wire [7:0] ram_bus_io;
    wire [3:0] ir_bus_out;
    wire [3:0] opcode;
    wire [7:0] a_bus_out;
    wire [3:0] pc_bus_out;
    wire [7:0] pc_bus_ext = {4'b0000, pc_bus_out};
    reg [7:0] bus_mux;

    // ============================
    // Datapath external wires
    // ============================    
    
    assign ram_in_en  = RI;
    assign ram_addr   = mar_addr;
    assign ram_data_out = a_bus_out;
    // ============================
    // INPUT Register
    // ============================
    
    INP_reg INP_inst(
        .clk(clk),
        .reset(reset),
        .inp_loaded(inp_loaded),
        .inp_data(inp_data),
        .data_out(data_out)
    );

    // ============================
    // Program Counter
    // ============================

    PC pc_inst (
        .clk(clk),
        .reset(reset),
        .pc_en(CE),
        .jmp(CL),          
        .bus_in(bus_mux[3:0]),
        .dis_out(pc_disp),
        .bus_out(pc_bus_out)
    );

    // ============================
    // MAR Register
    // ============================

    MAR_reg mar_inst (
        .clk(clk),
        .reset(reset),
        .mar_in(MI),
        .bus_in(bus_mux[3:0]),
        .ram_in(mar_addr)
    );   

    // ============================
    // Instruction Register
    // ============================

    reg_IR ir_inst (
       .clk(clk),
       .reset(reset),
       .ir_in(II), 
       .bus_in(bus_mux),
       .bus_out(ir_bus_out),
       .cs_in(opcode)
    );

    // ============================
    // Register A
    // ============================

    reg_a regA (
        .clk(clk),
        .reset(reset),
        .a_in(AI),  
        .bus_in(bus_mux),
        .bus_out(a_bus_out),
        .alu_in(a_alu_in)
    );

    // ============================
    // Register B
    // ============================

    reg_b regB (
        .clk(clk),
        .reset(reset),
        .b_in(BI),
        .bus_in(bus_mux),
        .alu_in(b_alu_in)
    );

    // ============================
    // ALU
    // ============================

    alu alu_inst (
        .sub(SUB),
        .xra(XRA),
        .ana(ANA),
        .a(a_alu_in),
        .b(b_alu_in),
        .bus_out(alu_out),
        .carry_flag(carry_flag),
        .zero_flag(zero_flag)
    );

    // ============================
    // Flag Register
    // ============================
    
    flag_reg fr_inst (
        .clk(clk),
        .reset(reset),
        .flag_en(FE),
        .cin(carry_flag),
        .zin(zero_flag),
        .cout(cout),
        .zout(zout)
        );
    
    // ============================
    // Output Register
    // ============================

    reg_OUT out_reg (
        .clk(clk),
        .out_in(OI),
        .reset(reset),
        .bus_in(bus_mux),
        .dis_out(out_display)
    );

    // ============================
    // Control Unit
    // ============================
    
    ctrl_unit cu_inst (
        .start(start),
        .clk(clk),
        .reset(reset),
        .inp_loaded(inp_loaded),
        .opcode(opcode),
        .carry_flag(cout),
        .zero_flag(zout),
        .inp_req(inp_req),
        .out(ctrl)//
    );

    // ============================
    // Bus Multiplexer (central bus)
    // ============================

    always @(*) begin   
        if (CO) begin  
            bus_mux = pc_bus_ext;
        end else if (IO) begin
            bus_mux = ir_bus_out;
        end else if (ALO) begin
            bus_mux = alu_out;
        end else if (AO) begin
            bus_mux = a_bus_out;
        end else if (RO) begin
            bus_mux = ram_data_in;
        end else if (INP) begin
            bus_mux = data_out;
        end else begin
            bus_mux = 8'b0;
        end 
    end
endmodule

module reg_a(
    input clk,
    input reset,
    input a_in,
    input [7:0] bus_in,
    output [7:0] alu_in,
    output [7:0] bus_out
);
    reg [7:0] reg_a;
    always @(posedge clk ) begin
        if (reset)
            reg_a <= 8'b0;
        else if (a_in)
            reg_a <= bus_in;
    end
    assign alu_in = reg_a;
    assign bus_out = reg_a;
endmodule

module alu(sub,xra,ana,a,b,bus_out,carry_flag,zero_flag);
  input sub,xra,ana;
  input [7:0] a,b;
  output [7:0] bus_out;
  output carry_flag ,zero_flag;
  reg [8:0] w;
  always @(*)begin
    case ({sub,xra,ana})
      3'b000: w = {1'b0,a}+{1'b0,b};
      3'b100: w = {1'b0,a}-{1'b0,b};
      3'b010: w = {1'b0,a^b};
      3'b001: w = {1'b0,a&b};
      default: w = 9'b0;
    endcase
  end
  assign bus_out    = w[7:0] ;
  assign carry_flag = w[8];
  assign zero_flag  = (w[7:0] == 8'b0);
endmodule

module reg_b(
    input clk,
    input reset,
    input b_in,
    input [7:0] bus_in,
    output [7:0] alu_in
);
    reg [7:0] reg_b;
    always @(posedge clk ) begin
        if (reset)
            reg_b <= 8'b0;
        else if (b_in)
            reg_b <= bus_in;
    end
    assign alu_in = reg_b;
endmodule

module ctrl_unit(
    input start,
    input clk,
    input reset,
    input inp_loaded,
    input [3:0] opcode,
    input carry_flag,
    input zero_flag,
    output reg inp_req,
    output [18:0] out
);
localparam INP=18,ANA=17,XRA=16,FE=15,HLT=14,MI=13,RI=12,RO=11,II=10;
localparam IO=9,AI=8,AO=7,ALO=6,SUB=5,BI=4,OI=3,CE=2,CL=1,CO=0;
localparam OP_NOP=4'b0000,OP_LDA=4'b0001,OP_STA=4'b0100,OP_LDI=4'b0101,OP_ADD=4'b0010,OP_SUB=4'b0011,OP_JMP=4'b0110,OP_JC=4'b0111;
localparam OP_JZ=4'b1000,OP_ADI=4'b1001,OP_SUI=4'b1010,OP_XRA=4'b1011,OP_ANA=4'b1100,OP_INP=4'b1101,OP_OUT=4'b1110,OP_HLT=4'b1111;
reg [18:0] ctrl_wd;
reg [2:0] stage;
reg running;
reg halt;
reg inp_wait;
always @(posedge clk ) begin
    if(reset) begin
        stage<=0;
        running<=0;
        halt<=0;
        inp_req<=0;
        inp_wait<=0;
    end
    else if(start && !running && !halt) begin
        running<=1;
        stage<=0;
    end
    else if(inp_wait) begin
        inp_req<=1;
        if(inp_loaded) begin
            inp_wait<=0;
            inp_req<=0;
            running<=1;
            stage<=stage+1;
        end
    end
    else if(running && !halt) begin
        if(opcode==OP_INP && stage==3) begin
            inp_wait<=1;
            running<=0;
        end
        else if(ctrl_wd[HLT]) begin
            running<=0;
            halt<=1;
        end
        else if(stage==5) stage<=0;
        else stage<=stage+1;
    end
end
always @(*) begin
    ctrl_wd=19'b0;
    case(stage)
        0: begin
            ctrl_wd[CO]=1;
            ctrl_wd[MI]=1;
        end
        1: begin
            ctrl_wd[RO]=1;
            ctrl_wd[II]=1;
        end
        2: begin
            ctrl_wd[CE]=1;
        end
        3: begin
            case(opcode)
                OP_LDA,OP_STA,OP_ADD,OP_SUB,OP_XRA,OP_ANA: begin
                    ctrl_wd[IO]=1;
                    ctrl_wd[MI]=1;
                end
                OP_LDI: begin
                    ctrl_wd[IO]=1;
                    ctrl_wd[AI]=1;
                end
                OP_JMP: begin
                    ctrl_wd[IO]=1;
                    ctrl_wd[CL]=1;
                end
                OP_JC: if(carry_flag) begin
                    ctrl_wd[IO]=1;
                    ctrl_wd[CL]=1;
                end
                OP_JZ: if(zero_flag) begin
                    ctrl_wd[IO]=1;
                    ctrl_wd[CL]=1;
                end
                OP_ADI,OP_SUI: begin
                    ctrl_wd[IO]=1;
                    ctrl_wd[BI]=1;
                end
                OP_OUT: begin
                    ctrl_wd[AO]=1;
                    ctrl_wd[OI]=1;
                end
                OP_HLT: ctrl_wd[HLT]=1;
            endcase
        end
        4: begin
            case(opcode)
                OP_LDA: begin
                    ctrl_wd[RO]=1;
                    ctrl_wd[AI]=1;
                end
                OP_STA: begin
                    ctrl_wd[AO]=1;
                    ctrl_wd[RI]=1;
                end
                OP_ADD,OP_SUB,OP_XRA,OP_ANA: begin
                    ctrl_wd[RO]=1;
                    ctrl_wd[BI]=1;
                end
                OP_ADI: begin
                    ctrl_wd[ALO]=1;
                    ctrl_wd[FE]=1;
                    ctrl_wd[AI]=1;
                end
                OP_SUI: begin
                    ctrl_wd[SUB]=1;
                    ctrl_wd[ALO]=1;
                    ctrl_wd[FE]=1;
                    ctrl_wd[AI]=1;
                end
                OP_INP: begin
                    ctrl_wd[INP]=1;
                    ctrl_wd[AI]=1;
                end
            endcase
        end
        5: begin
            case(opcode)
                OP_ADD: begin
                    ctrl_wd[ALO]=1;
                    ctrl_wd[FE]=1;
                    ctrl_wd[AI]=1;
                end
                OP_SUB: begin
                    ctrl_wd[SUB]=1;
                    ctrl_wd[ALO]=1;
                    ctrl_wd[FE]=1;
                    ctrl_wd[AI]=1;
                end
                OP_XRA: begin
                    ctrl_wd[XRA]=1;
                    ctrl_wd[ALO]=1;
                    ctrl_wd[FE]=1;
                    ctrl_wd[AI]=1;
                end
                OP_ANA: begin
                    ctrl_wd[ANA]=1;
                    ctrl_wd[ALO]=1;
                    ctrl_wd[FE]=1;
                    ctrl_wd[AI]=1;
                end
            endcase
        end
    endcase
end
assign out=ctrl_wd;
endmodule

module flag_reg(clk,reset,flag_en,cin,zin,cout,zout);
  input clk,reset,flag_en,cin,zin ;
  output reg cout,zout;
  always @(posedge clk )begin
    if(reset)begin
      cout<=1'b0;
      zout<=1'b1;
    end
    else if(flag_en)begin 
      cout<=cin;
      zout<=zin;
    end
  end
endmodule

module INP_reg(
    input clk,reset,inp_loaded,
    input [7:0] inp_data,
    output [7:0] data_out
);
    reg [7:0] inp;    
    assign data_out = inp;
    always @(posedge clk )begin
        if(reset)
            inp <= 8'b0;
        else if(inp_loaded)
                inp <= inp_data;  
    end
endmodule

module reg_IR(
    input clk,
    input reset,
    input ir_in,
    input [7:0] bus_in,
    output [3:0] cs_in,     
    output [3:0] bus_out 
);
    reg [7:0] reg_ir;
    always @(posedge clk ) begin
        if (reset)
            reg_ir <= 8'b0;
        else if (ir_in)
            reg_ir <= bus_in;         
    end
    assign cs_in = reg_ir[7:4];
    assign bus_out = reg_ir[3:0];
endmodule

module MAR_reg(clk,reset,mar_in,bus_in,ram_in);
  input clk, reset, mar_in;
  input [3:0] bus_in;
  output reg [3:0] ram_in;
  
  always @(posedge clk ) 
    begin
      if (reset)
        ram_in <= 4'b0000;       
      else if (mar_in)
        ram_in <= bus_in;        
      else
        ram_in <= ram_in;        
    end
endmodule

module binary_to_7segment_display (
    input  [7:0] bin,    
    output a_h, b_h, c_h, d_h, e_h, f_h, g_h,
    output a_t, b_t, c_t, d_t, e_t, f_t, g_t,
    output a_o, b_o, c_o, d_o, e_o, f_o, g_o
);
    wire [3:0] hundreds, tens, ones;
    wire [6:0] seg_hundreds, seg_tens, seg_ones;

    bin_to_bcd bcd_convert (
        .bin(bin),
        .hundreds(hundreds),
        .tens(tens),
        .ones(ones)
    );
    bcd_to_7seg seg1 (.bcd(ones),     .seg(seg_ones));
    bcd_to_7seg seg2 (.bcd(tens),     .seg(seg_tens));
    bcd_to_7seg seg3 (.bcd(hundreds), .seg(seg_hundreds));
    
    assign {a_h, b_h, c_h, d_h, e_h, f_h, g_h} = seg_hundreds;
    assign {a_t, b_t, c_t, d_t, e_t, f_t, g_t} = seg_tens;
    assign {a_o, b_o, c_o, d_o, e_o, f_o, g_o} = seg_ones;
endmodule

module bin_to_bcd (
    input  [7:0] bin,       
    output reg [3:0] hundreds,
    output reg [3:0] tens,
    output reg [3:0] ones
);
    integer i;
    reg [19:0] bcd;
    always @(*) begin
        bcd = 20'd0;
        bcd[7:0] = bin;
        for (i = 0; i < 8; i = i + 1) begin
            if (bcd[11:8]  >= 5) bcd[11:8]  = bcd[11:8]  + 3; 
            if (bcd[15:12] >= 5) bcd[15:12] = bcd[15:12] + 3; 
            if (bcd[19:16] >= 5) bcd[19:16] = bcd[19:16] + 3; 
            bcd = bcd << 1;  
        end
        hundreds = bcd[19:16];
        tens     = bcd[15:12];
        ones     = bcd[11:8];
    end
endmodule

module bcd_to_7seg (
    input  [3:0] bcd,   
    output reg [6:0] seg 
);
    always @(*) begin
        case (bcd)
            4'b0000: seg = 7'b1111110; 
            4'b0001: seg = 7'b0110000; 
            4'b0010: seg = 7'b1101101; 
            4'b0011: seg = 7'b1111001; 
            4'b0100: seg = 7'b0110011; 
            4'b0101: seg = 7'b1011011; 
            4'b0110: seg = 7'b1011111; 
            4'b0111: seg = 7'b1110000; 
            4'b1000: seg = 7'b1111111; 
            4'b1001: seg = 7'b1111011; 
            default: seg = 7'b0000001; 
        endcase
    end
endmodule

module reg_OUT(
  input clk,
  input reset,
  input out_in,
  input [7:0] bus_in,
  output reg [7:0] dis_out
);
  always @(posedge clk) begin
    if (reset)
      dis_out <= 8'd0;
    else if (out_in)
      dis_out <= bus_in;
  end
endmodule

module PC(clk,reset,pc_en,jmp,bus_in,dis_out,bus_out);
  input clk,reset,pc_en,jmp;
  input [3:0] bus_in;
  output [3:0] dis_out;
  output [3:0] bus_out;
  reg [3:0] pc_reg;
  initial 
    pc_reg = 4'b0000;    // need to be removed for ASIC 
  assign dis_out=pc_reg;
  assign bus_out=pc_reg;
  always @(posedge clk )
    begin 
      if(reset)
        pc_reg<=4'b0000;
      else if(jmp)  
        pc_reg<=bus_in;
      else if(pc_en)
        pc_reg<=pc_reg + 1'b1;
    end 
endmodule  

module RAM(
    input clk,
    input reset,
    input ram_in_en,
    input [3:0] ram_in_addr,
    input [7:0] bus_in,
    output [7:0] bus_out,
    input start,  //manual programming
    input [3:0] sw_addr,
    input [7:0] sw_data,
    input load_btn
);
    reg [7:0] memory [15:0];
    integer i;
    always @(posedge clk ) begin
        if (reset) begin
            for (i = 0; i < 16; i = i + 1)
                memory[i] <= 8'b00000000;
        end 
        else begin
            if (!start && load_btn) begin
                memory[sw_addr] <= sw_data;
            end
            else if (ram_in_en) begin
                memory[ram_in_addr] <= bus_in;
            end
        end
    end
    assign bus_out = memory[ram_in_addr];

endmodule

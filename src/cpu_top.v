module cpu_top (
    input clk,
    input reset,
    input start,

    input [3:0] sw_addr,
    input [7:0] sw_data,
    input load_btn,

    output [3:0] pc_disp,//PC output
    output [7:0] out_display,//8-bit Output

    output a_h, b_h, c_h, d_h, e_h, f_h, g_h,//h-hundred's place
    output a_t, b_t, c_t, d_t, e_t, f_t, g_t,//t-ten's place
    output a_o, b_o, c_o, d_o, e_o, f_o, g_o//o-one's place
);

    // ============================
    // Interconnect wires
    // ============================

    wire [3:0] ram_addr;
    wire [7:0] ram_data_in;
    wire [7:0] ram_data_out;
    wire ram_in_en;
    wire [7:0] out_display;

    // ============================
    // Instantiate CPU core
    // ============================

    cpu_core CPU (
        .clk(clk),
        .reset(reset),
        .start(start),
        .pc_disp(pc_disp),
        .out_display(out_display),

        // RAM interface
        .ram_data_in(ram_data_in),
        .ram_data_out(ram_data_out),
        .ram_addr(ram_addr),
        .ram_in_en(ram_in_en) // ram_out_en removed
    );

    // ============================
    // Instantiate RAM (external)
    // ============================

    RAM RAM_inst (
        .clk(clk),
        .reset(reset),
        .ram_in_en(ram_in_en),  //Twice Combinational changed
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

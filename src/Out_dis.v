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

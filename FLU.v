module FLU(
    input[31:0] A, B,
    input[3:0] flu_ctl,
    output reg [31:0] flu_out 
);
   wire c_out;
   wire [31:0] less,greater,equal,sum_out1,sum_out2,sub_out; 
   float_comparator comp_inst(A,B,less,greater,equal);
   float_add float_add_inst1(A,B,sum_out1,c_out);
   float_add float_add_inst2(A,32'd0,sum_out2,c_out);
   float_add float_add_inst3(A, {~B[31],B[30:0]},sub_out,c_out);
   always@(*) begin
    case(flu_ctl)
    // 0,4,8,9,10,11,13
        4'b0001: flu_out <= sum_out1;
        4'b0010: flu_out <= sub_out;
        4'b0011: flu_out <= equal;
        4'b0100: flu_out <= less || equal;
        4'b0101: flu_out <= less;
        4'b0110: flu_out <= greater || equal;
        4'b0111: flu_out <= greater;
        4'b1000: flu_out <= sum_out2;
        4'b1001: flu_out <= sum_out2;
         
    
    endcase
   end

endmodule

module float_comparator(
    input[31:0] A, B,
    output reg [31:0] less,greater,equal
);

always@(*) begin
                if(A[31] > B[31]) begin
                    less <= 32'd1;
                    greater <= 32'b0;
                    equal <= 32'b0;
                end
                else if(A[31] < B[31]) begin
                    less <= 32'd0;
                    greater <= 32'b1;
                    equal <= 32'b0;
                end
                else begin
                    if(A[30:23] < B[30:23]) begin
                        less <= 32'd1;
                        greater <= 32'b0;
                        equal <= 32'b0;
                    end
                    else if(A[30:23] > B[30:23]) begin
                        less <= 32'd0;
                        greater <= 32'b1;
                        equal <= 32'b0;
                    end
                    else begin
                        if(A[22:0] < B[22:0]) begin
                            less <= 32'd1;
                            greater <= 32'b0;
                            equal <= 32'b0;
                        end
                        else if(A[22:0] > B[22:0]) begin
                            less <= 32'd0;
                            greater <= 32'b1;
                            equal <= 32'b0;
                        end
                        else begin
                            less <= 32'd0;
                            greater <= 32'b0;
                            equal <= 32'b1;
                        end
                    end
                end
             end

    
endmodule

module float_add(
    input [31:0] a,
    input [31:0] b,
    output reg [31:0] sum,
    output reg c_overflow
    );
    
    wire a_sign = a[31];
    wire b_sign = b[31];
    
    wire[7:0] a_exp = a[30:23];
    wire[7:0] b_exp = b[30:23];
    
    wire[22:0] a_mantissa = a[22:0];
    wire[22:0] b_mantissa = b[22:0];
    
    reg[23:0] a_full_mant;
    reg[23:0] b_full_mant;
    
    reg[8:0] diff;
    
    reg[7:0] exp;
    reg cout;
    reg[24:0] tmp_sub;
    reg[1:0] tmp;
    
    always@(*) begin
        c_overflow = 1'b0;
        if(a_exp > b_exp) begin
           diff = a_exp - b_exp;
           a_full_mant = {1'b1,a_mantissa};
           b_full_mant = {1'b1,b_mantissa} >> diff;
           
           exp = a_exp;
        end
        else begin
           diff = b_exp - a_exp;
           a_full_mant = {1'b1,a_mantissa} >> diff;
           b_full_mant = {1'b1,b_mantissa};
           
           exp = b_exp;
        end
        
        if(a_sign == b_sign) begin
            tmp_sub = a_full_mant + b_full_mant;
            {c_overflow, sum[30:23]} = exp + tmp_sub[24];
            {tmp, sum[22:0]} = tmp_sub >> tmp_sub[24];
            sum[31] = a_sign;
        end
        else begin
            tmp_sub = a_full_mant - b_full_mant;
            sum[31] = a[31]^tmp_sub[24];
            tmp_sub[23:0] = (tmp_sub[24]==0)? tmp_sub[23:0] : -tmp_sub[23:0];
            $display("tmp_sub = %b",tmp_sub[23:0]);
            // if cout = 0,sum[22:0] = 0 infinite loop
            if (tmp_sub[24:0] == 24'b0) begin
                sum[30:0] = 31'b0;
            end 
            else begin
                
                while(tmp_sub[23] == 1'b0) begin
                    
                    tmp_sub = tmp_sub << 1;
                    exp = exp - 1;
                end
                sum[22:0] = tmp_sub[22:0];
                sum[30:23] = exp;
            end
        end
        if(c_overflow) begin
            sum[22:0] = 23'd0;
            sum[30:23] = 8'b11111111;
        end else if(a[30:0] == 31'd0) begin
            sum = b;
        end else if (b[30:0] == 31'd0) begin
            sum = a;
        end else if(a[30:23] == 8'b11111111) begin 
            sum = a;
        end else if (b[30:23] == 8'b11111111) begin
            sum = b;
        end
        // 
    end
    
    
endmodule
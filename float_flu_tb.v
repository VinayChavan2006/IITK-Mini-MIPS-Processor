module flu_tb();
    // 10.2 = 0 10000010 01000110011001100110011
    // 5.1 = 0 10000001 01000110011001100110011
    reg [31:0] A,B;
    wire [31:0] out;
    reg [3:0] ctl;
    
    FLU flu_inst(A,B,ctl,out);
    
    initial begin
        A <= 32'b01000001001000110011001100110011;
//        B <= 32'b01000000101000110011001100110011;
        B <= 32'b01000001001000110011001100110011;
        $monitor($time,"A = %b, B= %b, ctl = %d, out = %b",A,B,ctl,out);
        #5 ctl <= 4'd1;
        #5 ctl <= 4'd2;
        #5 ctl <= 4'd3;
        #5 ctl <= 4'd4;
        #5 ctl <= 4'd5;
        #5 ctl <= 4'd6;
        #5 ctl <= 4'd7;
        #5 ctl <= 4'd8;
        #5 ctl <= 4'd9;
        
    
    end

endmodule
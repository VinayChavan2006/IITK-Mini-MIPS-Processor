`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/06/2025 04:25:14 PM
// Design Name: 
// Module Name: processor
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module processor ( 
    input clk,
    input rst
);

    // Internal signals
    wire [31:0] pc, next_pc,float_reg_read1, float_reg_read2, instr, reg_read1, reg_read2, imm_ext, alu_src_muxout, alu_out, mem_read_data, write_data;
    wire [31:0] jump_addr, branch_addr, pc_plus_4;
    wire [4:0] write_reg;
    wire [5:0] alu_ctl;
    wire [5:0] flu_ctl;
    wire RegDst, AluSrc, FloatOp,ALUorFLU, MemtoReg, RegWrite,FloatRegWrite, MemRead, MemWrite, Branch, Jump, Jr, Jal, Zero, InvZero;

    // Program Counter
    PC pc_inst (
        .clk(clk),
        .rst(rst),
        .pcin(next_pc),
        .pcout(pc)
    );

    // Instruction Memory
    instruction_memory instr_mem_inst (
        .addr(pc),
        .instr(instr)
    );

    // Control Unit
    ControlUnit control_inst (
        .opcode(instr[31:26]),
        .funct(instr[5:0]),
        .RegDst(RegDst),
        .AluSrc(AluSrc),
        .MemtoReg(MemtoReg),
        .IntRegWrite(IntRegWrite),
        .FloatRegWrite(FloatRegWrite),
        .MemRead(MemRead),
        .MemWrite(MemWrite),
        .Branch(Branch),
        .Jump(Jump),
        .Jr(Jr),
        .Jal(Jal),
        .FloatOp(FloatOp),
        .ALUorFLU(ALUorFLU)
    );
    // Register File Selection
    assign reg_read1 = FloatOp ? float_reg_read1 : reg_read1;
    assign reg_read2 = FloatOp ? float_reg_read2 : reg_read2;
    
    // Register File
    mux2_1 #(.W(5)) reg_dst_mux (
        .A(instr[20:16]), // rt
        .B(instr[15:11]), // rd
        .Sel(RegDst),
        .mux_out(write_reg)
    );

    register_file reg_file_inst (
        .clk(clk),
        .read_reg1(instr[25:21]), // rs
        .read_reg2(instr[20:16]), // rt
        .write_reg(Jal ? 5'd27 : write_reg), // $ra for jal
        .write_data(Jal ? pc_plus_4 : write_data),
        .reg_write(RegWrite | Jal),
        .read_data1(reg_read1),
        .read_data2(reg_read2)
    );
    
    // Floating-Point Register File
    float_register_file float_reg_file_inst (
        .clk(clk),
        .read_reg1(instr[25:21]),
        .read_reg2(instr[20:16]),
        .write_reg(write_reg),
        .write_data(write_data),
        .reg_write(FloatRegWrite),
        .read_data1(float_reg_read1),
        .read_data2(float_reg_read2)
    );

    // Sign Extend
    sign_extender sign_ext_inst (
        .datain(instr[15:0]),
        .dataout(imm_ext)
    );

    // ALU Control
    ALUControl alu_ctrl_inst (
        .opcode(instr[31:26]),
        .funct(instr[5:0]),
        .alu_ctl(alu_ctl),
        .inv_zero(InvZero)
    );

    // ALU Source Mux
    mux2_1 #(.W(32)) alu_src_mux (
        .A(reg_read2),
        .B(imm_ext),
        .Sel(AluSrc),
        .mux_out(alu_src_muxout)
    );

    // ALU
    ALU alu_inst (
        .opcode(instr[31:26]),
        .A(reg_read1),
        .B(alu_src_muxout),
        .AluCtl(alu_ctl),
        .InvZero(InvZero),
        .AluOut(alu_out),
        .Zero(Zero)
    );
    // FLU
    FLU flu_inst (
        .A(reg_read1),
        .B(reg_read2),
        .flu_ctl(flu_ctl),
        .flu_out(flu_out)
    );
    
    // ALU/FLU Mux
    mux2_1 #(.W(32)) alu_flu_mux (
        .A(alu_out),
        .B(flu_out),
        .Sel(ALUorFLU),
        .mux_out(alu_flu_out)
    );
    // Data Memory
    data_memory data_mem_inst (
        .addr(alu_out),
        .write_data(reg_read2),
        .read_data(mem_read_data),
        .mem_write(MemWrite),
        .mem_read(MemRead)
    );

    // Writeback Mux
    mux2_1 #(.W(32)) mem_to_reg_mux (
        .A(alu_out),
        .B(mem_read_data),
        .Sel(MemtoReg),
        .mux_out(write_data)
    );

    // PC Update Logic
    assign pc_plus_4 = pc + 1;
    assign branch_addr = pc_plus_4 + (imm_ext);
    assign jump_addr = {6'b000000,instr[25:0]};
//    assign jump_addr = {pc[31:28], instr[25:0], 2'b00};

    wire [31:0] branch_or_pc = (Branch & Zero) ? branch_addr : pc_plus_4;
    wire [31:0] jump_or_branch = Jump ? jump_addr : branch_or_pc;
    assign next_pc = Jr ? reg_read1 : jump_or_branch;

endmodule

// 2-to-1 Multiplexer
module mux2_1 #(parameter W = 5) (
    input [W-1:0] A, B,
    input Sel,
    output [W-1:0] mux_out
);
    assign mux_out = Sel ? B : A;
endmodule

// Program Counter
module PC (
    input clk, rst,
    input [31:0] pcin,
    output reg [31:0] pcout
);
    always @(posedge clk or posedge rst) begin
        if (rst)
            pcout <= 32'd0;
        else
            pcout <= pcin;
    end
endmodule

// Instruction Memory
module instruction_memory (
    input [31:0] addr,
    output reg [31:0] instr
);
    reg [31:0] instr_mem [0:1023];
    initial begin
//        addi $t0, $zero, 0
//        addi $t1, $zero, 6
//        addi $t2, $zero, 1
//        bgte $t2, $t0, 18
//        add $t3, $t1, $t2
//        lw $t4, 0($t3)
//        addi $t5, $t2, -1
//        blt $t5, $zero, 6
//        add $t6, $t1, $t5
//        lw $t7, 0($t6)
//        bleq $t7, $t4, 3
//        sw $t7, 1($t6)
//        addi $t5, $t5, -1
//        j 7
//        add $t8, $t1, $t5
//        sw $t4, 1($t8)
//        addi $t2, $t2, 1
//        j 3
        
        instr_mem[0]  = 32'b10000100000010100000000000000000; 
        instr_mem[1]  = 32'b10000100000010010000000000000110; 
        instr_mem[2]  = 32'b10000100000010110000000000000001; 
        instr_mem[3]  = 32'b10110001011010010000000000010010; 
        instr_mem[4]  = 32'b00000001010010110110000000000001; 
        instr_mem[5]  = 32'b10011001100011010000000000000000; 
        instr_mem[6]  = 32'b10000101011011101111111111111111; 
        instr_mem[7]  = 32'b10110101110000000000000000000110; 
        instr_mem[8]  = 32'b00000001010011100111100000000001; 
        instr_mem[9]  = 32'b10011001111100000000000000000000; 
        instr_mem[10] = 32'b10111010000011010000000000000011; 
        instr_mem[11] = 32'b10011101111100000000000000000001; 
        instr_mem[12] = 32'b10000101110011101111111111111111; 
        instr_mem[13] = 32'b01000100000000000000000000000111; 
        instr_mem[14] = 32'b00000001010011101000100000000001; 
        instr_mem[15] = 32'b10011110001011010000000000000001; 
        instr_mem[16] = 32'b10000101011010110000000000000001; 
        instr_mem[17] = 32'b01000100000000000000000000000011; 
        instr_mem[18] = 32'b00000000000000000000000000000000;


    end
    always @(addr) begin
        instr = instr_mem[addr];
    end
endmodule

// Register File
module register_file (
    input clk,
    input [4:0] read_reg1, read_reg2, write_reg,
    input [31:0] write_data,
    input reg_write,
    output [31:0] read_data1, read_data2
);
    reg [31:0] registers [0:31];
    integer i;
    initial begin
        for (i = 0; i < 32; i = i + 1)
            registers[i] = 32'd0;
    end
    assign read_data1 = (read_reg1 == 0) ? 32'd0 : registers[read_reg1];
    assign read_data2 = (read_reg2 == 0) ? 32'd0 : registers[read_reg2];
    always @(posedge clk) begin
        if (reg_write && write_reg != 0)
            registers[write_reg] <= write_data;
    end
endmodule

// Sign Extender
module sign_extender (
    input [15:0] datain,
    output [31:0] dataout
);
    assign dataout = {{16{datain[15]}}, datain};
endmodule

// Data Memory
module data_memory (
    input [31:0] addr,
    input [31:0] write_data,
    input mem_write, mem_read,
    output reg [31:0] read_data
);
    reg [31:0] data_mem [0:1023];
//    initial begin
//        data_mem[32'h0] = 32'd6;
//        data_mem[32'h1] = 32'd3;
//        data_mem[32'h2] = 32'd8;
//        data_mem[32'h3] = 32'd1;
//        data_mem[32'h4] = 32'd9;
//        data_mem[32'h5] = 32'd2;
//    end
    always @(*) begin
        if (mem_write)
            data_mem[addr] = write_data;
        if (mem_read)
            read_data = data_mem[addr];
    end
endmodule

// ALU
module ALU (
    input [31:0] A, B,
    input [5:0] AluCtl,
    input [5:0] opcode,
    input InvZero,
    output reg [31:0] hi,lo,
    output reg [31:0] AluOut,
    output Zero
);
//    reg [31:0] hi, lo;
    reg [63:0] temp;
    always @(*) begin
        hi = 32'h0; lo = 32'h0; // Retain values unless updated
        case (AluCtl)
            6'd1:  AluOut = A + B;                    // add, addu, addi, addiu
            6'd2:  AluOut = A - B;                    // sub, subu
            6'd3:  begin temp = $signed(A) * $signed(B); hi = temp[63:32]; lo = temp[31:0]; AluOut = lo; end // mul
            6'd4:  begin temp = $signed(A) * $signed(B); hi = hi + temp[63:32]; lo = lo + temp[31:0]; AluOut = lo; end // madd, maddu
            6'd5:  AluOut = A << B[4:0];              // sll
            6'd6:  AluOut = A >> B[4:0];              // srl
            6'd7:  AluOut = $signed(A) >>> B[4:0];    // sra
            6'd8:  AluOut = A << B[4:0];              // sla
            6'd9:  AluOut = A | B;                    // or, ori
            6'd10: AluOut = A & B;                    // and, andi
            6'd11: AluOut = A ^ B;                    // xor, xori
            6'd12: AluOut = ~A;                       // not
            6'd13: AluOut = (opcode == 6'b101111 || opcode == 6'b110000) ? 
                            ($unsigned(A) < $unsigned(B) ? 32'd1 : 32'd0) : 
                            ($signed(A) < $signed(B) ? 32'd1 : 32'd0); // slt, slti, bgt, blt, bleu, bgtu
            6'd14: AluOut = A << 16;                        // lui (pass A, but shifted in data path)
            6'd15: AluOut = (A == B) ? 32'd1 : 32'd0; // seq
            6'd16: AluOut = $signed(A) > $signed(B) ? 32'd1 : 32'd0; //sgt
            
            default: AluOut = 32'd0;
        endcase
    end
    assign Zero = InvZero == 1'b1 ? (AluOut != 0) : (AluOut == 0);
endmodule

// ALU Control
module ALUControl (
    input [5:0] opcode, funct,
    output reg [5:0] alu_ctl,flu_ctl,
    output reg inv_zero
);
    always @(*) begin
        inv_zero = 0;
        if (opcode == 6'b000000) begin // R-type
            case (funct)
                6'd1:  alu_ctl = 6'd1;  // add
                6'd2:  alu_ctl = 6'd1;  // addu
                6'd3:  alu_ctl = 6'd2;  // sub
                6'd4:  alu_ctl = 6'd2;  // subu
                6'd5:  alu_ctl = 6'd4;  // madd
                6'd6:  alu_ctl = 6'd4;  // maddu
                6'd7:  alu_ctl = 6'd3;  // mul
                6'd8:  alu_ctl = 6'd10; // and
                6'd9:  alu_ctl = 6'd9;  // or
                6'd10: alu_ctl = 6'd12; // not
                6'd11: alu_ctl = 6'd11; // xor
                6'd12: alu_ctl = 6'd13; // slt
                6'd13: alu_ctl = 6'd5; // sll
                6'd14: alu_ctl = 6'd6; // srl
                6'd15: alu_ctl = 6'd8; // sla
                6'd16: alu_ctl = 6'd7; // sra
                
                default: alu_ctl = 6'd0;
            endcase
        end
        else if (opcode[5] == 1'b1) begin // I-type
            case (opcode[4:0])
                5'd1:  alu_ctl = 6'd1;  // addi
                5'd2:  alu_ctl = 6'd1;  // addiu
                5'd3:  alu_ctl = 6'd10; // andi
                5'd4:  alu_ctl = 6'd9;  // ori
                5'd5:  alu_ctl = 6'd11; // xori
                5'd6: alu_ctl = 6'd1;  // lw
                5'd7: alu_ctl = 6'd1;  // sw
                5'd8: alu_ctl = 6'd14; // lui
                5'd9: begin alu_ctl = 6'd2; inv_zero = 0; end // beq
                5'd10: begin alu_ctl = 6'd2; inv_zero = 1; end // bne
                5'd11: begin alu_ctl = 6'd16; inv_zero = 1; end // bgt
                5'd12: begin alu_ctl = 6'd13; inv_zero = 0; end // bgte
                5'd13: begin alu_ctl = 6'd13; inv_zero = 0; end // blt
                5'd14: begin alu_ctl = 6'd13; inv_zero = 1; end // bleq
                5'd15: begin alu_ctl = 6'd13; inv_zero = 1; end // bleu
                5'd16: begin alu_ctl = 6'd13; inv_zero = 1; end // bgtu
                5'd17: alu_ctl = 6'd13; // slti
                5'd18: alu_ctl = 6'd15; // seq

                default: alu_ctl = 6'd0;
            endcase
        end
        else if (opcode[5:4] == 2'b00) begin
            case(opcode[3:0])
                4'b0001: flu_ctl = 6'd2; // sub.s
                
            endcase
        end
        else
            alu_ctl = 6'd0;
    end
endmodule

// Control Unit
module ControlUnit (
    input [5:0] opcode,
    input [5:0] funct,
    output reg RegDst, AluSrc, MemtoReg, IntRegWrite, FloatRegWrite, MemRead, MemWrite, Branch, Jump, Jr, Jal, FloatOp, ALUorFLU
);
    always @(*) begin
        RegDst = 0; AluSrc = 0; MemtoReg = 0; IntRegWrite = 0; FloatRegWrite = 0;
        MemRead = 0; MemWrite = 0; Branch = 0; Jump = 0; Jr = 0; Jal = 0;
        FloatOp = 0; ALUorFLU = 0;
        casez (opcode)
            6'b000000: begin // R-type (integer)
                RegDst = 1;
                IntRegWrite = 1;
                ALUorFLU = 0;
            end
            6'b000010: begin // fsub
                RegDst = 1;
                FloatRegWrite = 1;
                FloatOp = 1;
                ALUorFLU = 1;
            end
            6'b100001: begin // addi
                AluSrc = 1;
                IntRegWrite = 1;
                ALUorFLU = 0;
            end
            6'b100110: begin // lw
                AluSrc = 1;
                MemtoReg = 1;
                IntRegWrite = 1;
                MemRead = 1;
                ALUorFLU = 0;
            end
            6'b100111: begin // sw
                AluSrc = 1;
                MemWrite = 1;
                ALUorFLU = 0;
            end
            6'b101100: begin // bgte
                Branch = 1;
                ALUorFLU = 0;
            end
            6'b101101: begin // blt
                Branch = 1;
                ALUorFLU = 0;
            end
            6'b101110: begin // bleq
                Branch = 1;
                ALUorFLU = 0;
            end
            6'b010001: begin // j
                Jump = 1;
                ALUorFLU = 0;
            end
            6'b010011: begin // jal
                Jal = 1;
                IntRegWrite = 1;
                ALUorFLU = 0;
            end
            default: begin
                RegDst = 0; AluSrc = 0; MemtoReg = 0; IntRegWrite = 0; FloatRegWrite = 0;
                MemRead = 0; MemWrite = 0; Branch = 0; Jump = 0; Jr = 0; Jal = 0;
                FloatOp = 0; ALUorFLU = 0;
            end
        endcase
    end
endmodule

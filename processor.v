
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
    wire [31:0] pc, next_pc, instr, reg_read1, reg_read2, imm_ext, alu_src_muxout, alu_out, mem_read_data, write_data;
    wire [31:0] jump_addr, branch_addr, pc_plus_4;
    wire [4:0] write_reg;
    wire [5:0] alu_ctl;
    wire RegDst, AluSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, Jump, Jr, Jal, Zero, InvZero;

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
        .RegDst(RegDst),
        .AluSrc(AluSrc),
        .MemtoReg(MemtoReg),
        .RegWrite(RegWrite),
        .MemRead(MemRead),
        .MemWrite(MemWrite),
        .Branch(Branch),
        .Jump(Jump),
        .Jr(Jr),
        .Jal(Jal)
    );

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
        .A(reg_read1),
        .B(alu_src_muxout),
        .AluCtl(alu_ctl),
        .InvZero(InvZero),
        .AluOut(alu_out),
        .Zero(Zero)
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
    assign branch_addr = pc_plus_4 + (imm_ext );
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
//        addi $t1, $zero, 0
//        addi $t0, $zero, 6
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
        
        // instr_mem[0]  = 32'b10000100000010100000000000000000; 
        // instr_mem[1]  = 32'b10000100000010010000000000000110; 
        // instr_mem[2]  = 32'b10000100000010110000000000000001; 
        // instr_mem[3]  = 32'b10110001011010010000000000010010; 
        // instr_mem[4]  = 32'b00000001010010110110000000000001; 
        // instr_mem[5]  = 32'b10011001100011010000000000000000; 
        // instr_mem[6]  = 32'b10000101011011101111111111111111; 
        // instr_mem[7]  = 32'b10110101110000000000000000000110; 
        // instr_mem[8]  = 32'b00000001010011100111100000000001; 
        // instr_mem[9]  = 32'b10011001111100000000000000000000; 
        // instr_mem[10] = 32'b10111010000011010000000000000011; 
        // instr_mem[11] = 32'b10011101111100000000000000000001; 
        // instr_mem[12] = 32'b10000101110011101111111111111111; 
        // instr_mem[13] = 32'b01000100000000000000000000000111; 
        // instr_mem[14] = 32'b00000001010011101000100000000001; 
        // instr_mem[15] = 32'b10011110001011010000000000000001; 
        // instr_mem[16] = 32'b10000101011010110000000000000001; 
        // instr_mem[17] = 32'b01000100000000000000000000000011; 
        // instr_mem[18] = 32'b00000000000000000000000000000000;
        instr_mem[0] = 32'b10000100000010010000000000000110;
        instr_mem[1] = 32'b10011101001100000000000000000001;

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
    always @(negedge clk) begin
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
    reg [31:0] data_mem [0:2047];
    initial begin
        data_mem[32'h0] = 32'd6;
        data_mem[32'h1] = 32'd3;
        data_mem[32'h2] = 32'd8;
        data_mem[32'h3] = 32'd1;
        data_mem[32'h4] = 32'd9;
        data_mem[32'h5] = 32'd2;
    end
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
    input InvZero,
    output reg [31:0] AluOut,
    output Zero
);
    reg [31:0] hi, lo;
    reg [63:0] temp;
    always @(*) begin
        hi = hi; lo = lo; // Retain values unless updated
        case (AluCtl)
            6'd1:  AluOut = A + B;                    // add, addu, addi, addiu
            6'd2:  AluOut = A - B;                    // sub, subu
            6'd3:  begin temp = A * B; hi = temp[63:32]; lo = temp[31:0]; AluOut = lo; end // mul
            6'd4:  begin temp = A * B; hi = hi + temp[63:32]; lo = lo + temp[31:0]; AluOut = lo; end // madd, maddu
            6'd5:  AluOut = A << B[4:0];              // sll
            6'd6:  AluOut = A >> B[4:0];              // srl
            6'd7:  AluOut = $signed(A) >>> B[4:0];    // sra
            6'd8:  AluOut = A << B[4:0];              // sla
            6'd9:  AluOut = A | B;                    // or, ori
            6'd10: AluOut = A & B;                    // and, andi
            6'd11: AluOut = A ^ B;                    // xor, xori
            6'd12: AluOut = ~A;                       // not
            6'd13: AluOut = (A < B) ? 32'd1 : 32'd0;  // slt, slti
            6'd14: AluOut = A;                        // lui (pass A, but shifted in data path)
            6'd15: AluOut = (A == B) ? 32'd1 : 32'd0; // seq
            
            
            default: AluOut = 32'd0;
        endcase
    end
    assign Zero = InvZero ? (AluOut != 0) : (AluOut == 0);
endmodule

// ALU Control
module ALUControl (
    input [5:0] opcode, funct,
    output reg [5:0] alu_ctl,
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
                5'd6:  alu_ctl = 6'd5;  // sll
                5'd7:  alu_ctl = 6'd6;  // srl
                5'd8:  alu_ctl = 6'd8;  // sla
                5'd9:  alu_ctl = 6'd7;  // sra
                5'd10: alu_ctl = 6'd1;  // lw
                5'd11: alu_ctl = 6'd1;  // sw
                5'd12: alu_ctl = 6'd14; // lui
                5'd13: begin alu_ctl = 6'd2; inv_zero = 0; end // beq
                5'd14: begin alu_ctl = 6'd2; inv_zero = 1; end // bne
                5'd15: begin alu_ctl = 6'd13; inv_zero = 1; end // bgt
                5'd16: begin alu_ctl = 6'd13; inv_zero = 0; end // bgte
                5'd17: begin alu_ctl = 6'd13; inv_zero = 0; end // blt
                5'd18: begin alu_ctl = 6'd13; inv_zero = 1; end // bleq
                5'd19: begin alu_ctl = 6'd13; inv_zero = 1; end // bleu
                5'd20: begin alu_ctl = 6'd13; inv_zero = 1; end // bgtu
                5'd21: alu_ctl = 6'd13; // slti
                5'd22: alu_ctl = 6'd15; // seq

                default: alu_ctl = 6'd0;
            endcase
        end
        else if (opcode[5:4] == 2'b00) begin
            case(opcode[3:0])
                4'b0001: alu_ctl = 6'd1; // mfc1
                
            endcase
        end
        else
            alu_ctl = 6'd0;
    end
endmodule

// Control Unit
module ControlUnit (
    input [5:0] opcode,
    output reg RegDst, AluSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, Jump, Jr, Jal
);
    always @(*) begin
        RegDst = 0; AluSrc = 0; MemtoReg = 0; RegWrite = 0;
        MemRead = 0; MemWrite = 0; Branch = 0; Jump = 0; Jr = 0; Jal = 0;
        casez (opcode)
            6'b000000: begin // R-type
                RegDst = 1;
                RegWrite = 1;
            end
            6'b100001: begin // addi
                AluSrc = 1;
                RegWrite = 1;
            end
            6'b100010: begin // addiu
                AluSrc = 1;
                RegWrite = 1;
            end
            6'b100011: begin // andi
                AluSrc = 1;
                RegWrite = 1;
            end
            6'b100100: begin // ori
                AluSrc = 1;
                RegWrite = 1;
            end
            6'b100101: begin // xori
                AluSrc = 1;
                RegWrite = 1;
            end
            6'b100110: begin // sll
                AluSrc = 1;
                RegWrite = 1;
            end
            6'b100111: begin // srl
                AluSrc = 1;
                RegWrite = 1;
            end
            6'b101000: begin // sla
                AluSrc = 1;
                RegWrite = 1;
            end
            6'b101001: begin // sra
                AluSrc = 1;
                RegWrite = 1;
            end
            6'b101010: begin // lw
                AluSrc = 1;
                MemtoReg = 1;
                RegWrite = 1;
                MemRead = 1;
            end
            6'b101011: begin // sw
                AluSrc = 1;
                MemWrite = 1;
            end
            6'b101100: begin // lui
                AluSrc = 1;
                RegWrite = 1;
            end
            6'b101101: Branch = 1; // beq
            6'b101110: Branch = 1; // bne
            6'b101111: Branch = 1; // bgt
            6'b110000: Branch = 1; // bgte
            6'b110001: Branch = 1; // blt
            6'b110010: Branch = 1; // bleq
            6'b110011: Branch = 1; // bleu
            6'b110100: Branch = 1; // bgtu
            6'b110101: begin // slti
                AluSrc = 1;
                RegWrite = 1;
            end
            6'b110110: begin // seq
                AluSrc = 1;
                RegWrite = 1;
            end
            6'b010001: Jump = 1; // j
            6'b010010: Jr = 1;   // jr
            6'b010011: Jal = 1;  // jal
            default: begin
                RegDst = 0; AluSrc = 0; MemtoReg = 0; RegWrite = 0;
                MemRead = 0; MemWrite = 0; Branch = 0; Jump = 0; Jr = 0; Jal = 0;
            end
        endcase
    end
endmodule

module processor_tb;
    reg clk, rst;
    processor uut (
        .clk(clk),
        .rst(rst)
    );
    initial begin
        clk = 0;
        forever #10 clk = ~clk;
    end
    initial begin
        rst = 1;
        #15 rst = 0;
        $monitor("Time=%0t PC=%h s0 = %h, s1 = %d, s2=%d s3=%d t0=%d t1=%h t2=%d ",$time,uut.pc,uut.reg_file_inst.registers[1],uut.reg_file_inst.registers[2],uut.reg_file_inst.registers[3],uut.reg_file_inst.registers[4],  uut.reg_file_inst.registers[9],uut.reg_file_inst.registers[10], uut.reg_file_inst.registers[11]);
        $monitor("Time=%0t PC=%h Instr=%h RegWrite=%b WriteReg=%h WriteData=%h ALUOut=%h MemReadData=%h ",
                 $time, uut.pc, uut.instr, uut.RegWrite, uut.write_reg, uut.write_data, uut.alu_out, uut.mem_read_data);
        $monitor("Time=%0t PC=%h data_mem[0]=%d data_mem[1]=%d data_mem[2]=%d data_mem[3]=%d data_mem[4]=%d data_mem[5]=%d",$time,uut.pc,uut.data_mem_inst.data_mem[32'h0],
                 uut.data_mem_inst.data_mem[32'h1],
                 uut.data_mem_inst.data_mem[32'h2],
                 uut.data_mem_inst.data_mem[32'h3],
                 uut.data_mem_inst.data_mem[32'h4],
                 uut.data_mem_inst.data_mem[32'h5]);
        #10000 $finish;
    end
endmodule

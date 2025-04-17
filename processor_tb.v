`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/08/2025 08:22:22 PM
// Design Name: 
// Module Name: processor_tb
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


// Testbench
`timescale 1ns / 1ps

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
        $monitor("Time=%0t s0 = %h, s1 = %d, s2=%d s3=%d t0=%d t1=%h t2=%d ",$time,uut.reg_file_inst.registers[1],uut.reg_file_inst.registers[2],uut.reg_file_inst.registers[3],uut.reg_file_inst.registers[4],  uut.reg_file_inst.registers[9],uut.reg_file_inst.registers[10], uut.reg_file_inst.registers[11]);
        $monitor("Time=%0t PC=%h Instr=%h RegWrite=%b WriteReg=%h WriteData=%h ALUOut=%h MemReadData=%h ",
                 $time, uut.pc, uut.instr, uut.RegWrite, uut.write_reg, uut.write_data, uut.alu_out, uut.mem_read_data);
        $monitor("Time=%0t data_mem[0x1000]=%d data_mem[0x1004]=%d data_mem[0x1008]=%d data_mem[0x100C]=%d data_mem[0x1010]=%d data_mem[0x1014]=%d",$time,uut.data_mem_inst.data_mem[32'h1000 >> 2],
                 uut.data_mem_inst.data_mem[32'h1004 >> 2],
                 uut.data_mem_inst.data_mem[32'h1008 >> 2],
                 uut.data_mem_inst.data_mem[32'h100C >> 2],
                 uut.data_mem_inst.data_mem[32'h1010 >> 2],
                 uut.data_mem_inst.data_mem[32'h1014 >> 2]);
        #5000 $finish;
    end
endmodule


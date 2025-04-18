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

module tb_processor();
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
        uut.data_mem_inst.data_mem[32'h0] = 32'd6;
        uut.data_mem_inst.data_mem[32'h1] = 32'd3;
        uut.data_mem_inst.data_mem[32'h2] = 32'd8;
        uut.data_mem_inst.data_mem[32'h3] = 32'd1;
        uut.data_mem_inst.data_mem[32'h4] = 32'd9;
        uut.data_mem_inst.data_mem[32'h5] = 32'd2;
    end
    initial begin
        rst = 1;
        #15 rst = 0;
        $monitor("Time=%0t PC=%h t0 = %h, t1 = %d, t2=%d t3=%d t4=%d t5=%h t6=%d t7=%h t8=%h",$time,uut.pc,uut.reg_file_inst.registers[9],uut.reg_file_inst.registers[10],uut.reg_file_inst.registers[11],uut.reg_file_inst.registers[12],  uut.reg_file_inst.registers[13],uut.reg_file_inst.registers[14], uut.reg_file_inst.registers[15],uut.reg_file_inst.registers[16],uut.reg_file_inst.registers[17]);
        $monitor("Time=%0t PC=%h Instr=%h RegWrite=%b WriteReg=%h WriteData=%h ALUOut=%h MemReadData=%h ",
                 $time, uut.pc, uut.instr, uut.RegWrite, uut.write_reg, uut.write_data, uut.alu_out, uut.mem_read_data);
        $monitor("Time=%0t PC=%h data_mem[0]=%d data_mem[1]=%d data_mem[2]=%d data_mem[3]=%d data_mem[4]=%d data_mem[5]=%d mem_write=%d",$time,uut.pc,uut.data_mem_inst.data_mem[32'h0],
                 uut.data_mem_inst.data_mem[32'h1],
                 uut.data_mem_inst.data_mem[32'h2],
                 uut.data_mem_inst.data_mem[32'h3],
                 uut.data_mem_inst.data_mem[32'h4],
                 uut.data_mem_inst.data_mem[32'h5],
                 uut.MemWrite);
        #5000 $finish;
    end
endmodule

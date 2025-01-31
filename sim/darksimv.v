/*
 * Copyright (c) 2018, Marcelo Samsoniuk
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * 
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * 
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 
 */

`timescale 1ns / 1ps
`include "rtl/config.vh"

// clock and reset logic

module darksimv;

    reg CLK = 0;
    
    wire READ;
    reg RES = 1;
    reg read_from_rom = 1;
    assign READ = read_from_rom;

    initial while(1) #(500e6/`BOARD_CK) CLK = !CLK; // clock generator w/ freq defined by config.vh

    integer i;

    initial
    begin
`ifdef __ICARUS__
        $dumpfile("darksocv.vcd");
        $dumpvars();

    `ifdef __REGDUMP__
        for(i=0;i!=`RLEN;i=i+1)
        begin
            $dumpvars(0,soc0.core0.REGS[i]);
        end
    `endif
`endif
        $display("reset (startup)");
        #1e3    RES = 0;            // wait 1us in reset state
        $display("reset (restart)");

        read_from_rom = 1;
        #2e3; // setup code, want to read from ROM
        $display("finsihed reading from ROM");
        read_from_rom = 0;
        #99e3; // feed in our custom IDATA!
        $display("turning read_from_rom: %d", read_from_rom);
        $display("turning READ: %d", READ);
        $finish();
        //#1000e3 $finish();          // run  1ms
    end

    wire TX;
    wire RX = 1;
    wire [31 : 0] pc;


    reg [31:0] IDATA_reg;
    wire [31:0] IDATA;

    // intiialize the first instruction to be nothing
    initial IDATA_reg <= 32'b0;

    // add (R) instruction
    reg [6:0] opcode = 7'b0110011;
    reg [2:0] funct3 = 3'b0;
    reg [6:0] funct7 = 7'b0;

    (* anyseq *) reg [4:0] rd;
    (* anyseq *) reg [4:0] rs1;
    (* anyseq *) reg [4:0] rs2;

    assign IDATA = {funct7, rs2, rs1, funct3, rd, opcode};

    darksocv soc0
    (
        .XCLK(CLK),
        .XRES(|RES),
        .UART_RXD(RX),
        .UART_TXD(TX),
        .IDATA_IN(IDATA), // controlled by corp 
        .READ(read_from_rom), // tells the SOC to read from ROM when need to

        // output of the RISCV core
        .pc_out(pc),
    );


    // for formal analysis
    always @(*) begin
        assert(soc0.core0.REGS[0] == soc0.core0.REGS[0]);
    end


    // TODO we now formally verify the ADD (R) instruction
    //always @(*) begin
        // TODO formal verification goes here
    //end

endmodule

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
`include "utils/packer.v"
`include "../rtl/config.vh"

module formal;

    reg CLK = 0;

    reg RES = 1;
    /* Setup code */
    initial while(1) #(500e6/`BOARD_CK) CLK = !CLK;
    initial begin
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
        $display("Hello World!");
        #1     RES = 0;            // wait 1us in reset state
        #100e3 RES = 1;            // run  1ms
        $finish();
    end

    // clock counter
    reg [40:0] counter = 5097'b0;
    wire [31:0] IDATA;
    wire [`REG_TOTAL - 1 : 0] regfile /*synthesis keep*/;

    wire [31 : 0] pc;

    // uart signals
    wire TX;
    wire RX = 1;

    // add (R) instruction
    reg [6:0] opcode = 7'b0110011;
    reg [2:0] funct3 = 3'b0;
    reg [6:0] funct7 = 7'b0;

    // add (I) instruction
    /*
    reg [6:0] opcode = 7'b0010011;
    reg [2:0] funct3 = 3'b0;
    reg [6:0] funct7 = 7'b0;
    */

    reg [4:0] rd;
    reg [4:0] rs1;
    reg [4:0] rs2;

    reg [31:0] rs1_history [1:0];
    reg [31:0] rs2_history [1:0];
    reg [4:0] rd_history [1:0];

    integer i;
    initial begin
        $display("RLEN: %d", `RLEN);
        rd <= $urandom % `RLEN;
        rs1 <= $urandom % `RLEN;
        rs2 <= $urandom % `RLEN;

        for (i = 0; i < 3; i += 1) begin
            rs1_history[i] = 32'b0;
            rs2_history[i] = 32'b0;
            rd_history[i] = 5'b0;
        end
    end

    // random adds begin coming in when the counter >= 10
    assign IDATA = (RES == 0) ? {funct7, rs2, rs1, funct3, rd, opcode} : 32'h00000013;

    // registers
    wire [31:0] regs [0:15] /*synthesis keep*/;
    `UNPACK_ARRAY(32, 16, regs, regfile);

    darksocv soc0
    (
        .XCLK(CLK),
        .XRES(|RES),
        .UART_RXD(RX),
        .UART_TXD(TX),
        .IDATA_IN(IDATA), // controlled by corp 

        // output of the RISCV core
        .pc_out(pc),
        .regfile_out(regfile)
    );

    /* test driver code */
    always @(posedge CLK) begin
        if (RES == 0) begin
            rd <= (counter % `RLEN);
            rs1 <= (counter + 1) % `RLEN;
            rs2 <= (counter + 2) % `RLEN;

            // Instructions exhibit architectural effects after
            // counter >= 3
            if (counter >= 3) begin
                rs2_history[counter % 2] <= regs[rs2];
                rs1_history[counter % 2] <= regs[rs1];
                rd_history[counter % 2] <= rd;
            end

            if (counter >= 5) begin
                if (rd_history[counter % 2] != 0) begin
                    // TODO assertions go here
                    $display("%d + %d = %d, c=%d", rs1_history[counter % 2], 
                        rs2_history[counter % 2],
                        regs[rd_history[counter % 2]], counter[12:0]);
                end
            end

            counter <= counter + 1;
        end
    end

endmodule

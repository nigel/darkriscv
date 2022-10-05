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
`include "darksocv.v"
`include "utils/packer.v"

module formal (
    input logic clk
);

    // clock counter
    reg [31:0] counter = 32'b0;

    // core signals
    wire RES;
    assign RES = counter <= 5;

    wire [31:0] IDATA;

    wire [1023 : 0] regfile;
    wire [31 : 0] pc;

    // uart signals
    wire TX;
    wire RX = 1;

    // add (R) instruction
    reg [6:0] opcode = 7'b0110011;
    reg [2:0] funct3 = 3'b0;
    reg [6:0] funct7 = 7'b0;

    (* anyseq *) reg [4:0] rd;
    (* anyseq *) reg [4:0] rs1;
    (* anyseq *) reg [4:0] rs2;

    // random adds begin coming in when the counter >= 10
    assign IDATA = (counter >= 10) ? {funct7, rs2, rs1, funct3, rd, opcode} : 32'h00000013;

    // registers
    reg [31:0] regs [0:31];
    `UNPACK_ARRAY(32, 32, regs, regfile);

    darksocv soc0
    (
        .XCLK(clk),
        .XRES(|RES),
        .UART_RXD(RX),
        .UART_TXD(TX),
        .IDATA_IN(IDATA), // controlled by corp 

        // TODO regfile_in
        // TODO regfile_wren

        // output of the RISCV core
        .pc_out(pc),
        .regfile_out(regfile)
    );


    always @(posedge clk) begin
        counter <= counter + 1;
        if (counter == 6) begin
            // set the regs
        end else if (counter >= 10) begin
            // TODO assertion goes here once the adds begin coming
        end
    end

endmodule

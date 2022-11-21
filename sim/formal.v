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

`define R_TYPE 0
`define I_TYPE 1
`define S_TYPE 2
`define L_TYPE 3 // I-type instruction but we're modeling loads

`define SB 0;
`define SH 1;
`define SW 2;
`define SD 3;

`define RAM_SIZE 2**`MLEN/4-1

module formal;

    wire HLT;
    reg CLK = 0;

    reg [31:0] RAM [0 : `RAM_SIZE];

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
        #1000 RES = 1;            // run  1ms
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

    // opcode definitions
    reg [6:0] OPCODES [3:0];

    /* Determines the type of instruction we're currently testing */
    reg [1:0] op_mode = `R_TYPE;

    // (R-Type) instruction
    wire [6:0] opcode = OPCODES[op_mode];
    reg [2:0] funct3 = 3'b001;
    reg [6:0] funct7 = 7'b0000000;
    reg [11:0] imm = 12'b0;
    reg [4:0] rd;
    reg [4:0] rs1;
    reg [4:0] rs2;

    /* History buffers */
    reg [31:0] rs1_history [1:0];
    reg [31:0] rs2_history [1:0];
    reg [4:0] rd_history [1:0];
    reg [2:0] f3_history [1:0];
    reg [6:0] f7_history [1:0];
    reg [6:0] op_history [1:0];
    reg [11:0] imm_history [1:0];

    wire counter_cur = counter % 2;

    /* Values of the current cycle */
    wire [31:0] rs1_cur = rs1_history[counter_cur];
    wire [31:0] rs2_cur = rs2_history[counter_cur];
    wire [31:0] rd_cur = regs[rd_history[counter_cur]];
    wire [2:0] f3_cur = f3_history[counter_cur];
    wire [6:0] f7_cur = f7_history[counter_cur];
    wire [6:0] op_cur = op_history[counter_cur];
    wire [11:0] imm_cur = imm_history[counter_cur];
    wire [31:0] imm_cur_signed = $signed({{20 {imm_history[counter_cur][11]}}, imm_history[counter_cur]});


    /* Correction signals */
    wire r_correct =    f3_cur == 3'b000 ? rs1_cur + rs2_cur == rd_cur    :
                        f3_cur == 3'b001 ? (rs1_cur << rs2_cur[4:0]) == rd_cur :
                        f3_cur == 3'b010 ? rd_cur == ($signed(rs1_cur) < $signed(rs2_cur)) :
                        f3_cur == 3'b011 ? rd_cur == (rs1_cur < rs2_cur) :
                        f3_cur == 3'b100 ? rd_cur == (rs1_cur ^ rs2_cur) :
                        f3_cur == 3'b101 ? rd_cur == (rs1_cur >> rs2_cur[4:0]) :
                        f3_cur == 3'b110 ? rd_cur == (rs1_cur | rs2_cur) :
                        f3_cur == 3'b111 ? rd_cur == (rs1_cur & rs2_cur) :
                        0;

    wire i_correct =    f3_cur == 3'b000 ? rd_cur == (rs1_cur + {{20{imm_cur[11]}}, imm_cur}) :
                        f3_cur == 3'b001 ? rd_cur == (rs1_cur << imm_cur[6:0]) :
                        f3_cur == 3'b010 ? rd_cur == ($signed(rs1_cur) < $signed({{20{imm_cur[11]}}, imm_cur})) :
                        f3_cur == 3'b011 ? rd_cur == (rs1_cur < {20'b0, imm_cur}) :
                        f3_cur == 3'b100 ? rd_cur == (rs1_cur ^ {{20{imm_cur[11]}}, imm_cur}) :
                        f3_cur == 3'b101 ? rd_cur == (rs1_cur >> imm_cur[6:0]) :
                        f3_cur == 3'b110 ? rd_cur == (rs1_cur | {{20{imm_cur[11]}}, imm_cur}) :
                        f3_cur == 3'b111 ? rd_cur == (rs1_cur & {{20{imm_cur[11]}}, imm_cur}) :
                        0;
                    
    wire l_correct =    f3_cur == 3'b000 ? rd_cur == ((((rs1_cur + imm_cur_signed) % 4) == 0) ?
                                    {{24 {RAM[(rs1_cur + imm_cur_signed)][7]}}, RAM[(rs1_cur + imm_cur_signed) / 4][7:0]} : // lb
                                    (((rs1_cur + imm_cur_signed) % 4) == 1) ?
                                    {{24 {RAM[(rs1_cur + imm_cur_signed)][15]}}, RAM[(rs1_cur + imm_cur_signed) / 4][15:8]} : // lb
                                    (((rs1_cur + imm_cur_signed) % 4) == 2) ?
                                    {{24 {RAM[(rs1_cur + imm_cur_signed)][23]}}, RAM[(rs1_cur + imm_cur_signed) / 4][23:16]} : // lb
                                    {{24 {RAM[(rs1_cur + imm_cur_signed)][31]}}, RAM[(rs1_cur + imm_cur_signed) / 4][31:24]} // lb
                                ) :

                        f3_cur == 3'b001 ? rd_cur == ((((rs1_cur + imm_cur_signed) % 4) < 2) ?
                                    {{16 {RAM[(rs1_cur + imm_cur_signed)][15]}}, RAM[(rs1_cur + imm_cur_signed) / 4][15:0]} : // lh
                                    {{16 {RAM[(rs1_cur + imm_cur_signed)][31]}}, RAM[(rs1_cur + imm_cur_signed) / 4][31:16]} // lh
                                ) :

                        f3_cur == 3'b010 ? rd_cur == RAM[(rs1_cur + imm_cur_signed) / 4] : // lw
                        f3_cur == 3'b011 ? rd_cur == RAM[(rs1_cur + imm_cur_signed)] :// ld
                        f3_cur == 3'b100 ? rd_cur == {24'b0, RAM[(rs1_cur + imm_cur_signed)][7:0]} : // lbu
                        f3_cur == 3'b101 ? rd_cur == {16'b0, RAM[(rs1_cur + imm_cur_signed)][15:0]} : // lhu
                        f3_cur == 3'b110 ? rd_cur == RAM[(rs1_cur + imm_cur_signed)] : // lwu
                        0;

    wire correct =  op_cur == `R_TYPE ? r_correct :
                    op_cur == `I_TYPE ? i_correct :
                    op_cur == `S_TYPE ? 1 : // no architectural effects when we do stores
                    op_cur == `L_TYPE ? l_correct :
                    0;

    integer i;
    initial begin
        /* Intial instruction field selection */
        rd <= $urandom % `RLEN;
        rs1 <= $urandom % `RLEN;
        rs2 <= $urandom % `RLEN;
        funct3 <= $urandom % 7;
        imm <= $urandom;

        /* OPCODE constant initialization */
        OPCODES[0] <= 7'b011_0011; // R format
        OPCODES[1] <= 7'b001_0011; // I format
        OPCODES[2] <= 7'b010_0011; // S format
        OPCODES[3] <= 7'b000_0011; // L format

        for (i = 0; i < 2; i += 1) begin
            rs1_history[i] = 32'b0;
            rs2_history[i] = 32'b0;
            rd_history[i] = 5'b0;
            op_history[i] = 7'b0;
            imm_history[i] = 12'b0;
            f3_history[i] = 3'b0;
        end

        // initialize our "shadow" ram with all 0's.
        for (i = 0; i < `RAM_SIZE; i += 1) begin
            RAM[i] = 32'b0;
        end
    end

    reg load = 0;

    assign IDATA = (RES == 0) && (load == 0) ? (
        op_mode == `R_TYPE ? {funct7, rs2, rs1, funct3, rd, opcode} :
        op_mode == `I_TYPE ? {imm, rs1, funct3, rd, opcode}         :
        op_mode == `L_TYPE ? {imm, rs1, funct3, rd, opcode}         :
        op_mode == `S_TYPE ? {funct7, rs2, rs1, funct3, rd, opcode} :
        // funct7: imm[11:5], rd: imm[4:0]
        32'h00000013) : 32'h00000013;

    darksocv soc0
    (
        .XCLK(CLK),
        .XRES(|RES),
        .UART_RXD(RX),
        .UART_TXD(TX),
        .IDATA_IN(IDATA), // controlled by corp 

        // output of the RISCV core
        .pc_out(pc),
        .regfile_out(regfile),
        .HLT_o(HLT)
    );

    /* Assertions */
    always @(posedge CLK) begin

        if (!HLT) begin
            if (counter >= 5
                && (rd_cur != 0)
                && (!correct))  begin
                $display("\n\nINCORRECT %d", counter);
                $display("recieve [%x] = %x",(rs1_cur + $signed(imm_cur)), rd_cur);

                $display("local RAM[%x] = %x", (rs1_cur + $signed(imm_cur)), RAM[(rs1_cur +$signed(imm_cur)) / 4]);
                $display("core RAM[%x] = %x", (rs1_cur + $signed(imm_cur)), (soc0.MEM[(rs1_cur + $signed(imm_cur))/ 4]));
                $display("expected: %x", RAM[(rs1_cur + imm_cur_signed) / 4]);

                $display("LOCAL");
                $display("RAM[0]: %x", RAM[0]);
                $display("RAM[1]: %x", RAM[1]);
                $display("RAM[2]: %x", RAM[2]);
                $display("RAM[3]: %x", RAM[3]);


                $display("MEM[0]: %x", soc0.MEM[0]);
                $display("MEM[1]: %x", soc0.MEM[1]);
                $display("MEM[2]: %x", soc0.MEM[2]);
                $display("MEM[3]: %x", soc0.MEM[3]);

            end
        end
    end

    /* Update these values after every cycle */
    always @(posedge CLK) begin
        if (!HLT) begin // ensure that the core isn't waiting for peripherals 
            
            counter <= counter + 1;

            /* Opmode selection */
            if (counter >= 20) begin
                // TODO only test sw, lw
                op_mode <= `L_TYPE;
                rs1 <= (counter) + 4 % `RLEN; // register that stores the address
                rs2 <= 0;                     // offset
                rd <= (counter + 1) % `RLEN;  // destination
                imm <= 0; // TODO
                funct3 <= $urandom % 3;
                funct7 <= 7'b0;
            end else begin
                op_mode <= `S_TYPE;
                rd <= 0; // offset
                funct7 <= 7'b0;
                funct3 <= $urandom % 3;
                rs1 <= (counter) % `RLEN; // address of where to store
                rs2 <= (counter + 2) % `RLEN;
            end

            /* History buffer */
            // only lower 5 bits so we don't mess with the funct7
            // imm <= ((op_mode == `I_TYPE) || (op_mode == `R_TYPE)) ? $urandom & (5'b1) : 0; // make sure that shifts can work 
            if (counter >= 3) begin
                rs2_history[counter_cur] <= regs[rs2];
                rs1_history[counter_cur] <= regs[rs1];
                rd_history[counter_cur] <= rd;
                f3_history[counter_cur] <= funct3;
                imm_history[counter_cur] <= imm;
                // do not use `opcodes` for this because it's a wire and results show up imm.
                op_history[counter_cur] <= op_mode;
            end
        end
    end

    // shadow RAM heuristics
    always @(posedge CLK) begin
        if (!HLT && (op_mode == `S_TYPE)) begin
            case (funct3)
                3'b000: begin // sb
                    if (((regs[rs1] + $signed({rd, funct7})) % 4) == 0) begin
                        RAM[(regs[rs1] + $signed({rd, funct7})) / 4][7:0] <= regs[rs2][7:0];
                    end else if (((regs[rs1] + $signed({rd, funct7})) % 4) == 1) begin
                        RAM[(regs[rs1] + $signed({rd, funct7})) / 4][15:8] <= regs[rs2][7:0];
                    end else if (((regs[rs1] + $signed({rd, funct7})) % 4) == 2) begin
                        RAM[(regs[rs1] + $signed({rd, funct7})) / 4][23:16] <= regs[rs2][7:0];
                    end else begin
                        RAM[(regs[rs1] + $signed({rd, funct7})) / 4][31:24] <= regs[rs2][7:0];
                    end
                end

                3'b001: begin // sh
                    if (((regs[rs1] + $signed({rd, funct7})) % 4) < 2) begin
                        RAM[(regs[rs1] + $signed({rd, funct7})) / 4][15:0] <= regs[rs2][15:0];
                        $display("(shadow): ram[%x] = %x",
                            ((regs[rs1] + $signed({rd, funct7}))),
                            regs[rs2][15:0]);
                    end else begin
                        RAM[(regs[rs1] + $signed({rd, funct7})) / 4][31:16] <= regs[rs2][15:0];
                        $display("(shadow): ram[%x] = %x",
                            ((regs[rs1] + $signed({rd, funct7}))),
                            regs[rs2][15:0]);
                    end
                end

                3'b010: begin // sw
                    RAM[(regs[rs1] + $signed({rd, funct7})) / 4] <= regs[rs2];
                end

                3'b011: begin // sd
                    RAM[(regs[rs1] + $signed({rd, funct7})) / 4][31:0] <= regs[rs2];
                end
            endcase
        end
    end

    /* -------- DEBUGGING */
    // registers
    wire [31:0] regs [0:15] /*synthesis keep*/;
    `UNPACK_ARRAY(32, 16, regs, regfile);

endmodule

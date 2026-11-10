/*------------------------------------------------------------------------------
--------------------------------------------------------------------------------
Copyright (c) 2016, Loongson Technology Corporation Limited.

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this 
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, 
this list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

3. Neither the name of Loongson Technology Corporation Limited nor the names of 
its contributors may be used to endorse or promote products derived from this 
software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
DISCLAIMED. IN NO EVENT SHALL LOONGSON TECHNOLOGY CORPORATION LIMITED BE LIABLE
TO ANY PARTY FOR DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE 
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
--------------------------------------------------------------------------------
------------------------------------------------------------------------------*/
`timescale 1ns / 1ps

`define TRACE_REF_FILE "../../../../../../../../gettrace/golden_trace.txt"
`define CONFREG_NUM_REG      soc_lite.u_confreg.num_data
`define CONFREG_OPEN_TRACE   soc_lite.u_confreg.open_trace
`define CONFREG_NUM_MONITOR  soc_lite.u_confreg.num_monitor
`define CONFREG_UART_DISPLAY soc_lite.u_confreg.write_uart_valid
`define CONFREG_UART_DATA    soc_lite.u_confreg.write_uart_data
`define END_PC 32'h1c000100

module tb_top( );
reg resetn;
reg clk;

//goio
wire [15:0] led;
wire [1 :0] led_rg0;
wire [1 :0] led_rg1;
wire [7 :0] num_csn;
wire [6 :0] num_a_g;
wire [7 :0] switch;
wire [3 :0] btn_key_col;
wire [3 :0] btn_key_row;
wire [1 :0] btn_step;
assign switch      = 8'hff;
assign btn_key_row = 4'd0;
assign btn_step    = 2'd3;

initial
begin
    clk = 1'b0;
    resetn = 1'b0;
    #2000;
    resetn = 1'b1;
end
// 200 MHz test clock: soc_lite_top divides it by 2 for the core and drives the
// memory subsystem with it directly, so the core still runs at 10 ns/cycle
// while memory/AXI runs 2x -- the same 2:1 ratio as the 50/100 MHz clk_pll pair.
always #2.5 clk=~clk;
soc_lite_top #(.SIMULATION(1'b1)) soc_lite
(
       .resetn      (resetn     ), 
       .clk         (clk        ),
    
        //------gpio-------
        .num_csn    (num_csn    ),
        .num_a_g    (num_a_g    ),
        .led        (led        ),
        .led_rg0    (led_rg0    ),
        .led_rg1    (led_rg1    ),
        .switch     (switch     ),
        .btn_key_col(btn_key_col),
        .btn_key_row(btn_key_row),
        .btn_step   (btn_step   )
    );   

//soc lite signals
//"soc_clk" means clk in cpu
//"wb" means write-back stage in pipeline
//"rf" means regfiles in cpu
//"w" in "wen/wnum/wdata" means writing
wire soc_clk;
wire [31:0] debug_wb_pc;
wire [3 :0] debug_wb_rf_we;
wire [4 :0] debug_wb_rf_wnum;
wire [31:0] debug_wb_rf_wdata;
// dual issue: slot 1 of the pair (the younger write-back)
wire [31:0] debug_wb_pc_1;
wire [3 :0] debug_wb_rf_we_1;
wire [4 :0] debug_wb_rf_wnum_1;
wire [31:0] debug_wb_rf_wdata_1;
assign soc_clk           = soc_lite.cpu_clk;
assign debug_wb_pc       = soc_lite.debug_wb_pc;
assign debug_wb_rf_we    = soc_lite.debug_wb_rf_we;
assign debug_wb_rf_wnum  = soc_lite.debug_wb_rf_wnum;
assign debug_wb_rf_wdata = soc_lite.debug_wb_rf_wdata;
assign debug_wb_pc_1       = soc_lite.debug_wb_pc_1;
assign debug_wb_rf_we_1    = soc_lite.debug_wb_rf_we_1;
assign debug_wb_rf_wnum_1  = soc_lite.debug_wb_rf_wnum_1;
assign debug_wb_rf_wdata_1 = soc_lite.debug_wb_rf_wdata_1;

// open the trace file;
integer trace_ref;
initial begin
    trace_ref = $fopen(`TRACE_REF_FILE, "r");
end

//get reference result in falling edge
reg        trace_cmp_flag;
reg        debug_end;

// one reference entry per write-back; a dual-issue core may produce two in the
// same cycle (slot 0 first, then slot 1), so two entries are fetched here
reg [31:0] ref_wb_pc        [1:0];
reg [4 :0] ref_wb_rf_wnum   [1:0];
reg [31:0] ref_wb_rf_wdata  [1:0];
reg        ref_wb_vld       [1:0];

task get_ref;
    input integer slot;
    begin
        trace_cmp_flag=1'b0;
        while (!trace_cmp_flag && !($feof(trace_ref)))
        begin
            $fscanf(trace_ref, "%h %h %h %h", trace_cmp_flag,
                    ref_wb_pc[slot], ref_wb_rf_wnum[slot], ref_wb_rf_wdata[slot]);
        end
        ref_wb_vld[slot] = 1'b1;
    end
endtask

always @(posedge soc_clk)
begin 
    #1;
    ref_wb_vld[0] = 1'b0;
    ref_wb_vld[1] = 1'b0;
    if(!debug_end && `CONFREG_OPEN_TRACE)
    begin
        if (|debug_wb_rf_we   && (debug_wb_rf_wnum   != 5'd0)) get_ref(0);
        if (|debug_wb_rf_we_1 && (debug_wb_rf_wnum_1 != 5'd0)) get_ref(1);
    end
end

//wdata[i*8+7 : i*8] is valid, only wehile wen[i] is valid
wire [31:0] debug_wb_rf_wdata_v;
wire [31:0] debug_wb_rf_wdata_v_1;
wire [31:0] ref_wb_rf_wdata_v   [1:0];
assign debug_wb_rf_wdata_v[31:24] = debug_wb_rf_wdata[31:24] & {8{debug_wb_rf_we[3]}};
assign debug_wb_rf_wdata_v[23:16] = debug_wb_rf_wdata[23:16] & {8{debug_wb_rf_we[2]}};
assign debug_wb_rf_wdata_v[15: 8] = debug_wb_rf_wdata[15: 8] & {8{debug_wb_rf_we[1]}};
assign debug_wb_rf_wdata_v[7 : 0] = debug_wb_rf_wdata[7 : 0] & {8{debug_wb_rf_we[0]}};
assign debug_wb_rf_wdata_v_1[31:24] = debug_wb_rf_wdata_1[31:24] & {8{debug_wb_rf_we_1[3]}};
assign debug_wb_rf_wdata_v_1[23:16] = debug_wb_rf_wdata_1[23:16] & {8{debug_wb_rf_we_1[2]}};
assign debug_wb_rf_wdata_v_1[15: 8] = debug_wb_rf_wdata_1[15: 8] & {8{debug_wb_rf_we_1[1]}};
assign debug_wb_rf_wdata_v_1[7 : 0] = debug_wb_rf_wdata_1[7 : 0] & {8{debug_wb_rf_we_1[0]}};
assign   ref_wb_rf_wdata_v[0][31:24] = ref_wb_rf_wdata[0][31:24] & {8{debug_wb_rf_we[3]}};
assign   ref_wb_rf_wdata_v[0][23:16] = ref_wb_rf_wdata[0][23:16] & {8{debug_wb_rf_we[2]}};
assign   ref_wb_rf_wdata_v[0][15: 8] = ref_wb_rf_wdata[0][15: 8] & {8{debug_wb_rf_we[1]}};
assign   ref_wb_rf_wdata_v[0][7 : 0] = ref_wb_rf_wdata[0][7 : 0] & {8{debug_wb_rf_we[0]}};
assign   ref_wb_rf_wdata_v[1][31:24] = ref_wb_rf_wdata[1][31:24] & {8{debug_wb_rf_we_1[3]}};
assign   ref_wb_rf_wdata_v[1][23:16] = ref_wb_rf_wdata[1][23:16] & {8{debug_wb_rf_we_1[2]}};
assign   ref_wb_rf_wdata_v[1][15: 8] = ref_wb_rf_wdata[1][15: 8] & {8{debug_wb_rf_we_1[1]}};
assign   ref_wb_rf_wdata_v[1][7 : 0] = ref_wb_rf_wdata[1][7 : 0] & {8{debug_wb_rf_we_1[0]}};


//compare result in rsing edge 
reg debug_wb_err;
task chk_wb;
    input integer slot;
    input [31:0] wb_pc;
    input [4 :0] wb_wnum;
    input [31:0] wb_wdata_v;
    begin
        if (  (wb_pc!==ref_wb_pc[slot]) || (wb_wnum!==ref_wb_rf_wnum[slot])
            ||(wb_wdata_v!==ref_wb_rf_wdata_v[slot]) )
        begin
            $display("--------------------------------------------------------------");
            $display("[%t] Error!!! (slot %0d)",$time, slot);
            $display("    reference: PC = 0x%8h, wb_rf_wnum = 0x%2h, wb_rf_wdata = 0x%8h",
                      ref_wb_pc[slot], ref_wb_rf_wnum[slot], ref_wb_rf_wdata_v[slot]);
            $display("    mycpu    : PC = 0x%8h, wb_rf_wnum = 0x%2h, wb_rf_wdata = 0x%8h",
                      wb_pc, wb_wnum, wb_wdata_v);
            $display("--------------------------------------------------------------");
            debug_wb_err <= 1'b1;
            #40;
            $finish;
        end
    end
endtask

always @(posedge soc_clk)
begin
    #2;
    if(!resetn)
    begin
        debug_wb_err <= 1'b0;
    end
    else if(!debug_end && `CONFREG_OPEN_TRACE)
    begin
        if (ref_wb_vld[0]) chk_wb(0, debug_wb_pc,   debug_wb_rf_wnum,   debug_wb_rf_wdata_v);
        if (ref_wb_vld[1]) chk_wb(1, debug_wb_pc_1, debug_wb_rf_wnum_1, debug_wb_rf_wdata_v_1);
    end
end

//monitor numeric display
reg [7:0] err_count;
wire [31:0] confreg_num_reg = `CONFREG_NUM_REG;
reg  [31:0] confreg_num_reg_r;
always @(posedge soc_clk)
begin
    confreg_num_reg_r <= confreg_num_reg;
    if (!resetn)
    begin
        err_count <= 8'd0;
    end
    else if (confreg_num_reg_r != confreg_num_reg && `CONFREG_NUM_MONITOR)
    begin
        if(confreg_num_reg[7:0]!=confreg_num_reg_r[7:0]+1'b1)
        begin
            $display("--------------------------------------------------------------");
            $display("[%t] Error(%d)!!! Occurred in number 8'd%02d Functional Test Point!",$time, err_count, confreg_num_reg[31:24]);
            $display("--------------------------------------------------------------");
            err_count <= err_count + 1'b1;
        end
        else if(confreg_num_reg[31:24]!=confreg_num_reg_r[31:24]+1'b1)
        begin
            $display("--------------------------------------------------------------");
            $display("[%t] Error(%d)!!! Unknown, Functional Test Point numbers are unequal!",$time,err_count);
            $display("--------------------------------------------------------------");
            $display("==============================================================");
            err_count <= err_count + 1'b1;
        end
        else
        begin
            $display("----[%t] Number 8'd%02d Functional Test Point PASS!!!", $time, confreg_num_reg[31:24]);
        end
    end
end

//monitor test
initial
begin
    $timeformat(-9,0," ns",10);
    while(!resetn) #5;
    $display("==============================================================");
    $display("Test begin!");

    #10000;
    while(`CONFREG_NUM_MONITOR)
    begin
        #10000;
        $display ("        [%t] Test is running, debug_wb_pc = 0x%8h",$time, debug_wb_pc);
    end
end

//模拟串口打印
// confreg 现在跑在 2x 的 mem_clk 上，write_uart_valid 只保持一个 mem 拍，
// 所以这里用 mem_clk 采样（原先在 soc_clk 上采，2x 之后会漏字符）。
wire uart_display;
wire [7:0] uart_data;
wire       soc_mem_clk;
assign uart_display = `CONFREG_UART_DISPLAY;
assign uart_data    = `CONFREG_UART_DATA;
assign soc_mem_clk  = soc_lite.mem_clk;

always @(posedge soc_mem_clk)
begin
    if(uart_display)
    begin
        if(uart_data==8'hff)
        begin
            ;//$finish;
        end
        else
        begin
            $write("%c",uart_data);
        end
    end
end

//test end
wire global_err = debug_wb_err || (err_count!=8'd0);
wire test_end = (debug_wb_pc==`END_PC) || (uart_display && uart_data==8'hff);
always @(posedge soc_clk)
begin
    if (!resetn)
    begin
        debug_end <= 1'b0;
    end
    else if(test_end && !debug_end)
    begin
        debug_end <= 1'b1;
        $display("==============================================================");
        $display("Test end!");
        #40;
        $fclose(trace_ref);
        if (global_err)
        begin
            $display("Fail!!!Total %d errors!",err_count);
        end
        else
        begin
            $display("----PASS!!!");
        end
	    $finish;
	end
end
endmodule

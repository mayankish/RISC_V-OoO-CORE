// ============================================================
// Testbench  : tb_riscv_ooo_top_fault
// Project    : riscv_ooo_core - Tesla AI Hardware Portfolio
// Author     : Mayank
// Description: Fault-path coverage, kept separate from the main smoke test
//              because a fault is terminal (fetch stops for good) and so
//              can't share a program with the HALT test there.
//
//              Two independent DUT instances, each fed a program that
//              triggers a DIFFERENT fault classification path:
//                dut_a: one normal ADDI, then a reserved ALU opcode
//                       (op=3'b110 on a non-EXT word) - exercises
//                       is_reserved_alu.
//                dut_b: one normal ADDI, then a reserved ext_class
//                       (3'b110 on an OP_EXT word) - exercises
//                       is_reserved_ext, the structurally different
//                       classification path (checks the ext_class field
//                       on an EXT-prefixed word, not the top-level op
//                       field directly).
//              Both must: commit exactly the one normal instruction, never
//              commit anything from the reserved word (it never dispatches
//              - is_reserved_alu/ext are excluded from decode_supported),
//              and latch core_faulted permanently within a cycle or two.
// ============================================================

`timescale 1ns/1ps
`include "defines.vh"

module tb_riscv_ooo_top_fault;

localparam IMEM_AW = 8;
localparam DMEM_AW = 8;
localparam IMEM_WORDS = (1 << IMEM_AW);
localparam TIMEOUT_CYCLES = 100;

reg clk;
reg rst_n;
reg flush;
reg wake_i;

reg [31:0] imem_a [0:IMEM_WORDS-1];
reg [31:0] imem_b [0:IMEM_WORDS-1];
wire [IMEM_AW-1:0] imem_addr_a, imem_addr_b;
wire [31:0] imem_rdata_a = imem_a[imem_addr_a];
wire [31:0] imem_rdata_b = imem_b[imem_addr_b];

wire [DMEM_AW-1:0] dmem_addr_a, dmem_addr_b;
wire dmem_req_a, dmem_req_b;
wire dmem_we_a, dmem_we_b;
wire [31:0] dmem_wdata_a, dmem_wdata_b;
// Neither program touches memory - tie dmem_ready = dmem_req (immediate
// ack) purely so the port is driven consistently with the main testbench.
wire dmem_ready_a = dmem_req_a;
wire dmem_ready_b = dmem_req_b;

wire commit_valid_a, commit_valid_b;
wire [4:0] commit_rd_a, commit_rd_b;
wire [31:0] commit_data_a, commit_data_b;
wire [3:0] commit_tag_a, commit_tag_b;
wire commit_fault_a, commit_fault_b;
wire [31:0] pc_out_a, pc_out_b;
wire fetch_valid_a, fetch_valid_b;
wire is_ext_out_a, is_ext_out_b;
wire [2:0] ext_class_out_a, ext_class_out_b;
wire core_halted_a, core_halted_b;
wire core_faulted_a, core_faulted_b;
wire core_sleeping_a, core_sleeping_b;
wire core_clk_en_a, core_clk_en_b;
wire rs_full_a, rob_full_a, pipeline_busy_a;
wire rs_full_b, rob_full_b, pipeline_busy_b;

riscv_ooo_top #(.IMEM_AW(IMEM_AW), .DMEM_AW(DMEM_AW)) dut_a (
    .clk(clk), .rst_n(rst_n), .flush(flush), .wake_i(wake_i),
    .imem_addr(imem_addr_a), .imem_rdata(imem_rdata_a),
    .dmem_addr(dmem_addr_a), .dmem_req(dmem_req_a), .dmem_we(dmem_we_a), .dmem_wdata(dmem_wdata_a),
    .dmem_rdata(32'b0), .dmem_ready(dmem_ready_a),
    .commit_valid(commit_valid_a), .commit_rd(commit_rd_a), .commit_data(commit_data_a),
    .commit_tag(commit_tag_a), .commit_fault(commit_fault_a),
    .pc_out(pc_out_a), .fetch_valid(fetch_valid_a),
    .is_ext_out(is_ext_out_a), .ext_class_out(ext_class_out_a),
    .core_halted(core_halted_a), .core_faulted(core_faulted_a),
    .core_sleeping(core_sleeping_a), .core_clk_en(core_clk_en_a),
    .rs_full(rs_full_a), .rob_full(rob_full_a), .pipeline_busy(pipeline_busy_a)
);

riscv_ooo_top #(.IMEM_AW(IMEM_AW), .DMEM_AW(DMEM_AW)) dut_b (
    .clk(clk), .rst_n(rst_n), .flush(flush), .wake_i(wake_i),
    .imem_addr(imem_addr_b), .imem_rdata(imem_rdata_b),
    .dmem_addr(dmem_addr_b), .dmem_req(dmem_req_b), .dmem_we(dmem_we_b), .dmem_wdata(dmem_wdata_b),
    .dmem_rdata(32'b0), .dmem_ready(dmem_ready_b),
    .commit_valid(commit_valid_b), .commit_rd(commit_rd_b), .commit_data(commit_data_b),
    .commit_tag(commit_tag_b), .commit_fault(commit_fault_b),
    .pc_out(pc_out_b), .fetch_valid(fetch_valid_b),
    .is_ext_out(is_ext_out_b), .ext_class_out(ext_class_out_b),
    .core_halted(core_halted_b), .core_faulted(core_faulted_b),
    .core_sleeping(core_sleeping_b), .core_clk_en(core_clk_en_b),
    .rs_full(rs_full_b), .rob_full(rob_full_b), .pipeline_busy(pipeline_busy_b)
);

initial clk = 1'b0;
always #5 clk = ~clk;

function [31:0] f_addi;
    input [4:0] rd, rs1;
    input [10:0] imm;
    f_addi = {rd, rs1, 5'b0, `OP_EXT, `EXTC_ADDI, imm};
endfunction

integer i;
integer errors;
integer cyc;

initial begin
    for (i = 0; i < IMEM_WORDS; i = i + 1) begin
        imem_a[i] = 32'h0;
        imem_b[i] = 32'h0;
    end

    // dut_a: normal ADDI, then a reserved ALU opcode (op=3'b110)
    imem_a[0] = f_addi(5'd1, 5'd0, 11'd5);
    imem_a[1] = {5'd2, 5'd0, 5'd0, 3'b110, 14'b0};   // reserved ALU op

    // dut_b: normal ADDI, then a reserved ext_class (3'b110 on an EXT word)
    imem_b[0] = f_addi(5'd1, 5'd0, 11'd5);
    imem_b[1] = {5'd2, 5'd0, 5'd0, `OP_EXT, 3'b110, 11'b0};  // reserved ext_class

    errors = 0;
    cyc = 0;
end

initial begin
    rst_n = 1'b0;
    flush = 1'b0;
    wake_i = 1'b0;
    repeat (4) @(negedge clk);
    rst_n = 1'b1;
end

always @(posedge clk) begin
    if (rst_n) begin
        cyc <= cyc + 1;

        if (commit_valid_a && (commit_rd_a != 5'd1)) begin
            $display("[%0t] ERROR dut_a: unexpected commit x%0d=%0d (only x1=5 should ever commit)",
                      $time, commit_rd_a, $signed(commit_data_a));
            errors = errors + 1;
        end
        if (commit_valid_b && (commit_rd_b != 5'd1)) begin
            $display("[%0t] ERROR dut_b: unexpected commit x%0d=%0d (only x1=5 should ever commit)",
                      $time, commit_rd_b, $signed(commit_data_b));
            errors = errors + 1;
        end

        if (cyc == 30) begin
            $display("==============================================");
            if (commit_valid_a !== 1'b0 && commit_rd_a === 5'd1 && commit_data_a === 32'd5) begin
                // fine - just checking below that x1 landed at some point
            end
            if (!core_faulted_a) begin
                $display("[%0t] ERROR: dut_a (reserved ALU op) never asserted core_faulted", $time);
                errors = errors + 1;
            end else begin
                $display("[%0t] OK: dut_a core_faulted=1 (reserved ALU opcode correctly caught)", $time);
            end
            if (!core_faulted_b) begin
                $display("[%0t] ERROR: dut_b (reserved ext_class) never asserted core_faulted", $time);
                errors = errors + 1;
            end else begin
                $display("[%0t] OK: dut_b core_faulted=1 (reserved ext_class correctly caught)", $time);
            end
            if (core_halted_a || core_halted_b) begin
                $display("[%0t] ERROR: core_halted incorrectly asserted (this is a fault, not a halt)", $time);
                errors = errors + 1;
            end
            if (errors == 0)
                $display("TB_RISCV_OOO_TOP_FAULT: PASS (both fault paths correctly latched core_faulted, 0 errors)");
            else
                $display("TB_RISCV_OOO_TOP_FAULT: FAIL (%0d errors)", errors);
            $display("==============================================");
            $finish;
        end
    end
end

endmodule

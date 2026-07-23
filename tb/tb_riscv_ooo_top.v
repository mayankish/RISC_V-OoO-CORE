// ============================================================
// Testbench  : tb_riscv_ooo_top
// Project    : riscv_ooo_core - Tesla AI Hardware Portfolio
// Author     : Mayank
// Description: Full-coverage smoke test for phase 2 (branch/JAL/load/store/
//              HALT/WFI). Hand-encodes a program exercising every wired
//              instruction class, self-checks the DYNAMIC commit sequence
//              (i.e. the order things actually commit in once branches/JAL
//              skip instructions - not just static program order), checks
//              dmem contents directly, and checks that x9 (used as the
//              destination of every "poison" instruction placed where a
//              branch/JAL is supposed to skip past it) is NEVER committed -
//              a strong, independent cross-check that redirects landed
//              exactly where intended, not just "close enough".
//
//              Covers: ADDI (incl. negative immediate - sign-extension with
//              payload[10]=1, not otherwise exercised by ALU-produced
//              negatives), ALU RAW-hazard chaining through the CDB/ROB-
//              lookup path (re-confirms Fix #1 through this fuller
//              frontend), BEQ taken, BNE not-taken, BLT taken with a signed
//              negative operand (exercises the true-comparator path in
//              branch_unit, not a subtraction-overflow-prone one), STORE +
//              LOAD with zero and non-zero offsets, JAL (link value +
//              redirect), WFI (sleep + external wake + resume), and HALT
//              (permanent stop).
//
//              NOT covered here (explicitly out of scope for this smoke
//              test, candidates for the fuller task #11 environment):
//              backward branches/loops, and the reserved-encoding fault
//              path (covered separately in tb_riscv_ooo_top_fault.v, since
//              a fault is terminal and can't share a program with the
//              HALT test above).
// ============================================================

`timescale 1ns/1ps
`include "defines.vh"

module tb_riscv_ooo_top;

localparam IMEM_AW    = 8;
localparam DMEM_AW    = 8;
localparam IMEM_WORDS = (1 << IMEM_AW);
localparam DMEM_WORDS = (1 << DMEM_AW);
localparam TIMEOUT_CYCLES = 500;

reg clk;
reg rst_n;
reg flush;
reg wake_i;

wire [IMEM_AW-1:0] imem_addr;
reg  [31:0]        imem [0:IMEM_WORDS-1];
wire [31:0]        imem_rdata;
assign imem_rdata = imem[imem_addr];

wire [DMEM_AW-1:0] dmem_addr;
wire               dmem_req;
wire               dmem_we;
wire [31:0]        dmem_wdata;
reg  [31:0]        dmem [0:DMEM_WORDS-1];
wire [31:0]        dmem_rdata;
assign dmem_rdata = dmem[dmem_addr];

// Same-cycle behavioral RAM: ack every request immediately (dmem_ready =
// dmem_req) so this testbench's timing is byte-for-byte unchanged from
// before the req/ready handshake was added for SoC/AXI integration.
wire dmem_ready = dmem_req;

always @(posedge clk) begin
    if (dmem_we && dmem_req) dmem[dmem_addr] <= dmem_wdata;
end

wire        commit_valid;
wire [4:0]  commit_rd;
wire [31:0] commit_data;
wire [3:0]  commit_tag;
wire        commit_fault;

wire [31:0] pc_out;
wire        fetch_valid;
wire        is_ext_out;
wire [2:0]  ext_class_out;

wire core_halted, core_faulted, core_sleeping, core_clk_en;
wire rs_full, rob_full, pipeline_busy;

riscv_ooo_top #(
    .IMEM_AW (IMEM_AW),
    .DMEM_AW (DMEM_AW)
) dut (
    .clk               (clk),
    .rst_n             (rst_n),
    .flush             (flush),
    .wake_i            (wake_i),
    .imem_addr         (imem_addr),
    .imem_rdata        (imem_rdata),
    .dmem_addr         (dmem_addr),
    .dmem_req          (dmem_req),
    .dmem_we           (dmem_we),
    .dmem_wdata        (dmem_wdata),
    .dmem_rdata        (dmem_rdata),
    .dmem_ready        (dmem_ready),
    .commit_valid      (commit_valid),
    .commit_rd         (commit_rd),
    .commit_data       (commit_data),
    .commit_tag        (commit_tag),
    .commit_fault      (commit_fault),
    .pc_out            (pc_out),
    .fetch_valid       (fetch_valid),
    .is_ext_out        (is_ext_out),
    .ext_class_out     (ext_class_out),
    .core_halted       (core_halted),
    .core_faulted      (core_faulted),
    .core_sleeping     (core_sleeping),
    .core_clk_en       (core_clk_en),
    .rs_full           (rs_full),
    .rob_full          (rob_full),
    .pipeline_busy     (pipeline_busy)
);

initial clk = 1'b0;
always #5 clk = ~clk;

// ---- Instruction encoders (mirror docs/isa_spec.md exactly) -------------
function [31:0] f_alu;
    input [4:0] rd, rs1, rs2;
    input [2:0] op;
    f_alu = {rd, rs1, rs2, op, 14'b0};
endfunction

function [31:0] f_addi;
    input [4:0] rd, rs1;
    input [10:0] imm;
    f_addi = {rd, rs1, 5'b0, `OP_EXT, `EXTC_ADDI, imm};
endfunction

function [31:0] f_branch;
    input [2:0] cond;
    input [4:0] rs1, rs2;
    input [10:0] offset;
    f_branch = {2'b0, cond, rs1, rs2, `OP_EXT, `EXTC_BRANCH, offset};
endfunction

function [31:0] f_store;
    input [4:0] rs1, rs2;
    input [10:0] offset;
    f_store = {5'b0, rs1, rs2, `OP_EXT, `EXTC_STORE, offset};
endfunction

function [31:0] f_load;
    input [4:0] rd, rs1;
    input [10:0] offset;
    f_load = {rd, rs1, 5'b0, `OP_EXT, `EXTC_LOAD, offset};
endfunction

function [31:0] f_jal;
    input [4:0] rd;
    input [10:0] offset;
    f_jal = {rd, 5'b0, 5'b0, `OP_EXT, `EXTC_JAL, offset};
endfunction

function [31:0] f_system;
    input [10:0] payload;
    f_system = {5'b0, 5'b0, 5'b0, `OP_EXT, `EXTC_SYSTEM, payload};
endfunction

// ---- Expected DYNAMIC commit trace (see header) -------------------------
localparam N_EXP = 18;
reg [4:0]         exp_rd   [0:N_EXP-1];
reg signed [31:0] exp_data [0:N_EXP-1];
integer commit_ptr;
integer errors;
integer cyc;
integer i;
reg [5:0] halt_grace;
initial halt_grace = 6'd0;

initial begin
    for (i = 0; i < IMEM_WORDS; i = i + 1) imem[i] = 32'h0;
    for (i = 0; i < DMEM_WORDS; i = i + 1) dmem[i] = 32'h0;

    imem[0]  = f_addi(5'd1, 5'd0, 11'd5);
    imem[1]  = f_addi(5'd2, 5'd0, 11'd7);
    imem[2]  = f_alu (5'd3, 5'd1, 5'd2, `OP_ADD);
    imem[3]  = f_addi(5'd1, 5'd0, 11'd100);
    imem[4]  = f_alu (5'd4, 5'd3, 5'd1, `OP_SUB);
    imem[5]  = f_alu (5'd5, 5'd3, 5'd2, `OP_ADD);
    imem[6]  = f_branch(`BR_BEQ, 5'd1, 5'd1, 11'd3);     // taken -> addr 10
    imem[7]  = f_addi(5'd9, 5'd0, 11'd999);               // poison
    imem[8]  = f_addi(5'd9, 5'd0, 11'd998);               // poison
    imem[9]  = f_addi(5'd9, 5'd0, 11'd997);               // poison
    imem[10] = f_addi(5'd6, 5'd0, 11'd42);
    imem[11] = f_branch(`BR_BNE, 5'd1, 5'd1, 11'd3);      // not taken -> falls to 12
    imem[12] = f_addi(5'd7, 5'd0, 11'd55);
    imem[13] = f_branch(`BR_BLT, 5'd4, 5'd1, 11'd2);      // x4(-88) < x1(100) signed -> taken -> addr 16
    imem[14] = f_addi(5'd9, 5'd0, 11'd996);               // poison
    imem[15] = f_addi(5'd9, 5'd0, 11'd995);               // poison
    imem[16] = f_addi(5'd8, 5'd0, 11'd77);
    imem[17] = f_addi(5'd10, 5'd0, 11'd3);
    imem[18] = f_addi(5'd11, 5'd0, 11'd123);
    imem[19] = f_store(5'd10, 5'd11, 11'd0);               // dmem[3] = 123
    imem[20] = f_addi(5'd12, 5'd0, 11'd0);
    imem[21] = f_load(5'd13, 5'd10, 11'd0);                // x13 = dmem[3]
    imem[22] = f_store(5'd10, 5'd1, 11'd2);                // dmem[5] = x1 = 100
    imem[23] = f_load(5'd14, 5'd10, 11'd2);                // x14 = dmem[5]
    imem[24] = f_jal(5'd15, 11'd2);                        // link=25, target -> addr 27
    imem[25] = f_addi(5'd9, 5'd0, 11'd994);                // poison
    imem[26] = f_addi(5'd9, 5'd0, 11'd993);                // poison
    imem[27] = f_addi(5'd16, 5'd0, 11'd88);
    imem[28] = f_addi(5'd22, 5'd0, -11'sd10);               // negative-immediate sign-extension
    imem[29] = f_system(`SYS_WFI);
    imem[30] = f_addi(5'd17, 5'd0, 11'd200);                // resumes here after wake
    imem[31] = f_system(`SYS_HALT);
    imem[32] = f_addi(5'd18, 5'd0, 11'd999);                // poison - should never fetch (halted)

    exp_rd[0]=5'd1;  exp_data[0]=32'sd5;
    exp_rd[1]=5'd2;  exp_data[1]=32'sd7;
    exp_rd[2]=5'd3;  exp_data[2]=32'sd12;
    exp_rd[3]=5'd1;  exp_data[3]=32'sd100;
    exp_rd[4]=5'd4;  exp_data[4]=-32'sd88;
    exp_rd[5]=5'd5;  exp_data[5]=32'sd19;
    exp_rd[6]=5'd6;  exp_data[6]=32'sd42;
    exp_rd[7]=5'd7;  exp_data[7]=32'sd55;
    exp_rd[8]=5'd8;  exp_data[8]=32'sd77;
    exp_rd[9]=5'd10; exp_data[9]=32'sd3;
    exp_rd[10]=5'd11; exp_data[10]=32'sd123;
    exp_rd[11]=5'd12; exp_data[11]=32'sd0;
    exp_rd[12]=5'd13; exp_data[12]=32'sd123;
    exp_rd[13]=5'd14; exp_data[13]=32'sd100;
    exp_rd[14]=5'd15; exp_data[14]=32'sd25;
    exp_rd[15]=5'd16; exp_data[15]=32'sd88;
    exp_rd[16]=5'd22; exp_data[16]=-32'sd10;
    exp_rd[17]=5'd17; exp_data[17]=32'sd200;
end

// ---- Reset ----------------------------------------------------------------
initial begin
    rst_n = 1'b0;
    flush = 1'b0;
    wake_i = 1'b0;
    commit_ptr = 0;
    errors = 0;
    cyc = 0;
    // Deassert away from a posedge - see prior race-condition note.
    repeat (4) @(negedge clk);
    rst_n = 1'b1;
end

// ---- WFI wake sequencing ----------------------------------------------
// Same race avoided as the reset deassertion above: driving wake_i via a
// blocking assign immediately after @(posedge clk) would race the DUT's
// own (posedge clk or negedge rst_n) sleep-state always block on that same
// edge. Using negedge-based timing instead settles wake_i mid-low-phase,
// well clear of any posedge sampling ambiguity, and holds it high across
// one full posedge so the DUT unambiguously sees it.
initial begin
    wait (core_sleeping === 1'b1);
    repeat (5) @(negedge clk);
    wake_i = 1'b1;
    @(negedge clk);
    wake_i = 1'b0;
end

// ---- Checker ------------------------------------------------------------
always @(posedge clk) begin
    if (rst_n) begin
        cyc <= cyc + 1;

        if (commit_fault) begin
            $display("[%0t] ERROR: commit_fault asserted unexpectedly (tag=%0d)", $time, commit_tag);
            errors = errors + 1;
        end

        if (commit_valid && (commit_rd == 5'd9)) begin
            $display("[%0t] ERROR: x9 (poison target) was committed with data=%0d - a branch/JAL redirect landed wrong", $time, $signed(commit_data));
            errors = errors + 1;
        end

        if (commit_valid) begin
            if (commit_ptr < N_EXP) begin
                if ((commit_rd !== exp_rd[commit_ptr]) || ($signed(commit_data) !== exp_data[commit_ptr])) begin
                    $display("[%0t] MISMATCH commit #%0d: got x%0d=%0d, expected x%0d=%0d",
                             $time, commit_ptr, commit_rd, $signed(commit_data),
                             exp_rd[commit_ptr], exp_data[commit_ptr]);
                    errors = errors + 1;
                end else begin
                    $display("[%0t] OK commit #%0d: x%0d <= %0d", $time, commit_ptr, commit_rd, $signed(commit_data));
                end
                commit_ptr = commit_ptr + 1;
            end
        end

        if (core_halted) begin
            halt_grace <= halt_grace + 1;
            if (halt_grace == 6'd10) begin
                $display("==============================================");
                if (dmem[3] !== 32'd123) begin
                    $display("[%0t] ERROR: dmem[3] = %0d, expected 123", $time, dmem[3]);
                    errors = errors + 1;
                end
                if (dmem[5] !== 32'd100) begin
                    $display("[%0t] ERROR: dmem[5] = %0d, expected 100", $time, dmem[5]);
                    errors = errors + 1;
                end
                if (commit_ptr != N_EXP) begin
                    $display("[%0t] ERROR: expected %0d commits, saw %0d", $time, N_EXP, commit_ptr);
                    errors = errors + 1;
                end
                if (errors == 0)
                    $display("TB_RISCV_OOO_TOP: PASS (%0d/%0d commits verified, dmem verified, core_halted=1, 0 errors)", commit_ptr, N_EXP);
                else
                    $display("TB_RISCV_OOO_TOP: FAIL (%0d errors)", errors);
                $display("==============================================");
                $finish;
            end
        end

        if (cyc >= TIMEOUT_CYCLES) begin
            $display("[%0t] ERROR: TIMEOUT after %0d cycles - only %0d/%0d expected commits, core_halted=%0b, core_sleeping=%0b",
                     $time, TIMEOUT_CYCLES, commit_ptr, N_EXP, core_halted, core_sleeping);
            errors = errors + 1;
            $display("==============================================");
            $display("TB_RISCV_OOO_TOP: FAIL (%0d errors)", errors);
            $display("==============================================");
            $finish;
        end
    end
end

initial begin
    $dumpfile("tb_riscv_ooo_top.vcd");
    $dumpvars(0, tb_riscv_ooo_top);
end

endmodule

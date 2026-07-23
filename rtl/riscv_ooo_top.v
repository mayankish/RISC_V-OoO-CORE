// ============================================================
// Module      : riscv_ooo_top
// Project     : riscv_ooo_core - Tesla AI Hardware Portfolio
// Author      : Mayank
// Description : Full integration: fetch_unit + decoder + branch_unit +
//               lsu_frontend + the fixed/extended ooo_top backend.
//
// Design summary (see README.md for the full rationale/trade-off writeup):
//   - ALU-class, ADDI, JAL, and a resolved LOAD dispatch into the backend
//     (rename -> RS -> ALU -> CDB -> ROB -> commit), inheriting the
//     RAT-deadlock (Fix #1), multi-FU CDB (Fix #2), and fault-bit (Fix #3)
//     infrastructure unchanged.
//   - BRANCH and STORE never dispatch into the backend at all - they are
//     resolved entirely in the frontend (branch_unit / lsu_frontend), using
//     a read-only tap (ooo_top's src1/src2 ports) into the same RAT
//     readiness logic Fix #1 already made correct, with zero new hazard
//     logic added to the backend itself.
//   - SYSTEM/HALT stops fetch permanently. SYSTEM/WFI sleeps until an
//     external `wake_i` pulse. Any reserved encoding (ext_class 110/111,
//     or ALU op 110) is treated as a fault and also stops fetch
//     permanently, distinctly flagged from a clean halt.
//   - None of this is speculative: fetch never advances past an
//     unresolved branch or an unresolved load/store, so ooo_top's `flush`
//     port is never asserted by this design (misprediction recovery has
//     nothing to recover from, because nothing wrong was ever fetched).
//
// Load write-back sequencing (why a load takes an extra cycle):
//   A load's register write has to go through ooo_top's normal dispatch
//   path (so later instructions see Fix #1's correctness for free), which
//   means ooo_top's `instr` port needs its rs1 field forced to x0 (so the
//   ALU computes 0 + loaded_value = loaded_value). But the SAME cycle the
//   load's real base register becomes ready, that same `instr` port's rs1
//   field is what's being used to compute the memory address in the first
//   place - the port cannot simultaneously mean "the real base register"
//   (for address computation) and "x0" (for the write-back dispatch).
//   Splitting it into two cycles - resolve-and-latch, then dispatch-the-
//   latched-value - removes the conflict entirely: only one meaning of
//   the `instr` port is needed per cycle. Stores don't have this problem
//   (they never dispatch into the backend, so their instr word is never
//   rewritten) and JAL doesn't either (its "source" is the PC, already
//   available with no read-then-rewrite conflict).
// ============================================================

`include "defines.vh"

module riscv_ooo_top #(
    parameter RS_DEPTH   = `RS_DEPTH,
    parameter ROB_DEPTH  = `ROB_DEPTH,
    parameter IMEM_AW    = 8,           // word-address width exposed to imem
    parameter DMEM_AW    = 8            // word-address width exposed to dmem
)(
    input  wire            clk,
    input  wire            rst_n,
    input  wire            flush,        // passthrough to ooo_top; unused by
                                          // this phase's branch/load/store
                                          // handling (see header) - reserved
                                          // for future trap/SoC-level use

    input  wire             wake_i,       // external wake/interrupt for WFI

    // Instruction memory interface (word-addressed, combinational read)
    output wire [IMEM_AW-1:0] imem_addr,
    input  wire [31:0]        imem_rdata,

    // Data memory interface (word-addressed; req/ready handshake so this
    // can reach either a same-cycle behavioral RAM - tie dmem_ready =
    // dmem_req - or a genuinely multi-cycle AXI-backed peripheral through
    // an adapter, e.g. for SoC integration. See lsu_frontend.v's header.)
    output wire [DMEM_AW-1:0] dmem_addr,
    output wire                dmem_req,
    output wire                dmem_we,
    output wire [31:0]         dmem_wdata,
    input  wire [31:0]         dmem_rdata,
    input  wire                dmem_ready,

    // Commit visibility (external observe / testbench checking)
    output wire        commit_valid,
    output wire [4:0]  commit_rd,
    output wire [31:0] commit_data,
    output wire [3:0]  commit_tag,
    output wire        commit_fault,

    // Fetch/decode observability
    output wire [31:0] pc_out,
    output wire        fetch_valid,
    output wire        is_ext_out,
    output wire [2:0]  ext_class_out,

    // Core status
    output wire        core_halted,   // SYS_HALT seen - fetch stopped for good
    output wire        core_faulted,  // reserved/illegal encoding - fetch stopped for good
    output wire        core_sleeping, // in WFI, waiting for wake_i
    output wire        core_clk_en,   // advisory: 0 while halted/faulted/asleep -
                                       // an SoC-level power manager's coarse
                                       // whole-core clock-gate input (see README)

    // Backend status
    output wire        rs_full,
    output wire        rob_full,
    output wire        pipeline_busy
);

localparam DW = `DATA_WIDTH;

// ---- Fetch --------------------------------------------------------------
wire [31:0] pc_w;
wire        fetch_valid_w;
wire        fetch_stall;
wire        redirect_valid_w;
wire [31:0] redirect_target_w;

fetch_unit #(.PC_W(32)) u_fetch (
    .clk             (clk),
    .rst_n           (rst_n),
    .stall           (fetch_stall),
    .redirect_valid  (redirect_valid_w),
    .redirect_target (redirect_target_w),
    .pc              (pc_w),
    .instr_valid     (fetch_valid_w)
);

assign imem_addr = pc_w[IMEM_AW-1:0];

// ---- Decode ---------------------------------------------------------------
wire [31:0] decoded_instr;
wire        decode_use_imm;
wire [31:0] decode_imm;
wire        decode_is_alu;
wire        decode_is_ext;
wire [2:0]  decode_ext_class;
wire        decode_supported;
wire [4:0]  decode_ext_rd, decode_ext_rs1, decode_ext_rs2;
wire [2:0]  decode_ext_subcode;
wire [31:0] decode_ext_imm;

decoder u_decoder (
    .instr_in      (imem_rdata),
    .instr_out     (decoded_instr),
    .instr_use_imm (decode_use_imm),
    .instr_imm     (decode_imm),
    .is_alu        (decode_is_alu),
    .is_ext        (decode_is_ext),
    .ext_class     (decode_ext_class),
    .supported     (decode_supported),
    .ext_rd        (decode_ext_rd),
    .ext_rs1       (decode_ext_rs1),
    .ext_rs2       (decode_ext_rs2),
    .ext_subcode   (decode_ext_subcode),
    .ext_imm       (decode_ext_imm)
);

wire [2:0] top_op = imem_rdata[`OP_HI:`OP_LO];

// ---- Classification -------------------------------------------------------
wire is_branch = decode_is_ext && (decode_ext_class == `EXTC_BRANCH);
wire is_load   = decode_is_ext && (decode_ext_class == `EXTC_LOAD);
wire is_store  = decode_is_ext && (decode_ext_class == `EXTC_STORE);
wire is_jal    = decode_is_ext && (decode_ext_class == `EXTC_JAL);
wire is_system = decode_is_ext && (decode_ext_class == `EXTC_SYSTEM);

wire is_reserved_ext = decode_is_ext && (decode_ext_class == 3'b110 || decode_ext_class == 3'b111);
wire is_reserved_alu = decode_is_alu && (top_op == 3'b110);

wire is_sys_halt  = is_system && (decode_ext_imm == `SYS_HALT);
wire is_sys_wfi   = is_system && (decode_ext_imm == `SYS_WFI);
wire is_sys_other = is_system && !is_sys_halt && !is_sys_wfi;   // undefined payload -> fault

assign is_ext_out    = decode_is_ext;
assign ext_class_out = decode_ext_class;

// ---- Backend (ooo_top) source-operand tap + dispatch ----------------------
wire        src1_ready, src2_ready;
wire [DW-1:0] src1_data, src2_data;
wire        backend_instr_ready;

// [load write-back state] - see header for why this needs two cycles.
reg        load_wb_pending_r;
reg [DW-1:0] load_wb_value_r;

wire        load_ready_comb;   // is_load && rs1_ready (from lsu_frontend)
wire        store_ready_comb;
wire        lsu_op_done;       // 1-cycle pulse: dmem_ready seen for the outstanding request
wire [DW-1:0] dmem_addr_full;

lsu_frontend #(.DW(DW)) u_lsu (
    .clk         (clk),
    .rst_n       (rst_n),
    .is_load     (is_load),
    .is_store    (is_store),
    .imm         (decode_ext_imm),
    .rs1_ready   (src1_ready),
    .rs1_data    (src1_data),
    .rs2_ready   (src2_ready),
    .rs2_data    (src2_data),
    .load_ready  (load_ready_comb),
    .store_ready (store_ready_comb),
    .op_done     (lsu_op_done),
    .dmem_addr   (dmem_addr_full),
    .dmem_req    (dmem_req),
    .dmem_we     (dmem_we),
    .dmem_wdata  (dmem_wdata),
    .dmem_rdata  (dmem_rdata),
    .dmem_ready  (dmem_ready),
    .load_result ()   // read directly via dmem_rdata below at latch time
);

assign dmem_addr = dmem_addr_full[DMEM_AW-1:0];

wire branch_stall;
wire branch_redirect_valid;
wire [DW-1:0] branch_redirect_target;

branch_unit #(.DW(DW)) u_branch (
    .is_branch       (is_branch),
    .cond            (decode_ext_subcode),
    .imm             (decode_ext_imm),
    .pc              (pc_w),
    .rs1_ready       (src1_ready),
    .rs1_data        (src1_data),
    .rs2_ready       (src2_ready),
    .rs2_data        (src2_data),
    .stall           (branch_stall),
    .redirect_valid  (branch_redirect_valid),
    .redirect_target (branch_redirect_target)
);

// Load write-back: latch the read value the cycle it first resolves, then
// hold it until the rewritten "ADD rd,x0,imm=<value>" successfully
// dispatches (may itself stall on rs_full/rob_full - held via
// load_wb_pending_r until backend_instr_ready).
wire load_dispatch_fire = is_load && load_wb_pending_r && backend_instr_ready;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        load_wb_pending_r <= 1'b0;
        load_wb_value_r   <= {DW{1'b0}};
    end else if (is_load) begin
        if (!load_wb_pending_r && lsu_op_done) begin
            load_wb_pending_r <= 1'b1;
            load_wb_value_r   <= dmem_rdata;   // valid this cycle: op_done means dmem_ready this cycle
        end else if (load_wb_pending_r && backend_instr_ready) begin
            load_wb_pending_r <= 1'b0;
        end
    end else begin
        load_wb_pending_r <= 1'b0;
    end
end

wire jal_dispatch_fire = is_jal && backend_instr_ready;

// ---- WFI sleep state --------------------------------------------------
reg sleeping_r;
wire wfi_proceed = sleeping_r && wake_i;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        sleeping_r <= 1'b0;
    end else if (sleeping_r && wake_i) begin
        sleeping_r <= 1'b0;
    end else if (is_sys_wfi && !sleeping_r) begin
        sleeping_r <= 1'b1;
    end
end

wire wfi_stall = is_sys_wfi && !wfi_proceed;

// ---- Halt / fault (sticky) ---------------------------------------------
reg core_halted_r, core_faulted_r;
wire halt_this_cycle  = fetch_valid_w && is_sys_halt;
wire fault_this_cycle = fetch_valid_w && (is_reserved_ext || is_reserved_alu || is_sys_other);

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        core_halted_r  <= 1'b0;
        core_faulted_r <= 1'b0;
    end else begin
        if (halt_this_cycle)  core_halted_r  <= 1'b1;
        if (fault_this_cycle) core_faulted_r <= 1'b1;
    end
end

assign core_halted  = core_halted_r;
assign core_faulted = core_faulted_r;
assign core_sleeping = sleeping_r;
assign core_clk_en  = !core_halted_r && !core_faulted_r && !sleeping_r;

wire terminal_stall = is_sys_halt || is_reserved_ext || is_reserved_alu || is_sys_other
                      || core_halted_r || core_faulted_r;

// ---- Backend instruction / immediate mux --------------------------------
wire [31:0] jal_link_value = pc_w + 32'd1;
wire [31:0] jal_target     = pc_w + 32'd1 + decode_ext_imm;

wire        load_wb_now = is_load && load_wb_pending_r;

wire [31:0] backend_instr =
      (is_jal || load_wb_now) ? {decode_ext_rd, 5'b0, 5'b0, `OP_ADD, 14'b0}
                               : decoded_instr;

wire backend_use_imm = (is_jal || load_wb_now) ? 1'b1 : decode_use_imm;

wire [31:0] backend_imm =
      is_jal    ? jal_link_value
    : load_wb_now ? load_wb_value_r
                  : decode_imm;

wire backend_dispatch_valid =
      fetch_valid_w && (decode_supported || is_jal || load_wb_now);

ooo_top #(
    .RS_DEPTH  (RS_DEPTH),
    .ROB_DEPTH (ROB_DEPTH)
) u_ooo (
    .clk            (clk),
    .rst_n          (rst_n),
    .flush          (flush),
    .instr_valid    (backend_dispatch_valid),
    .instr          (backend_instr),
    .instr_ready    (backend_instr_ready),
    .instr_use_imm  (backend_use_imm),
    .instr_imm      (backend_imm),
    .src1_ready     (src1_ready),
    .src1_data      (src1_data),
    .src2_ready     (src2_ready),
    .src2_data      (src2_data),
    .commit_valid   (commit_valid),
    .commit_rd      (commit_rd),
    .commit_data    (commit_data),
    .commit_tag     (commit_tag),
    .commit_fault   (commit_fault),
    .rs_full        (rs_full),
    .rob_full       (rob_full),
    .pipeline_busy  (pipeline_busy)
);

// ---- Fetch stall composition ---------------------------------------------
wire generic_backend_stall = !backend_instr_ready;

// NOTE on the is_load case: fetch must stay stalled through ALL THREE
// phases of a load now - (1) waiting for rs1 (lsu_frontend's operands_ready),
// (2) waiting for dmem_ready once the request is issued (lsu_op_done), and
// (3) waiting for the backend to accept the write-back dispatch
// (backend_instr_ready). Using !load_dispatch_fire (rather than checking
// each phase's condition separately) means there is exactly one condition,
// "the write-back actually fired this cycle," that ungates the stall -
// avoiding a one-cycle window where an earlier phase's completion looks
// like "ready to advance" before the next phase has actually happened.
// NOTE on is_store: now gated on lsu_op_done (dmem_ready seen), not just
// operands being ready, for the same reason - dmem may take multiple
// cycles once the request is issued.
assign fetch_stall =
      terminal_stall ? 1'b1
    : is_sys_wfi      ? wfi_stall
    : is_branch       ? branch_stall
    : is_store        ? !lsu_op_done
    : is_load         ? !load_dispatch_fire
                       : generic_backend_stall;   // ALU, ADDI, JAL

// ---- Redirect composition -------------------------------------------------
assign redirect_valid_w =
      is_branch ? branch_redirect_valid
    : is_jal    ? jal_dispatch_fire
                : 1'b0;

assign redirect_target_w = is_branch ? branch_redirect_target : jal_target;

// ---- Observability ---------------------------------------------------------
assign pc_out      = pc_w;
assign fetch_valid = fetch_valid_w;

endmodule

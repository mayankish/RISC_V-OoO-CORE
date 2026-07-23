// ============================================================
// Module      : lsu_frontend
// Project     : riscv_ooo_core - Tesla AI Hardware Portfolio
// Author      : Mayank
// Description : Frontend-resolved load/store address/data path (task #7).
//
// [CHANGED for SoC integration] dmem_req/dmem_ready handshake added:
//   The original version assumed a memory that responds in the same cycle
//   (fine for a testbench's behavioral RAM, wrong for anything reached
//   through an AXI4-Lite adapter/crossbar, which is inherently multi-
//   cycle). dmem_req is asserted the cycle operands are ready and HELD
//   until dmem_ready pulses back (exactly one cycle, coincident with
//   dmem_rdata being valid for a load). A behavioral RAM caller can still
//   get the original single-cycle behavior by simply tying
//   dmem_ready = dmem_req (see riscv_ooo_core's own testbenches, updated
//   to do exactly this - their timing is unchanged).
//
//   req_outstanding_r exists so dmem_req stays asserted across however
//   many cycles the memory takes, without re-triggering a second request
//   for the same instruction once dmem_ready arrives: op_done (pulsed
//   exactly the cycle dmem_ready is seen) is what the caller
//   (riscv_ooo_top.v) uses to advance to the next state/unstall fetch -
//   NOT operands_ready, which would still read true for one extra cycle
//   after completion (same instruction, same operands, fetch hasn't
//   advanced yet) and would otherwise cause a spurious second request.
//   This is the same class of one-cycle-completion-edge care as the
//   load write-back sequencing in riscv_ooo_top.v - see that file's
//   header for the fuller pattern.
//
//   Design choice, matching branch_unit's rationale: rather than build a
//   true second OoO-issued functional unit (structural surgery on
//   already-verified, timing-sensitive modules), loads and stores are
//   resolved in-order in the fetch stage - now also correctly tolerating
//   multi-cycle memory, which the SoC's AXI-backed peripherals require.
// ============================================================

`include "defines.vh"

module lsu_frontend #(
    parameter DW = `DATA_WIDTH
)(
    input  wire            clk,
    input  wire            rst_n,

    input  wire            is_load,
    input  wire            is_store,
    input  wire [DW-1:0]   imm,        // sign-extended signed word offset

    input  wire            rs1_ready,  // base register
    input  wire [DW-1:0]   rs1_data,
    input  wire            rs2_ready,  // store data register (don't-care for load)
    input  wire [DW-1:0]   rs2_data,

    output wire             load_ready,   // is_load && rs1_ready (operands ready, request may not be issued/done yet)
    output wire             store_ready,  // is_store && rs1_ready && rs2_ready
    output wire             op_done,      // 1-cycle pulse: dmem_ready seen for the outstanding request

    // Data memory interface (word-addressed; req/ready handshake - see header)
    output wire [DW-1:0]    dmem_addr,
    output wire             dmem_req,
    output wire             dmem_we,
    output wire [DW-1:0]    dmem_wdata,
    input  wire [DW-1:0]    dmem_rdata,
    input  wire             dmem_ready,

    output wire [DW-1:0]    load_result   // = dmem_rdata, valid the cycle op_done is high
);

assign load_ready  = is_load  && rs1_ready;
assign store_ready = is_store && rs1_ready && rs2_ready;

wire operands_ready = load_ready || store_ready;

reg req_outstanding_r;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        req_outstanding_r <= 1'b0;
    end else if (!req_outstanding_r && operands_ready) begin
        req_outstanding_r <= 1'b1;
    end else if (req_outstanding_r && dmem_ready) begin
        req_outstanding_r <= 1'b0;
    end
end

assign dmem_req    = operands_ready || req_outstanding_r;
assign dmem_addr   = rs1_data + imm;   // word address = base + signed word offset
assign dmem_we     = store_ready || (req_outstanding_r && is_store);
assign dmem_wdata  = rs2_data;

assign op_done      = req_outstanding_r && dmem_ready;
assign load_result  = dmem_rdata;

endmodule

// ============================================================
// Module      : decoder
// Project     : riscv_ooo_core - Tesla AI Hardware Portfolio
// Author      : Mayank
// Description : Decodes a raw 32-bit fetched word against the custom ISA
//               (docs/isa_spec.md). Two jobs:
//
//               1. Field extraction for every extended (OP_EXT) class, so
//                  later phases (branch/load/store/jal/system - tasks
//                  #6/#7/#8) can consume ext_rd/ext_rs1/ext_rs2/ext_imm/
//                  ext_subcode directly without re-deriving them.
//
//               2. For the ONE extended class wired into the backend this
//                  phase (EXTC_ADDI), rewrites the instruction into an
//                  ALU-class ADD word (instr_out) and drives instr_use_imm/
//                  instr_imm so ooo_top's source-2 mux supplies the
//                  immediate instead of a register read. rs2 is left as 0
//                  in instr_out since ooo_top ignores dec_rs2 whenever
//                  instr_use_imm=1.
//
//               `supported` is 0 for any fetched word this phase cannot
//               yet dispatch (BRANCH/LOAD/STORE/JAL/SYSTEM) — the wrapper
//               (riscv_ooo_top) uses it to stall fetch rather than push
//               garbage into the backend. This is an honest, explicit
//               "not yet wired" signal, not silent misbehavior.
// ============================================================

`include "defines.vh"

module decoder (
    input  wire [31:0] instr_in,

    // Feed straight into ooo_top
    output wire [31:0] instr_out,
    output wire        instr_use_imm,
    output wire [31:0] instr_imm,

    // Classification
    output wire        is_alu,       // op != OP_EXT: passes straight through
    output wire        is_ext,       // op == OP_EXT
    output wire [2:0]  ext_class,    // valid when is_ext
    output wire        supported,    // 1 = safe to dispatch this phase

    // Decoded EXT fields (for tasks #6/#7/#8 - not yet consumed here)
    output wire [4:0]  ext_rd,
    output wire [4:0]  ext_rs1,
    output wire [4:0]  ext_rs2,
    output wire [2:0]  ext_subcode,  // branch condition code (= ext_rd[2:0])
    output wire [31:0] ext_imm       // sign-extended 11-bit payload
);

wire [2:0] op = instr_in[`OP_HI:`OP_LO];

assign is_ext    = (op == `OP_EXT);
assign is_alu    = !is_ext;
assign ext_class = instr_in[`EXT_HI:`EXT_LO];

wire [10:0] payload = instr_in[`PAYLOAD_HI:`PAYLOAD_LO];
assign ext_imm = {{21{payload[10]}}, payload};   // sign-extend 11 -> 32

assign ext_rd      = instr_in[`RD_HI:`RD_LO];
assign ext_rs1     = instr_in[`RS1_HI:`RS1_LO];
assign ext_rs2     = instr_in[`RS2_HI:`RS2_LO];
assign ext_subcode = ext_rd[2:0];

// Only ADDI is dispatched into the backend this phase. is_alu is a
// STRUCTURAL classification (op != OP_EXT) and deliberately includes the
// reserved ALU opcode 3'b110 - excluding it here, not by narrowing is_alu
// itself, keeps is_alu's meaning consistent for any other consumer.
// Bug found via tb_riscv_ooo_top_fault.v: without this exclusion, a
// reserved-opcode word was still `supported`, so it dispatched into the
// backend every cycle fetch was stalled on it (fetch stalls on it forever
// once core_faulted latches, but nothing was stopping ooo_top from
// re-dispatching the same un-advancing instruction each of those cycles) -
// repeatedly allocating new ROB entries for the identical word and
// committing a bogus default-case ALU result each time.
wire is_reserved_alu_op = is_alu && (op == 3'b110);
assign supported = (is_alu && !is_reserved_alu_op) || (is_ext && (ext_class == `EXTC_ADDI));

assign instr_use_imm = is_ext && (ext_class == `EXTC_ADDI);
assign instr_imm     = ext_imm;

// ADDI -> ALU-class ADD: keep rd/rs1, zero rs2 (unused - s2 is the immediate).
assign instr_out = instr_use_imm
    ? {ext_rd, ext_rs1, 5'b0, `OP_ADD, 14'b0}
    : instr_in;

endmodule

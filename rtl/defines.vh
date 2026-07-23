// ============================================================
// File        : defines.vh
// Project     : riscv_ooo_core - Tesla AI Hardware Portfolio
// Author      : Mayank
// Description : Global parameters, opcode encodings, and state constants.
//               Extends ooo_issue_queue's defines.vh (base ALU-class fields
//               and opcodes are byte-for-byte unchanged) with a custom
//               instruction-class extension used by fetch_unit / decoder.
//               RS_DEPTH may be overridden at compile time: -DRS_DEPTH=4
// ============================================================

`ifndef DEFINES_VH
`define DEFINES_VH

// ---- Overridable via -D flag ----------------------------------------
`ifndef RS_DEPTH
`define RS_DEPTH 8        // Reservation station entries (4/8/16)
`endif

`ifndef ROB_DEPTH
`define ROB_DEPTH 16      // Reorder buffer entries (must be power of 2)
`endif

// ---- Fixed microarchitecture parameters ----------------------------
`define NUM_REGS    32    // Architectural register count (x0-x31)
`define DATA_WIDTH  32    // Data path width in bits
`define NUM_FU      1     // Functional unit count (ALU only; LSU is FU-1,
                           // still tied off at the ooo_top level until the
                           // load/store unit itself is built - see README)
`define ALU_LATENCY 2     // Pipeline stages inside integer_alu

// Derived widths (hardcoded to avoid $clog2 in preprocessor)
`define TAG_WIDTH   4     // log2(ROB_DEPTH=16): ROB index / rename tag
`define REG_ADDR_W  5     // log2(NUM_REGS=32): architectural register index
`define SEQW        8     // Dispatch sequence counter width (age tracking)

// ---- Base instruction encoding (32-bit, ALU class) ------------------
// UNCHANGED from ooo_issue_queue - every existing ALU instruction remains
// byte-for-byte valid input to this project's copy of ooo_top.
//   [31:27] rd   [26:22] rs1   [21:17] rs2   [16:14] opcode   [13:0] imm
`define RD_HI   31
`define RD_LO   27
`define RS1_HI  26
`define RS1_LO  22
`define RS2_HI  21
`define RS2_LO  17
`define OP_HI   16
`define OP_LO   14

// ---- ALU opcode encoding (op field, unchanged) ----------------------
`define OP_ADD  3'b000
`define OP_SUB  3'b001
`define OP_MUL  3'b010
`define OP_SHL  3'b011
`define OP_SHR  3'b100
// 3'b101        - reserved by the base project's own spec; repurposed
//                 below as the EXT escape code for this project only.
// 3'b110        - reserved (unused by either project)
`define OP_NOP  3'b111   // No-op / architectural x0 write sink

`define OP_EXT  3'b101   // [NEW] escape: bits[13:11]=ext_class, bits[10:0]=payload
                          // See docs/isa_spec.md for the full derivation.

// ---- RS entry valid / invalid flag ---------------------------------
`define RS_INVALID 1'b0
`define RS_VALID   1'b1

// ============================================================
// [NEW] Extended instruction classes (op == OP_EXT)
//   bits[13:11] = ext_class     bits[10:0] = class-specific payload
// ============================================================
`define EXT_HI   13
`define EXT_LO   11
`define PAYLOAD_HI 10
`define PAYLOAD_LO 0
`define IMM_W    11        // width of the sign-extended immediate/offset field

`define EXTC_BRANCH 3'b000  // rd[2:0]=condition, rs1/rs2=compare operands, payload=signed word offset
`define EXTC_LOAD   3'b001  // rd=dest, rs1=base, payload=signed word offset (rs2 unused)
`define EXTC_STORE  3'b010  // rs1=base, rs2=data, payload=signed word offset (rd unused)
`define EXTC_ADDI   3'b011  // rd=dest, rs1=base, payload=signed immediate (rs2 unused)
`define EXTC_JAL    3'b100  // rd=link (gets PC+1), payload=signed word offset (rs1/rs2 unused)
`define EXTC_SYSTEM 3'b101  // rd/rs1/rs2 unused, payload=cause code (SYS_HALT / SYS_WFI)
// 3'b110, 3'b111 reserved for future CSR / trap-return work (see task list)

// ---- Branch condition codes (rd[2:0] field of an EXTC_BRANCH instr) -
`define BR_BEQ  3'b000   // branch if (rs1 - rs2) == 0
`define BR_BNE  3'b001   // branch if (rs1 - rs2) != 0
`define BR_BLT  3'b010   // branch if rs1 <  rs2 (signed)
`define BR_BGE  3'b011   // branch if rs1 >= rs2 (signed)
`define BR_BLTU 3'b100   // branch if rs1 <  rs2 (unsigned)
`define BR_BGEU 3'b101   // branch if rs1 >= rs2 (unsigned)

// ---- SYSTEM payload cause codes --------------------------------------
`define SYS_HALT 11'd1   // stop fetch / signal program completion to the TB
`define SYS_WFI  11'd2   // sleep until next interrupt (task #9 - not yet wired)

`endif // DEFINES_VH

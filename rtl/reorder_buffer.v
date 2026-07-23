// ============================================================
// Module      : reorder_buffer
// Project     : Out-of-Order Issue Queue - Tesla AI Hardware Portfolio
// Author      : Mayank
// Description : Circular ROB for in-order commit of out-of-order results.
//               Head commits only when the oldest entry is marked complete.
//               alloc_tag (= tail) is the rename tag handed to the RS/RAT.
//
// Changes (Fix #1 + Fix #3):
//   - Added lk1/lk2 combinational tag-lookup ports so the RAT can check
//     whether a tag's ROB entry is already complete at dispatch time.
//     This closes the deadlock window where a consumer dispatches after the
//     CDB broadcast but before commit and sees a stale in-flight tag.
//   - Added rob_entry_fault[] per-entry bit, alloc_fault input, and
//     commit_fault output.  commit_ack is now gated externally on
//     !commit_fault so a faulting head stalls until an external flush.
//
// Parameters  : DEPTH=16, DW=32, TW=4, AW=5, OW=3
// ============================================================

`include "defines.vh"

module reorder_buffer #(
    parameter DEPTH = `ROB_DEPTH,   // ROB capacity (power of 2)
    parameter DW    = `DATA_WIDTH,  // Data width
    parameter TW    = `TAG_WIDTH,   // Tag / pointer width = log2(DEPTH)
    parameter AW    = `REG_ADDR_W,  // Architectural register address width
    parameter OW    = 3             // Opcode width
)(
    input  wire           clk,          // System clock
    input  wire           rst_n,        // Active-low synchronous reset
    input  wire           flush,        // Flush: drain entire ROB

    // Allocation port (from ooo_top at dispatch)
    input  wire           alloc_valid,  // Dispatch is allocating a ROB slot
    input  wire [AW-1:0]  alloc_rd,     // Destination architectural register
    input  wire [OW-1:0]  alloc_opcode, // Opcode (for later extension: store/branch)
    input  wire           alloc_fault,  // [Fix #3] Instruction known to fault at dispatch
    output wire [TW-1:0]  alloc_tag,    // Assigned ROB tag (= current tail index)
    output wire           rob_full,     // ROB cannot accept a new entry

    // Completion port (from CDB - marks an in-flight entry as done)
    input  wire           complete_valid, // A result is ready to be written
    input  wire [TW-1:0]  complete_tag,   // ROB slot receiving the result
    input  wire [DW-1:0]  complete_data,  // Result value

    // Commit port (to RAT and external observe)
    output wire           commit_valid, // Head entry is complete; committing now
    output wire [AW-1:0]  commit_rd,    // Architectural register to update
    output wire [DW-1:0]  commit_data,  // Value to write into the register file
    output wire [TW-1:0]  commit_tag,   // ROB tag being retired (WAW guard for RAT)
    output wire           commit_fault, // [Fix #3] Head entry faulted; suppress arch write
    input  wire           commit_ack,   // Commit acknowledged; advance head

    // [Fix #1] Combinational tag-lookup ports (for RAT dispatch-time readiness check)
    // Given a tag, returns whether that ROB entry is already complete + its data.
    // This lets the RAT declare a source operand ready even if the CDB already
    // broadcast but commit hasn't fired yet (closing the post-CDB deadlock window).
    input  wire [TW-1:0]  lk1_tag,      // Tag to look up (rs1 rename tag)
    output wire           lk1_complete,  // 1 = that entry already has its result
    output wire [DW-1:0]  lk1_data,     // Result data for lk1_tag

    input  wire [TW-1:0]  lk2_tag,      // Tag to look up (rs2 rename tag)
    output wire           lk2_complete,  // 1 = that entry already has its result
    output wire [DW-1:0]  lk2_data,     // Result data for lk2_tag

    output wire           rob_empty     // ROB has no allocated entries
);

// ---- Per-entry storage arrays --------------------------------------
reg          rob_entry_valid    [0:DEPTH-1]; // Slot is allocated
reg          rob_entry_complete [0:DEPTH-1]; // Result has been written
reg          rob_entry_fault    [0:DEPTH-1]; // [Fix #3] Instruction faulted
reg [AW-1:0] rob_entry_rd      [0:DEPTH-1]; // Destination arch register
reg [OW-1:0] rob_entry_opcode  [0:DEPTH-1]; // Opcode
reg [DW-1:0] rob_entry_data    [0:DEPTH-1]; // Result data

// ---- Head and tail pointers ----------------------------------------
reg [TW-1:0] rob_head; // Index of oldest allocated (not yet committed) entry
reg [TW-1:0] rob_tail; // Index of next slot to allocate

// ---- Combinational status signals ----------------------------------
// Full: allocating would make tail reach head (one slot wasted, standard)
assign rob_full  = rob_entry_valid[rob_tail];
assign rob_empty = !rob_entry_valid[rob_head];

// Allocation tag is the CURRENT tail (before incrementing)
assign alloc_tag = rob_tail;

// ---- Commit: head entry is valid AND complete ----------------------
assign commit_valid = rob_entry_valid[rob_head] && rob_entry_complete[rob_head];
assign commit_rd    = rob_entry_rd   [rob_head];
assign commit_data  = rob_entry_data [rob_head];
assign commit_tag   = rob_head;
assign commit_fault = rob_entry_fault[rob_head]; // [Fix #3]

// ---- [Fix #1] Combinational tag lookups ----------------------------
// Purely combinational: index the complete/data arrays by the incoming tag.
// Safe to read at any time; garbage when the slot is unallocated but callers
// gate on reg_in_flight so they never act on a stale lookup.
assign lk1_complete = rob_entry_complete[lk1_tag];
assign lk1_data     = rob_entry_data    [lk1_tag];
assign lk2_complete = rob_entry_complete[lk2_tag];
assign lk2_data     = rob_entry_data    [lk2_tag];

// ---- Sequential: allocate, complete, commit, flush -----------------
integer r;
always @(posedge clk) begin
    if (!rst_n) begin
        rob_head <= {TW{1'b0}};
        rob_tail <= {TW{1'b0}};
        for (r = 0; r < DEPTH; r = r + 1) begin
            rob_entry_valid[r]    <= 1'b0;
            rob_entry_complete[r] <= 1'b0;
            rob_entry_fault[r]    <= 1'b0;
            rob_entry_rd[r]       <= {AW{1'b0}};
            rob_entry_opcode[r]   <= {OW{1'b0}};
            rob_entry_data[r]     <= {DW{1'b0}};
        end
    end else if (flush) begin
        rob_head <= {TW{1'b0}};
        rob_tail <= {TW{1'b0}};
        for (r = 0; r < DEPTH; r = r + 1) begin
            rob_entry_valid[r]    <= 1'b0;
            rob_entry_complete[r] <= 1'b0;
            rob_entry_fault[r]    <= 1'b0;
        end
    end else begin
        // CDB completion: write result into the target ROB slot
        if (complete_valid) begin
            rob_entry_data[complete_tag]     <= complete_data;
            rob_entry_complete[complete_tag] <= 1'b1;
        end

        // Allocation: reserve tail slot for a new instruction
        if (alloc_valid && !rob_full) begin
            rob_entry_valid[rob_tail]    <= 1'b1;
            rob_entry_complete[rob_tail] <= 1'b0;
            rob_entry_fault[rob_tail]    <= alloc_fault; // [Fix #3]
            rob_entry_rd[rob_tail]       <= alloc_rd;
            rob_entry_opcode[rob_tail]   <= alloc_opcode;
            rob_entry_data[rob_tail]     <= {DW{1'b0}};
            rob_tail                     <= rob_tail + 1'b1; // wraps at DEPTH (power-of-2)
        end

        // Commit: retire head entry when acknowledged
        // [Fix #3] commit_ack is now gated externally: commit_valid && !commit_fault
        // so a faulting head stalls here until flush resolves it.
        if (commit_ack && commit_valid) begin
            rob_entry_valid[rob_head]    <= 1'b0;
            rob_entry_complete[rob_head] <= 1'b0;
            rob_entry_fault[rob_head]    <= 1'b0;
            rob_head                     <= rob_head + 1'b1;
        end
    end
end

endmodule

// ============================================================
// Module      : register_alias_table
// Project     : Out-of-Order Issue Queue - Tesla AI Hardware Portfolio
// Author      : Mayank
// Description : Register Alias Table (RAT) / physical register file.
//               Tracks which architectural registers are in-flight and their
//               ROB tags.  Read ports include same-cycle CDB forwarding so
//               dispatch can see a result that broadcasts on the same clock.
//
// Fix #1 — RAT deadlock (post-CDB, pre-commit readiness):
//   The original design declared a source operand NOT ready when its tag was
//   in-flight but the CDB broadcast had already happened in a prior cycle.
//   A consumer dispatching in that window would enter the RS with s_rdy=0 and
//   a tag that will never broadcast again, causing a permanent RS stall.
//
//   Fix (Option 2 — ROB-based lookup, recommended because the second FU /
//   LSU integration will need it anyway):
//     Two new input ports, rob_rs1_complete/data and rob_rs2_complete/data,
//     carry the ROB's own per-entry complete flag for each source's rename tag.
//     The readiness combinational logic adds a third arm:
//       rs1_ready = !in_flight || cdb_hit_this_cycle || rob_already_complete
//     Data is taken from the ROB's stored result when the ROB-hit path fires.
//     Clearing: reg_in_flight is still cleared at commit; no extra state needed
//     because the ROB tag can never be stale (only freed at commit, which also
//     clears in_flight, making rob_hit false thereafter).
//
// Parameters  : DW=32, TW=4, AW=5, NR=32
// ============================================================

`include "defines.vh"

module register_alias_table #(
    parameter DW = `DATA_WIDTH,   // Data width per register
    parameter TW = `TAG_WIDTH,    // ROB tag width
    parameter AW = `REG_ADDR_W,   // Architectural register address width
    parameter NR = `NUM_REGS      // Number of architectural registers
)(
    input  wire           clk,          // System clock
    input  wire           rst_n,        // Active-low synchronous reset
    input  wire           flush,        // Flush: clear all in-flight bits

    // Dispatch read ports (combinational, used by ooo_top at dispatch time)
    input  wire [AW-1:0]  rs1_addr,     // Source-1 architectural register index
    input  wire [AW-1:0]  rs2_addr,     // Source-2 architectural register index
    output wire           rs1_ready,    // Source-1 is ready (data valid now)
    output wire [DW-1:0]  rs1_data,     // Source-1 value (if rs1_ready)
    output wire [TW-1:0]  rs1_tag,      // Source-1 in-flight ROB tag (if !rs1_ready)
    output wire           rs2_ready,    // Source-2 is ready (data valid now)
    output wire [DW-1:0]  rs2_data,     // Source-2 value (if rs2_ready)
    output wire [TW-1:0]  rs2_tag,      // Source-2 in-flight ROB tag (if !rs2_ready)

    // Rename write port (from ooo_top at dispatch, clocked)
    input  wire           rename_valid, // Dispatch is occurring this cycle
    input  wire [AW-1:0]  rename_rd,    // Architectural destination register
    input  wire [TW-1:0]  rename_tag,   // ROB slot assigned to this instruction

    // Commit port (from ROB head, clocked)
    input  wire           commit_valid, // ROB is committing head entry this cycle
    input  wire [AW-1:0]  commit_rd,    // Architectural register being written
    input  wire [DW-1:0]  commit_data,  // Committed value
    input  wire [TW-1:0]  commit_tag,   // ROB tag of committing instruction (WAW guard)

    // CDB snoop (combinational, used for read-port forwarding AND clocked capture)
    input  wire           cdb_valid,    // Result available on CDB this cycle
    input  wire [TW-1:0]  cdb_tag,      // Tag of result on CDB
    input  wire [DW-1:0]  cdb_data,     // Data of result on CDB

    // [Fix #1] ROB tag-lookup results (combinational, driven by reorder_buffer lk* ports)
    // rob_rsN_complete: the ROB entry for rsN's current rename tag is already done.
    // rob_rsN_data   : the result stored in that ROB entry.
    // These let the RAT mark a source ready even when the CDB has already broadcast
    // but commit has not yet fired — closing the post-CDB deadlock window.
    input  wire           rob_rs1_complete,  // ROB says rs1's tag entry is complete
    input  wire [DW-1:0]  rob_rs1_data,     // ROB's stored result for rs1's tag
    input  wire           rob_rs2_complete,  // ROB says rs2's tag entry is complete
    input  wire [DW-1:0]  rob_rs2_data      // ROB's stored result for rs2's tag
);

// ---- Register state arrays -----------------------------------------
reg [DW-1:0] reg_data      [0:NR-1];   // Committed / forwarded register values
reg [TW-1:0] reg_tag       [0:NR-1];   // Current rename tag (valid when in_flight)
reg          reg_in_flight [0:NR-1];   // 1 = a pending instruction will write this reg

// ---- Combinational read with same-cycle CDB forwarding -------------
// Priority for rs1_ready (three independent arms, highest-priority listed first):
//   1. Not in-flight at all              → ready, use reg_data
//   2. In-flight AND CDB hits this cycle → ready, use cdb_data (same-cycle fwd)
//   3. In-flight AND ROB entry complete  → ready, use rob_rs1_data  [Fix #1]
//   4. In-flight, no result yet          → not ready, consumer waits in RS
wire rs1_cdb_hit = reg_in_flight[rs1_addr] && cdb_valid && (reg_tag[rs1_addr] == cdb_tag);
wire rs2_cdb_hit = reg_in_flight[rs2_addr] && cdb_valid && (reg_tag[rs2_addr] == cdb_tag);

// [Fix #1] ROB-hit: the tag was already broadcast in a previous cycle;
// the ROB has the result but commit has not fired yet.
wire rs1_rob_hit = reg_in_flight[rs1_addr] && rob_rs1_complete;
wire rs2_rob_hit = reg_in_flight[rs2_addr] && rob_rs2_complete;

assign rs1_ready = !reg_in_flight[rs1_addr] || rs1_cdb_hit || rs1_rob_hit;
assign rs1_data  = rs1_cdb_hit ? cdb_data  :
                   rs1_rob_hit ? rob_rs1_data :
                                 reg_data[rs1_addr];
assign rs1_tag   = reg_tag[rs1_addr];

assign rs2_ready = !reg_in_flight[rs2_addr] || rs2_cdb_hit || rs2_rob_hit;
assign rs2_data  = rs2_cdb_hit ? cdb_data  :
                   rs2_rob_hit ? rob_rs2_data :
                                 reg_data[rs2_addr];
assign rs2_tag   = reg_tag[rs2_addr];

// ---- Sequential: rename, commit, CDB data capture -----------------
integer r;
always @(posedge clk) begin
    if (!rst_n) begin
        for (r = 0; r < NR; r = r + 1) begin
            reg_data[r]      <= {DW{1'b0}};
            reg_tag[r]       <= {TW{1'b0}};
            reg_in_flight[r] <= 1'b0;
        end
    end else if (flush) begin
        // Flush: retire all in-flight state; data already written by CDB is kept
        for (r = 0; r < NR; r = r + 1)
            reg_in_flight[r] <= 1'b0;
    end else begin
        // CDB capture: forward result into architectural register data array.
        // reg_in_flight is NOT cleared here — only commit clears it.
        // The ROB-lookup path (Fix #1) bridges the gap between this capture
        // and the eventual commit.
        if (cdb_valid) begin
            for (r = 0; r < NR; r = r + 1) begin
                if (reg_in_flight[r] && (reg_tag[r] == cdb_tag))
                    reg_data[r] <= cdb_data;
            end
        end

        // Commit: ROB head retires - clear in_flight only if tags still match
        // (WAW guard: a newer rename may have already overwritten reg_tag)
        if (commit_valid && (commit_rd != {AW{1'b0}})) begin
            reg_data[commit_rd] <= commit_data;
            if (reg_tag[commit_rd] == commit_tag)
                reg_in_flight[commit_rd] <= 1'b0;
        end

        // Rename: new dispatch creates a new mapping for rd.
        // Processed AFTER commit so a commit+rename of the same rd in the
        // same cycle leaves the register marked in-flight (the rename wins).
        if (rename_valid && (rename_rd != {AW{1'b0}})) begin
            reg_tag[rename_rd]       <= rename_tag;
            reg_in_flight[rename_rd] <= 1'b1;
        end
    end
end

endmodule

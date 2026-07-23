// ============================================================
// Module      : common_data_bus
// Project     : Out-of-Order Issue Queue - Tesla AI Hardware Portfolio
// Author      : Mayank
// Description : CDB broadcast with round-robin arbitration for up to 2 FUs.
//               With NUM_FU=1 (current default), fu1 is tied to 0 externally
//               and fu0_grant = fu0_valid every cycle — zero overhead.
//               When NUM_FU=2, both FUs compete for the bus; a registered
//               round-robin priority bit ensures fairness under contention.
//               fu_grant outputs feed back into the FUs so a denied FU can
//               hold its result and retry the next cycle.
//
// Fix #2 — multi-FU interface prep:
//   Added clk/rst_n, fu0_grant, fu1_valid/fu1_tag/fu1_data, fu1_grant,
//   and the round-robin priority register.  The existing single-FU pass-
//   through behaviour is fully preserved when fu1_valid=0.
//
// Parameters  : DW=32 (data width), TW=4 (tag width), NUM_FU=1
// ============================================================

`include "defines.vh"

module common_data_bus #(
    parameter DW     = `DATA_WIDTH,  // Data path width
    parameter TW     = `TAG_WIDTH,   // ROB tag width
    parameter NUM_FU = `NUM_FU       // Functional unit count
)(
    input  wire           clk,        // System clock (needed for RR state)
    input  wire           rst_n,      // Active-low synchronous reset

    // FU-0 result request
    input  wire           fu0_valid,  // FU-0 has a result ready
    input  wire [TW-1:0]  fu0_tag,    // FU-0 destination ROB tag
    input  wire [DW-1:0]  fu0_data,   // FU-0 result data
    output wire           fu0_grant,  // CDB granted to FU-0 this cycle

    // FU-1 result request (tie fu1_valid=0 while NUM_FU=1; LSU hooks in here)
    input  wire           fu1_valid,  // FU-1 has a result ready
    input  wire [TW-1:0]  fu1_tag,    // FU-1 destination ROB tag
    input  wire [DW-1:0]  fu1_data,   // FU-1 result data
    output wire           fu1_grant,  // CDB granted to FU-1 this cycle

    // Broadcast outputs (connected to RS, RAT, ROB simultaneously)
    output wire           cdb_valid,  // Broadcast valid this cycle
    output wire [TW-1:0]  cdb_tag,    // Winning tag on the bus
    output wire [DW-1:0]  cdb_data    // Winning data on the bus
);

// ---- Round-robin priority register ---------------------------------
// 0: prefer fu0 when both valid, 1: prefer fu1 when both valid.
// Alternates after each cycle where BOTH FUs had results and one was denied.
reg rr_priority;

// ---- Arbitration ---------------------------------------------------
// Grant: give the bus to the requester that wins arbitration this cycle.
// If only one FU is valid, it always wins.  If both valid, rr_priority decides.
assign fu0_grant = fu0_valid && (!fu1_valid || !rr_priority);
assign fu1_grant = fu1_valid && (!fu0_valid ||  rr_priority);

// ---- Broadcast mux -------------------------------------------------
assign cdb_valid = fu0_grant || fu1_grant;
assign cdb_tag   = fu0_grant ? fu0_tag  : fu1_tag;
assign cdb_data  = fu0_grant ? fu0_data : fu1_data;

// ---- Priority update -----------------------------------------------
// Only flip when BOTH FUs are active so single-FU operation never touches rr_priority.
always @(posedge clk) begin
    if (!rst_n)
        rr_priority <= 1'b0;
    else if (fu0_grant && fu1_valid)
        // fu0 won this cycle but fu1 was waiting — prefer fu1 next
        rr_priority <= 1'b1;
    else if (fu1_grant && fu0_valid)
        // fu1 won this cycle but fu0 was waiting — prefer fu0 next
        rr_priority <= 1'b0;
    // else: only one FU active, priority unchanged
end

endmodule

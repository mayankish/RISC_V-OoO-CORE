// ============================================================
// Module      : branch_unit
// Project     : riscv_ooo_core - Tesla AI Hardware Portfolio
// Author      : Mayank
// Description : Frontend-resolved conditional branch handling (task #6).
//
//               Design choice: branches never enter the OoO backend at all
//               (they don't write a register - rd is repurposed as the
//               condition code). Fetch simply STALLS at a branch until its
//               two operands are ready (using ooo_top's src1/src2 tap, which
//               already carries the Fix #1 post-CDB/pre-commit correctness
//               for free), then this module evaluates the condition
//               directly from the actual operand values and redirects the
//               PC if taken.
//
//               This is deliberately NOT speculative/predicted execution:
//               fetch never advances past an unresolved branch, so there is
//               nothing to flush on a "misprediction" (there is no
//               prediction to get wrong) - ooo_top's `flush` port is
//               therefore untouched by this design. The cost is a stall on
//               every branch until its operands are ready; the benefit is
//               zero new hazard logic inside the already-verified RAT/RS/
//               ROB, and a correctness argument that doesn't depend on any
//               new timing reasoning. Branch prediction / speculative
//               fetch-past-branch is a natural future upgrade, explicitly
//               out of scope here (see README.md).
//
//               Signed vs. unsigned comparisons are computed directly from
//               the two operand values (not derived from a subtraction),
//               so there is no overflow pitfall of the classic
//               "check the sign of rs1-rs2" trick.
// ============================================================

`include "defines.vh"

module branch_unit #(
    parameter DW = `DATA_WIDTH
)(
    input  wire            is_branch,   // this cycle's fetched instr is EXTC_BRANCH
    input  wire [2:0]      cond,        // condition code (instr's rd[2:0] field)
    input  wire [DW-1:0]   imm,         // sign-extended signed word offset
    input  wire [DW-1:0]   pc,          // this branch's own fetch PC (word address)

    input  wire            rs1_ready,
    input  wire [DW-1:0]   rs1_data,
    input  wire            rs2_ready,
    input  wire [DW-1:0]   rs2_data,

    output wire             stall,           // hold fetch: operands not ready yet
    output wire             redirect_valid,  // resolved this cycle AND taken
    output wire [DW-1:0]    redirect_target
);

wire operands_ready = rs1_ready && rs2_ready;
assign stall = is_branch && !operands_ready;

wire eq   = (rs1_data == rs2_data);
wire lt_s = ($signed(rs1_data) < $signed(rs2_data));   // true signed compare - no
                                                        // subtraction-overflow pitfall
wire lt_u = (rs1_data < rs2_data);                     // unsigned compare

reg taken;
always @(*) begin
    case (cond)
        `BR_BEQ : taken = eq;
        `BR_BNE : taken = !eq;
        `BR_BLT : taken = lt_s;
        `BR_BGE : taken = !lt_s;
        `BR_BLTU: taken = lt_u;
        `BR_BGEU: taken = !lt_u;
        default : taken = 1'b0;   // undefined condition code: never taken (safe default)
    endcase
end

assign redirect_valid  = is_branch && operands_ready && taken;
assign redirect_target = pc + 1'b1 + imm;  // pc + 1 + signed offset (widened to DW by context)

endmodule

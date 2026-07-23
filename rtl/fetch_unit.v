// ============================================================
// Module      : fetch_unit
// Project     : riscv_ooo_core - Tesla AI Hardware Portfolio
// Author      : Mayank
// Description : Minimal in-order fetch stage: a free-running, word-addressed
//               program counter with two override inputs (redirect, stall).
//               Instruction memory itself lives outside this module (a real
//               boot ROM/SRAM once integrated into the SoC - see README);
//               this module only produces the fetch address and validity.
//
//               instr_valid is purely combinational (= rst_n), not a
//               registered copy, specifically so pc's reset value (word 0)
//               is presented as valid on the very first cycle out of reset
//               rather than being skipped by an off-by-one delay.
//
//               redirect_valid always wins over stall (a resolved branch/
//               jump/trap redirect must be able to override an in-flight
//               stall on the instruction that caused it).
// ============================================================

module fetch_unit #(
    parameter PC_W = 32
)(
    input  wire            clk,
    input  wire            rst_n,
    input  wire            stall,           // hold PC (backend not ready, or
                                             // this cycle's instruction isn't
                                             // supported yet - see decoder.v)
    input  wire            redirect_valid,  // resolved branch/jump/trap (task #6/#8)
    input  wire [PC_W-1:0] redirect_target, // word address to redirect to

    output wire [PC_W-1:0] pc,              // this cycle's fetch address
    output wire            instr_valid      // this cycle's fetch is valid
);

reg [PC_W-1:0] pc_r;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        pc_r <= {PC_W{1'b0}};
    end else if (redirect_valid) begin
        pc_r <= redirect_target;
    end else if (!stall) begin
        pc_r <= pc_r + 1'b1;   // word-addressed: next sequential instruction
    end
    // else: stall && !redirect_valid -> hold pc_r (retry the same fetch)
end

assign pc          = pc_r;
assign instr_valid = rst_n;

endmodule

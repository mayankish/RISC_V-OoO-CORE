// ============================================================
// Module      : reservation_station
// Project     : Out-of-Order Issue Queue - Tesla AI Hardware Portfolio
// Author      : Mayank
// Description : Parameterized reservation station with age-based (oldest-first)
//               issue priority and same-cycle CDB-to-issue operand forwarding.
//               CAM tag comparison is the design-critical path at 500 MHz.
// Parameters  : RS_DEPTH=8, DW=32, TW=4, AW=5, OW=3, SEQW=8
// ============================================================

`include "defines.vh"

module reservation_station #(
    parameter RS_DEPTH = `RS_DEPTH,   // Number of RS entries
    parameter DW       = `DATA_WIDTH, // Data width
    parameter TW       = `TAG_WIDTH,  // ROB tag width
    parameter AW       = `REG_ADDR_W, // Arch register address width
    parameter OW       = 3,           // Opcode width
    parameter SEQW     = `SEQW        // Dispatch sequence counter width
)(
    input  wire           clk,              // System clock
    input  wire           rst_n,            // Active-low synchronous reset
    input  wire           flush,            // Flush all entries immediately

    // Dispatch port (from ooo_top)
    input  wire           dispatch_valid,   // New instruction being dispatched
    input  wire [OW-1:0]  dispatch_opcode,  // ALU operation
    input  wire [AW-1:0]  dispatch_rs1,     // Arch source-1 register
    input  wire [AW-1:0]  dispatch_rs2,     // Arch source-2 register
    input  wire [AW-1:0]  dispatch_rd,      // Arch destination register
    input  wire           dispatch_s1_rdy,  // Source-1 data available now
    input  wire [DW-1:0]  dispatch_s1_data, // Source-1 data (valid when s1_rdy)
    input  wire [TW-1:0]  dispatch_s1_tag,  // Source-1 pending ROB tag (!s1_rdy)
    input  wire           dispatch_s2_rdy,  // Source-2 data available now
    input  wire [DW-1:0]  dispatch_s2_data, // Source-2 data (valid when s2_rdy)
    input  wire [TW-1:0]  dispatch_s2_tag,  // Source-2 pending ROB tag (!s2_rdy)
    input  wire [TW-1:0]  dispatch_dest_tag,// ROB slot assigned to this instruction

    // Issue port (to integer_alu)
    input  wire           fu_ready,         // Functional unit is free to accept work
    output wire           issue_valid,      // An instruction is ready to issue
    output wire [OW-1:0]  issue_opcode,     // Opcode of instruction being issued
    output wire [DW-1:0]  issue_src1,       // Resolved source-1 data
    output wire [DW-1:0]  issue_src2,       // Resolved source-2 data
    output wire [TW-1:0]  issue_tag,        // Destination ROB tag being issued

    // CDB snoop (broadcast from common_data_bus)
    input  wire           cdb_valid,        // CDB has a valid result this cycle
    input  wire [TW-1:0]  cdb_tag,          // Tag of result on CDB
    input  wire [DW-1:0]  cdb_data,         // Data of result on CDB

    // Status
    output wire           rs_full,          // RS has no free entries
    output wire           rs_empty          // RS has no valid entries
);

// ---- Derived local parameters ---------------------------------------
localparam IDX_W = $clog2(RS_DEPTH); // Width of an RS entry index

// ---- Per-entry storage arrays --------------------------------------
reg             rs_valid    [0:RS_DEPTH-1]; // Entry occupied
reg [OW-1:0]    rs_opcode   [0:RS_DEPTH-1];
reg [AW-1:0]    rs_rd       [0:RS_DEPTH-1]; // Arch dest (debug/trace only)
reg [DW-1:0]    rs_src1     [0:RS_DEPTH-1]; // Source-1 data
reg [DW-1:0]    rs_src2     [0:RS_DEPTH-1]; // Source-2 data
reg [TW-1:0]    rs_src1_tag [0:RS_DEPTH-1]; // Source-1 pending tag
reg [TW-1:0]    rs_src2_tag [0:RS_DEPTH-1]; // Source-2 pending tag
reg             rs_s1_rdy   [0:RS_DEPTH-1]; // Source-1 operand ready
reg             rs_s2_rdy   [0:RS_DEPTH-1]; // Source-2 operand ready
reg [TW-1:0]    rs_dest_tag [0:RS_DEPTH-1]; // Destination ROB tag
reg [SEQW-1:0]  rs_seq      [0:RS_DEPTH-1]; // Dispatch sequence (age tracker)

// ---- Global dispatch sequence counter (oldest = lowest seq value) --
reg [SEQW-1:0] dispatch_seq;

// ---- Free-slot combinational priority encoder ----------------------
// Lowest-index free slot is selected for dispatch allocation.
reg [IDX_W-1:0] free_idx;
reg             free_found;
integer m;
// --- Combinational: find first free slot ---
always @(*) begin
    free_idx   = {IDX_W{1'b1}};
    free_found = 1'b0;
    for (m = RS_DEPTH-1; m >= 0; m = m - 1) begin
        if (!rs_valid[m]) begin
            free_idx   = m[IDX_W-1:0];
            free_found = 1'b1;
        end
    end
end

assign rs_full = !free_found;

// OR-reduce rs_valid array via combinational loop (array range-index not allowed)
reg rs_empty_c;
integer n;
always @(*) begin
    rs_empty_c = 1'b1;
    for (n = 0; n < RS_DEPTH; n = n + 1)
        if (rs_valid[n]) rs_empty_c = 1'b0;
end
assign rs_empty = rs_empty_c;

// ---- Per-entry: same-cycle CDB forwarding for issue readiness ------
// CRITICAL PATH: CAM lookup - constrained to 500 MHz, see constraints.sdc
wire e_s1_rdy_now [0:RS_DEPTH-1]; // src1 ready including current-cycle CDB
wire e_s2_rdy_now [0:RS_DEPTH-1]; // src2 ready including current-cycle CDB
wire e_ready_now  [0:RS_DEPTH-1]; // both sources ready this cycle

genvar gi;
generate
    for (gi = 0; gi < RS_DEPTH; gi = gi + 1) begin : g_rdy
        assign e_s1_rdy_now[gi] = rs_s1_rdy[gi] ||
               (cdb_valid && rs_valid[gi] && !rs_s1_rdy[gi] &&
                (rs_src1_tag[gi] == cdb_tag));
        assign e_s2_rdy_now[gi] = rs_s2_rdy[gi] ||
               (cdb_valid && rs_valid[gi] && !rs_s2_rdy[gi] &&
                (rs_src2_tag[gi] == cdb_tag));
        assign e_ready_now[gi]  = rs_valid[gi] &&
                                  e_s1_rdy_now[gi] && e_s2_rdy_now[gi];
    end
endgenerate

// ---- Issue selection: age-based priority (lowest seq = oldest) -----
reg [IDX_W-1:0]  issue_idx;
reg              issue_valid_c;
reg [SEQW-1:0]   min_seq;
integer k;
// --- Combinational: pick oldest READY entry ---
always @(*) begin
    issue_idx     = {IDX_W{1'b0}};
    issue_valid_c = 1'b0;
    min_seq       = {SEQW{1'b1}};
    for (k = 0; k < RS_DEPTH; k = k + 1) begin
        if (e_ready_now[k]) begin
            if (!issue_valid_c || rs_seq[k] < min_seq) begin
                min_seq       = rs_seq[k];
                issue_idx     = k[IDX_W-1:0];
                issue_valid_c = 1'b1;
            end
        end
    end
end

// Issue fires only when FU is available
assign issue_valid = issue_valid_c && fu_ready;

// ---- Issue output mux: resolve operand data via CDB if needed ------
// If an operand became ready only due to current-cycle CDB, use cdb_data
wire s1_via_cdb = !rs_s1_rdy[issue_idx] && cdb_valid &&
                  (rs_src1_tag[issue_idx] == cdb_tag);
wire s2_via_cdb = !rs_s2_rdy[issue_idx] && cdb_valid &&
                  (rs_src2_tag[issue_idx] == cdb_tag);

assign issue_opcode = rs_opcode  [issue_idx];
assign issue_src1   = s1_via_cdb ? cdb_data : rs_src1[issue_idx];
assign issue_src2   = s2_via_cdb ? cdb_data : rs_src2[issue_idx];
assign issue_tag    = rs_dest_tag[issue_idx];

// ---- Sequential: dispatch, CDB capture, issue clear, flush ---------
integer i;
always @(posedge clk) begin
    if (!rst_n) begin
        dispatch_seq <= {SEQW{1'b0}};
        for (i = 0; i < RS_DEPTH; i = i + 1) begin
            rs_valid[i]    <= 1'b0;
            rs_opcode[i]   <= {OW{1'b0}};
            rs_rd[i]       <= {AW{1'b0}};
            rs_src1[i]     <= {DW{1'b0}};
            rs_src2[i]     <= {DW{1'b0}};
            rs_src1_tag[i] <= {TW{1'b0}};
            rs_src2_tag[i] <= {TW{1'b0}};
            rs_s1_rdy[i]   <= 1'b0;
            rs_s2_rdy[i]   <= 1'b0;
            rs_dest_tag[i] <= {TW{1'b0}};
            rs_seq[i]      <= {SEQW{1'b0}};
        end
    end else if (flush) begin
        dispatch_seq <= {SEQW{1'b0}};
        for (i = 0; i < RS_DEPTH; i = i + 1)
            rs_valid[i] <= 1'b0;
    end else begin
        // CDB operand capture for WAITING entries
        if (cdb_valid) begin
            for (i = 0; i < RS_DEPTH; i = i + 1) begin
                if (rs_valid[i] && !rs_s1_rdy[i] && (rs_src1_tag[i] == cdb_tag)) begin
                    rs_src1[i]   <= cdb_data;
                    rs_s1_rdy[i] <= 1'b1;
                end
                if (rs_valid[i] && !rs_s2_rdy[i] && (rs_src2_tag[i] == cdb_tag)) begin
                    rs_src2[i]   <= cdb_data;
                    rs_s2_rdy[i] <= 1'b1;
                end
            end
        end

        // Dispatch: allocate free slot
        if (dispatch_valid && !rs_full) begin
            rs_valid[free_idx]    <= 1'b1;
            rs_opcode[free_idx]   <= dispatch_opcode;
            rs_rd[free_idx]       <= dispatch_rd;
            rs_src1[free_idx]     <= dispatch_s1_data;
            rs_src2[free_idx]     <= dispatch_s2_data;
            rs_src1_tag[free_idx] <= dispatch_s1_tag;
            rs_src2_tag[free_idx] <= dispatch_s2_tag;
            rs_s1_rdy[free_idx]   <= dispatch_s1_rdy;
            rs_s2_rdy[free_idx]   <= dispatch_s2_rdy;
            rs_dest_tag[free_idx] <= dispatch_dest_tag;
            rs_seq[free_idx]      <= dispatch_seq;
            dispatch_seq          <= dispatch_seq + 1'b1;
        end

        // Issue: clear the issued entry
        if (issue_valid)
            rs_valid[issue_idx] <= 1'b0;
    end
end

endmodule

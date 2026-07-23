# riscv_ooo_core — Custom ISA Specification

**Status: all 6 instruction classes are wired, dispatched/resolved, and
verified in simulation** (`tb/tb_riscv_ooo_top.v`: 18/18 commits correct
across ALU, ADDI, BEQ/BNE/BLT, STORE+LOAD, JAL, WFI+wake, HALT;
`tb/tb_riscv_ooo_top_fault.v`: both reserved-encoding fault paths correctly
latch `core_faulted`). See README.md for the full execution-model writeup
(what dispatches into the backend vs. what resolves entirely in the fetch
stage) and the honest list of what this does *not* cover yet (backward
branches/loops specifically weren't exercised; a true OoO load/store unit
and branch prediction are explicitly deferred, not attempted).

Despite the name "RISC-V", this is **not** RV32I — the base project
(`ooo_issue_queue`) already committed to a custom 32-bit encoding before this
extension existed, and the instruction was to extend it with a custom
encoding rather than switch families. "RISC-V-based" here means: a RISC
load/store register machine with the same *philosophy* as RV32I (fixed
32-bit words, 32 general registers, x0 hardwired zero, sign-extended
immediates, PC-relative control flow) but a bespoke field layout chosen to
slot cleanly on top of the existing, already-verified OoO backend.

## 1. Base ALU-class encoding (unchanged, from ooo_issue_queue)

```
 31        27 26        22 21        17 16    14 13                    0
+------------+------------+------------+--------+----------------------+
|     rd     |    rs1     |    rs2     |   op   |       unused         |
+------------+------------+------------+--------+----------------------+
```

| Field | Bits    | Meaning                     |
|-------|---------|-----------------------------|
| rd    | [31:27] | Destination register (x0-x31) |
| rs1   | [26:22] | Source register 1           |
| rs2   | [21:17] | Source register 2           |
| op    | [16:14] | ALU opcode (see below)      |

| op (3'b) | Mnemonic | Operation      |
|----------|----------|----------------|
| 000      | ADD      | rd = rs1 + rs2 |
| 001      | SUB      | rd = rs1 - rs2 |
| 010      | MUL      | rd = rs1 * rs2 |
| 011      | SHL      | rd = rs1 << rs2 |
| 100      | SHR      | rd = rs1 >> rs2 |
| 101      | **EXT**  | escape — see §2 (was reserved/unused in the base project) |
| 110      | —        | **reserved — illegal; latches `core_faulted` (verified)** |
| 111      | NOP      | rd forced to x0 (discarded); architectural no-op |

x0 is hardwired zero: any commit with `rd == 0` is discarded by the RAT
(`register_alias_table.v`), matching RV32I convention.

## 2. Extended-class encoding (op = 3'b101, EXT)

The base project never used op=101, so it is repurposed as a 6-way escape
into everything RV32I would normally cover with separate major opcodes
(branches, loads, stores, immediates, jumps, system calls):

```
 31        27 26        22 21        17 16    14 13    11 10          0
+------------+------------+------------+--------+--------+------------+
|     rd     |    rs1     |    rs2     | 1 0 1  |ext_class|  payload   |
+------------+------------+------------+--------+--------+------------+
```

`payload` is 11 bits, sign-extended to 32 for anything used as an
immediate/offset (range: -1024..+1023, in words for branch/load/store/jal
offsets since this machine has no separate byte addressing).

| ext_class (3'b) | Name        | rd            | rs1     | rs2      | payload            | Status |
|------------------|-------------|---------------|---------|----------|--------------------|--------|
| 000              | BRANCH      | cond[2:0] (§2.1) | compare A | compare B | signed word offset | **wired** — resolved in fetch, never dispatches (§3) |
| 001              | LOAD        | dest          | base    | unused   | signed word offset | **wired** — resolves in fetch, dispatches a write-back (§3) |
| 010              | STORE       | unused        | base    | data     | signed word offset | **wired** — resolved in fetch, never dispatches (§3) |
| 011              | ADDI        | dest          | base    | unused   | signed immediate   | **wired** — dispatches via the immediate mux |
| 100              | JAL         | link (gets PC+1) | unused | unused | signed word offset | **wired** — redirects in fetch, dispatches the link write (§3) |
| 101              | SYSTEM      | unused        | unused  | unused   | cause code (§2.2)  | **wired** — HALT/WFI handled, undefined payload faults |
| 110, 111         | reserved    | —             | —       | —        | reserved for CSR / trap-return (future) | **illegal — latches `core_faulted`** (verified) |

### 2.1 Branch condition codes (BRANCH's `rd[2:0]` field)

| Code (3'b) | Mnemonic | Taken if           |
|------------|----------|--------------------|
| 000        | BEQ      | rs1 == rs2         |
| 001        | BNE      | rs1 != rs2         |
| 010        | BLT      | rs1 < rs2 (signed) |
| 011        | BGE      | rs1 >= rs2 (signed)|
| 100        | BLTU     | rs1 < rs2 (unsigned)|
| 101        | BGEU     | rs1 >= rs2 (unsigned)|

`branch_unit.v` computes these directly from the two operand values
(`$signed(rs1) < $signed(rs2)` etc.), not by inspecting the sign of a
subtraction — the classic "check if rs1-rs2 is negative" shortcut is wrong
in the presence of signed overflow; a true comparator has no such pitfall.
BEQ and BLT are both verified in simulation (BLT specifically with a
negative operand, `-88 < 100`).

### 2.2 System cause codes (SYSTEM's `payload` field)

| Code       | Mnemonic | Meaning                          | Status |
|------------|----------|-----------------------------------|--------|
| 11'd1      | SYS_HALT | Stop fetch permanently             | verified |
| 11'd2      | SYS_WFI  | Sleep until `wake_i` pulses, then resume | verified (sleep + wake + resume) |
| any other  | —        | undefined — latches `core_faulted` | verified |

## 3. Execution model per class (see README.md for full rationale)

- **ALU, ADDI, JAL, LOAD** dispatch into the OoO backend (rename -> RS ->
  ALU -> CDB -> ROB -> commit), inheriting the RAT-deadlock, multi-FU-CDB,
  and fault-bit fixes unchanged. JAL and LOAD reuse the same
  `instr_use_imm`/`instr_imm` mux ADDI uses — JAL supplies `pc+1` (the link
  value) as the "immediate," LOAD supplies the memory read result once
  resolved. A load takes one extra cycle versus ADDI/JAL specifically
  because its base-register read and its write-back dispatch can't share
  the same `ooo_top` instruction-word cycle (see `riscv_ooo_top.v`'s header
  comment for the exact conflict and why it's split into two phases).
- **BRANCH and STORE never dispatch into the backend at all.** They resolve
  entirely in the fetch stage: stall until their operands are ready (via a
  read-only tap into the RAT's existing Fix #1 readiness logic), then act
  directly — redirect the PC (branch) or drive the memory write (store).
  Because fetch never advances past an unresolved branch/store, there is
  nothing speculative to unwind, so `flush` is never asserted by this
  design.
- **SYSTEM/HALT and SYSTEM/WFI** are handled by dedicated sticky state in
  `riscv_ooo_top.v` (`core_halted_r`, `sleeping_r`), not by dispatch.
- **Reserved encodings** (ext_class 110/111, or ALU op 110) latch
  `core_faulted` and stop fetch permanently, distinctly flagged from a
  clean halt. Finding and fixing the bug that made this necessary is worth
  noting: the reserved ALU opcode was initially still being accepted by
  `decoder.v`'s `supported` signal (which only checked "is this EXT-format",
  not "is this specific op value defined"), so it dispatched into the
  backend every cycle fetch was stalled on it — the fault-detection latch
  worked, but the instruction was *also* quietly executing and repeatedly
  committing a bogus zero result the whole time. Caught by a dedicated
  fault-path testbench, not the main one - see README.md.

## 4. Why ADDI went first

Every architectural register resets to 0, and a register-register ALU op on
two zero operands can only ever produce zero — so without at least one
immediate-capable instruction, no test program can inject a non-zero value
and every "test" is vacuously true. ADDI is the minimum addition needed to
make dispatch testable at all, which is why it was wired first, ahead of
branch/load/store/jal/system.

## 5. Encoding-level safety property

An all-zero 32-bit word decodes as `ADD x0, x0, x0` (op=000=ADD, all
registers x0) — a harmless, discarded no-op. Unformatted/zero-initialized
instruction memory therefore fails safe rather than executing garbage.

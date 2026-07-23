# Regenerating the circuit diagrams

Commands to regenerate every SVG under `docs/images/` from the RTL, using
Yosys (`write_json`) piped into [netlistsvg](https://github.com/nturley/netlistsvg).
Run from this repository's root.

```bash
npm install -g netlistsvg   # one-time setup
```

## Top-level architecture (`riscv_ooo_top_arch.svg`)

Deletes every gate-level primitive belonging to `riscv_ooo_top` itself
while keeping its five sub-module instances as labelled boxes — a clean
architecture view instead of a flattened gate soup.

```bash
mkdir -p docs/images
yosys -p "
read_verilog -I rtl rtl/fetch_unit.v rtl/decoder.v rtl/branch_unit.v rtl/lsu_frontend.v
read_verilog -I rtl rtl/register_alias_table.v rtl/reservation_station.v rtl/reorder_buffer.v
read_verilog -I rtl rtl/integer_alu.v rtl/common_data_bus.v rtl/ooo_top.v rtl/riscv_ooo_top.v
hierarchy -top riscv_ooo_top
proc
opt_clean
select riscv_ooo_top/t:\$* riscv_ooo_top/t:\$paramod* %d riscv_ooo_top/w:\$* %u
delete
select riscv_ooo_top
write_json -selected docs/images/riscv_ooo_top_arch.json
"
netlistsvg docs/images/riscv_ooo_top_arch.json -o docs/images/riscv_ooo_top_arch.svg
```

## Out-of-order backend architecture (`ooo_top_arch.svg`)

Same technique, one level deeper — the RAT/RS/ROB/ALU/CDB sub-modules of
`ooo_top` as five boxes.

```bash
yosys -p "
read_verilog -I rtl rtl/register_alias_table.v rtl/reservation_station.v rtl/reorder_buffer.v
read_verilog -I rtl rtl/integer_alu.v rtl/common_data_bus.v rtl/ooo_top.v
hierarchy -top ooo_top
proc
opt_clean
select ooo_top/t:\$* ooo_top/t:\$paramod* %d ooo_top/w:\$* %u
delete
select ooo_top
write_json -selected docs/images/ooo_top_arch.json
"
netlistsvg docs/images/ooo_top_arch.json -o docs/images/ooo_top_arch.svg
```

## Small leaves (fetch_unit, decoder, branch_unit, lsu_frontend, integer_alu, common_data_bus)

Small enough (4-41 cells) to diagram whole, no filtering needed:

```bash
for LEAF in fetch_unit decoder branch_unit lsu_frontend integer_alu common_data_bus; do
  yosys -p "
  read_verilog -I rtl rtl/${LEAF}.v
  hierarchy -top ${LEAF}
  proc
  opt_clean
  write_json docs/images/${LEAF}.json
  "
  netlistsvg docs/images/${LEAF}.json -o docs/images/${LEAF}.svg
done
```

## Reservation station / register alias table / reorder buffer (reduced instance)

These are wide parameterized arrays (8 RS entries / 32 registers / 16 ROB
entries in production) — thousands of cells at full size. `chparam`
overrides the array size before elaboration so the diagram shows the
real per-entry structure at a legible scale.

```bash
yosys -p "
read_verilog -I rtl rtl/reservation_station.v
chparam -set RS_DEPTH 1 reservation_station
hierarchy -top reservation_station
proc
opt_clean
write_json docs/images/reservation_station_1entry.json
"
netlistsvg docs/images/reservation_station_1entry.json -o docs/images/reservation_station_1entry.svg

yosys -p "
read_verilog -I rtl rtl/register_alias_table.v
chparam -set NR 2 register_alias_table
hierarchy -top register_alias_table
proc
opt_clean
write_json docs/images/register_alias_table_2reg.json
"
netlistsvg docs/images/register_alias_table_2reg.json -o docs/images/register_alias_table_2reg.svg

yosys -p "
read_verilog -I rtl rtl/reorder_buffer.v
chparam -set DEPTH 2 reorder_buffer
hierarchy -top reorder_buffer
proc
opt_clean
write_json docs/images/reorder_buffer_2entry.json
"
netlistsvg docs/images/reorder_buffer_2entry.json -o docs/images/reorder_buffer_2entry.svg
```

Production sizes for reference: `RS_DEPTH=8` (1363 cells unreduced),
`NR=32` (1228 cells), `DEPTH=16` (526 cells).

Every SVG this produces has a white background rect injected after the
opening `<svg>` tag so it renders correctly embedded in GitHub markdown
regardless of light/dark mode. If you regenerate a diagram from scratch,
re-add it:

```bash
python3 -c "
import re
f = 'docs/images/YOUR_FILE.svg'
c = open(f).read()
m = re.search(r'(<svg\b[^>]*>)', c)
open(f, 'w').write(c[:m.end()] + '\n  <rect x=\"0\" y=\"0\" width=\"100%\" height=\"100%\" fill=\"#ffffff\"/>' + c[m.end():])
"
```

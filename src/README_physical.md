# Step 5: Physical design (floorplan → placement → routing)

Files: `lef.cpp`, `floorplan.cpp`, `place.cpp`, `groute.cpp`, `def.cpp`, `fastpnr_main.cpp`.
Check scripts: `tools/check_def_openroad.sh`, `tools/compare_place_openroad.sh`, `tools/compare_groute_openroad.sh`.

`fastpnr` turns a gate-level cell netlist into a physical design. **The floorplan, placement and global routing are done.** Detailed routing comes next.

```
cell netlist + LEF ── floorplan ── global placement ── legalization ── detailed placement ── global routing ── DEF + route guides
```

## The two file formats

- **LEF** (Library Exchange Format) is the PDK's physical catalogue.
  - The *tech LEF* lists the metal layers with their direction, track pitch, width, spacing, resistance and capacitance, and the vias between them.
  - The *cell LEF* gives every cell's outline, the metal shapes of its pins, and the areas the router must avoid (OBS).
- **DEF** (Design Exchange Format) is the blueprint of one design:
  - the die size and placement rows
  - the routing tracks
  - every cell instance with its position
  - the I/O pins and the power grid
  - later, every routed wire (today: placed cells; global routes go to a separate guide file)

OpenROAD, KLayout and Magic all read DEF, so every stage can be checked by another tool.

## LEF reader (`lef.cpp`)

It reads everything later steps need:
- layers: type, direction, pitch, offset, width, spacing tables, minimum area, via enclosure, R and C
- vias, sites, and cell macros: size, class, pins with shapes and USE, obstructions

All coordinates are integers in database units (1 nm for IHP). Cut layers without a `WIDTH` (IHP's Via1–Via3) take their cut size from the via definitions.

`libraryFromLef` builds the cell list straight from the LEF, so `fastpnr` reads netlists without any Liberty file. Bus pins such as `A_ADDR[0..9]` are grouped back into buses.

## Floorplan (`floorplan.cpp`)

1. **Tie cells.** Pins tied to a constant (`1'b0` / `1'b1`) get a `tielo` / `tiehi` cell each, as OpenROAD's `insert_tiecells` does. DEF has no constants.
2. **Size.** The core area is the cell area divided by the utilization (default 50%), plus each macro with its halo. The core is snapped to the 0.48 µm site grid and to whole 3.78 µm rows. Its origin sits on the Metal1 track grid, so the rows line up with the tracks. `-die W H` fixes the size instead.
3. **Macros (SRAMs).**
   - They are packed left to right from the top of the core, in N orientation, so their pins (on the bottom edge) face the logic.
   - A 20 µm column of rows is always kept beside them, so the power grid can run past the macros.
   - No cells are placed in a 10 µm halo around each macro.
4. **Rows.** They alternate N and FS, so neighbouring rows share a power rail, and are cut around the macro halos.
5. **Tracks.** They come from LibreLane's `tracks.info` for sg13cmos5l, the same as OpenROAD uses: Metal1/Metal3 every 0.42 µm, Metal2/Metal4 every 0.48 µm. Without that file, the LEF pitches are used.
6. **I/O pins.**
   - Inputs go on the left and bottom edges, outputs on the right and top. Left and right pins are on Metal3, top and bottom pins on Metal2.
   - Each pin sits on a routing track, spread evenly, on every other track when there is room.
   - A pin is the minimum metal width, and long enough for the minimum-area rule.
7. **Power grid (PDN).**
   - Metal1 rails, 0.44 µm wide, run along every row boundary. VSS and VDD alternate.
   - Vertical Metal4 straps, 2.2 µm wide, run every 75 µm, cut around the macros.
   - Horizontal TopMetal1 straps, 3.2 µm wide, run every 75 µm, also over the macros.
   - Vias go wherever the same net crosses. The Metal1 → Metal4 stack is written as three single-cut vias, as OpenROAD's pdngen writes it.
   - TopMetal1 drops vias onto the SRAMs' own Metal4 power pins.
   - Cut arrays respect the cut size, spacing and enclosure rules from the tech LEF.
   - Two repair passes make sure no part of the grid floats. A rail that no strap reaches gets its own strap. A strap segment that no top strap crosses (for example under a macro) gets a top strap.
8. **Self-check** (`checkFloorplan`):
   - rows stay inside the core, on the site grid, and clear of macros
   - the cells fit in the rows
   - every pin is inside the die, on a track, and has its own spot
   - macros don't overlap
   - no constant connections are left

## Placement (`place.cpp`)

**1. Global placement (SimPL style).** Two placements are kept, and each iteration brings them closer together:
- **Lower bound.** This placement minimises the total squared wire length.
  - Each net uses the bound-to-bound model: every pin connects to the net's two outermost pins, with weights that make the squared length behave like the real half-perimeter.
  - It is a large sparse linear system, solved by conjugate gradient without ever building the matrix (an edge list), warm-started from the last solution.
- **Upper bound.** The same cells spread out so that no region holds more cell area than 60% of its free row area.
  - Each overfilled region grows until it has room.
  - Its cells are then split recursively in proportion to the capacity on each side of a cut, alternating x and y. This is SimPL's look-ahead legalization.
- **Anchors.** Every cell is pulled towards its spread position. The pull grows by 1.3× each iteration, and the loop stops when the two bounds are within 8% of each other, usually after about 25 iterations.
- **Large nets.** Nets with more than 100 pins (clocks, resets) are left out. They would pull everything together.

**2. Legalization (Abacus).**
- Cells are taken from left to right.
- Each one tries the rows near its position, nearest first. In each row it joins the end of the row's clusters of touching cells.
- A cluster sits at the average of its cells' wanted positions, snapped to the site grid, and clusters that collide merge.
- The row with the smallest movement wins.

**3. Detailed placement**, repeated up to 6 times:
- **Optimal region.** Each cell moves into a free gap at the median of its nets' bounding boxes, or swaps with a cell of the same width there.
- **Pair swap.** Two touching neighbours swap when that shortens the wires.
- **Row re-spacing.** Each row is re-packed by the Abacus clustering, with every cell aimed at its optimal interval and the order kept. A row's change is kept only when its wire length drops.
- A move is kept only if the real half-perimeter wire length of the cell's nets goes down. Nets with more than 100 pins are not costed: moving one cell barely changes them.

**Self-check** (`checkPlacement`): every cell is inside a row, on the site grid, in the row's orientation, with no overlaps. OpenROAD's `check_placement` also accepts every placement.

## Global routing (`groute.cpp`)

**Grid.** Gcells are 15 Metal3 tracks (6.3 µm), the same as OpenROAD's FastRoute. Signals use Metal2–Metal4.

**Capacity.**
- An edge between two gcells has as many tracks as cross that boundary.
- A track is lost where the power grid or a macro (its obstructions and pins) covers the crossing point, counting the layer's width and spacing.
- 30% of the rest is held back for detailed routing, like `set_global_routing_layer_adjustment 0.3`.
- On mac32 this gives 32,998 / 37,061 / 31,118 tracks on Metal2 / Metal3 / Metal4. FastRoute counts 32,970 / 37,139 / 30,941.

**Routing**, in 2D, with the capacity of each direction's layers summed:
1. **Pattern routing.** Pins are taken in Prim order. Each pin is joined to the nearest point of the net's tree so far, by the cheaper of the two L shapes.
2. **Negotiated rip-up and reroute (PathFinder).**
   - Edge cost = (1 + history) × (1 + present penalty × overflow). The penalty grows by 1.5× per round, and the history grows on every overflowed edge, so persistent hot spots become expensive.
   - Nets crossing an overflowed edge are ripped up and rerouted by A* maze search.
   - The search starts from the whole tree, so a new pin can join it anywhere. That gives Steiner-like trees.
   - The search area starts small (around the pin and its nearest tree point) and widens only if needed, which keeps nets with 2,000 pins fast.
3. **Layer assignment.**
   - Straight runs go onto a layer of their direction, choosing the least used one that has room.
   - A run that would overflow on every layer is split, and each edge chooses its own layer.
   - Vias are counted at every node, from its lowest layer (including the pin's) to its highest.

**Output:** route guides in OpenROAD's format (`-guide FILE`), ready for a detailed router.

## Independent check by OpenROAD

`tools/check_def_openroad.sh design.def` loads our DEF into OpenROAD (docker image `openlane2:2.3.10`) and runs:
1. `read_def`
2. global and detailed placement into our rows, then `check_placement`
3. `check_power_grid` on VDD and VSS: every placed cell must reach the grid
4. `global_route` over our tracks and pins

All four pass on these designs:
- `mul16`: 1,480 cells
- `mac32`: 6,490 cells
- `sram_buffer`: 1P 1024×8 SRAM
- `fifo_2p`: 2P 256×8 SRAM
- `constant_folding` and `old_casez`: 2 cells and 0 cells, the minimum-size core

All 50 designs in the test suite floorplan cleanly under the self-check.

## Results against OpenROAD

Same floorplan for both tools; wire length is measured by the same routine inside OpenROAD. Global routing runs both routers on fastpnr's placement, with the same layers and adjustment.

| Design | Cells (with tie cells) | Placement HPWL vs OpenROAD | Placement time | Routing overflow (ours / FastRoute) | Routed wire vs FastRoute | Routing time |
|---|---|---|---|---|---|---|
| acc16 | 89 | −15.7% | 4 vs 21 ms | 0 / 0 | +0.5% | 0.4 vs 145 ms |
| add64 | 344 | −14.6% | 12 vs 59 ms | 0 / 0 | −0.6% | 2 vs 118 ms |
| mul16 | 1,480 | −6.8% | 123 vs 231 ms | 0 / 0 | −1.6% | 8 vs 199 ms |
| mac32 | 6,490 | −1.5% | 584 vs 1,199 ms | 0 / 0 | −1.9% | 53 vs 492 ms |
| fifo_2p (2P SRAM) | 244 | −4.2% | 12 vs 91 ms | 0 / 0 | −1.6% | 9 vs 126 ms |
| sram_buffer (1P SRAM) | 66 | −46.8% | 1 vs 533 ms | 0 / 0 | −0.4% | 4 vs 147 ms |
| mac32_x8 | 51,920 | −17.8% | 3.8 vs 10.9 s | 0 / 0 | −2.4% | 1.2 vs 3.6 s |

Lower HPWL is not yet proof of a better layout. Detailed routing and timing will decide that.

## Speed

Floorplanning `mac32` (6,490 cells) takes 66 ms in total. Most of that is reading the 1.7 MB cell LEF and the netlist; the floorplan itself takes 0.6 ms.

OpenROAD takes 285 ms in total and 64 ms for the floorplan step itself. Its run does the same work: `initialize_floorplan`, `make_tracks`, `insert_tiecells`, `place_pins` and `pdngen` with the same grid. OpenROAD also has to read the Liberty file, because `insert_tiecells` needs it.

## Next

- Detailed routing from the guides. The GPU will route batches of nets whose bounding boxes do not overlap, one thread block per net.
- Extraction (SPEF) and timing-driven routing with lsta.

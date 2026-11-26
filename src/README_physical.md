# Step 5: Physical design (floorplan → placement → routing)

Files: `lef.cpp`, `floorplan.cpp`, `place.cpp`, `groute.cpp`, `droute.cpp`, `def.cpp`, `fastpnr_main.cpp`.
Check scripts: `tools/check_def_openroad.sh`, `tools/compare_place_openroad.sh`, `tools/compare_groute_openroad.sh`, `tools/compare_droute_openroad.sh`, `tools/drc_klayout.sh`, `tools/lvs_klayout.sh`.

`fastpnr` turns a gate-level cell netlist into a routed layout. **The floorplan, placement, global routing and detailed routing (`-droute`) are done.** The routed designs pass IHP's own DRC and LVS decks, SRAM designs included.

```
cell netlist + LEF ── floorplan ── placement ── fillers ── global routing ── detailed routing ── routed DEF
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
   For an IHP Tiny Tapeout digital standard-cell tile, pass `-tt-template path/to/tt_block_3x2_pgvdd.def`. The template supplies the die outline, fixed signal pin rectangles and supply names. Both supplies use near-full-height Metal4 ports over the vertical stripes, with via stacks down to the Metal1 row rails. The tile mode supports standard cells. The default grid also supports SRAM macros.
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

## Detailed routing (`droute.cpp`, `-droute`)

**Correct by construction.** Instead of creating violations and repairing them, the router only builds legal metal:
- **The grid.** Nodes sit where the Metal2/Metal4 tracks (every 0.48 µm) cross the Metal1/Metal3 tracks (every 0.42 µm). Wires run only along tracks, at minimum width, in each layer's direction. The track pitches already give side-by-side spacing.
- **Vias.** Only the PDK's single-cut vias whose pads lie along each layer's direction are used: Via1_XY or Via1_YY, Via2_YX and Via3_XY. So pads on neighbouring tracks never touch.
- **End-of-line spacing.** Two different nets never use neighbouring nodes of the same track. This one rule covers spacing at wire ends and between via pads.
- **Obstacles.** The power grid and macros block every node whose wire or pad would come too close, using the widest-wire spacing.
- **Pins.** Metal around an I/O or macro pin is reserved for that pin's net.

**Standard-cell pin access**, computed once per cell master and orientation (84 masters × 2):
1. **On-grid points** whose Via1 pad lies inside the pin.
2. **If none: off-grid.** A via with its pad inside the pin, placed between two grid rows and/or up to 0.06 µm beside a Metal2 track. That is the most that still keeps spacing to the neighbouring tracks. A Metal2 patch covering the via pad and the node joins it to the track.
   - The node on the other side of the via, which the pad reaches towards, is reserved. Where an obstacle already blocks that node, the real distance from the pad to the obstacle is checked instead.
   - Where another pin reserved the same node, it becomes a *soft* claim that negotiation resolves. A reserved node is never metal, so its neighbours stay free.
3. **Last resort:** a pad that overlaps the pin. It must keep Metal1 spacing from all other shapes, and must not leave a notch against other parts of the same pin.
4. **Compatible set.** The pins of a cell get access points at least a pad height plus spacing apart on any one Metal2 track. Pins with the fewest options choose first (TritonRoute calls this a pin-access pattern).

**SRAM pin escapes.** IHP's SRAM pins are 0.26 µm Metal2 squares on the macro's bottom edge, as little as 0.51 µm apart. That doesn't fit the 0.48 µm tracks, so no on-track wire can reach them legally. Each pin gets an escape instead:
- a Metal2 stub straight down from the pin centre, off-track (stubs 0.51 µm apart keep 0.31 µm spacing)
- a Via2 on a Metal3 row below the macro
- a Metal3 patch along the row to the nearest node, which becomes the pin's terminal

Neighbouring pins take different rows ("levels"), assigned left to right so that no two nets' Metal3 comes within spacing. Metal2 nodes near a stub are blocked for every net.

**Routing.**
- A* search on the 3D grid, from the whole tree of the net built so far, cheap inside the global-route guides and three times as expensive outside them.
- The estimate is weighted ×1.2. On mac32 that halves the search time (17.5 s → 8.4 s) for 0.2% more wire.
- Conflicts are resolved by negotiation (PathFinder): history and present-congestion costs on nodes, ripping up only the conflicting nets.
- **When negotiation stalls for 3 rounds**, every net within 3 nodes of a conflict is ripped up as well, so the neighbours the conflicting nets would have to squeeze past can move too.

**Three choices that keep it fast on dense designs:**
- **Certify the leftovers instead of exact rules everywhere.** The search keeps the cautious rule (no two nets on neighbouring nodes of a track). Only the conflicts left at the end of a round are measured against real geometry:
  - a wire end reaches half a width past its node, a via pad half a pad length
  - a pair whose gap still meets the layer's spacing is accepted: on Metal3 two wire ends, or a pad and a wire end; on Metal2/Metal4 two wire ends

  Easy designs end with no conflicts, so they pay nothing.
- **Short wrong-way jogs.** A hop to the next track on the same layer costs 2.5× its length. It replaces a two-via detour over the other layer: vias drop about 20%, and negotiation needs about half the rounds.
- **Wires as wide as the via pads.** Via2/Via3 bottom pads are 0.21 µm wide against a 0.20 µm minimum wire. Metal2/Metal3 are drawn 0.21 µm wide (a DEF non-default rule), so wires and pads never form 5 nm steps. The spacing still holds: 0.27 µm on Metal2, exactly 0.21 µm on Metal3.

A stalled negotiation also reroutes the nets within a few nodes of each conflict (at most 5) and raises the history cost there, so nets passing through a stuck spot move away. Following global routing's layer choices was tried and added vias, so it is off.

**Finishing.**
- Same-net nodes next to each other are joined.
- Short wire pieces are extended to the minimum-area rule.
- Two pieces of one net on a track that come closer than the spacing (via pads and off-grid patches alike) are bridged at pad width, so no notch is left.
- Filler cells close every gap in the rows, keeping the N-wells legal.
- Top-level VDD and VSS pins go on the TopMetal1 straps.

**Checks.**
1. **Our own:** conflicts, plus opens found by union-find over every net.
2. **`tools/drc_klayout.sh`:** the DEF becomes GDS with the real cell and SRAM layouts and goes through **IHP's KLayout DRC deck**.
3. **`tools/lvs_klayout.sh`: IHP's KLayout LVS deck.** It extracts every transistor from the layout and compares the circuit with a schematic built from the DEF and the cells' CDL. A match means no opens and no shorts.
   - The PDK's own SRAM GDS and CDL don't compare clean with this deck even on their own, so SRAMs are black boxes there.
   - Each SRAM pin is instead probed in the extracted metal: it must share a net with another pin of its net, and with no other net.
   - A planted open (one via removed) is caught by both parts.

**Results:**
- **Every synthesized test design routes completely (47 of 47):** 0 conflicts, 0 opens, every pin reached.
- **IHP's DRC deck:** 0 violations on acc16, add64, mul16, mac32, mul8, shifters, alu4, cpu_like, sram_buffer and fifo_2p. Metal density is set aside, since there is no fill yet.
- **IHP's LVS deck:** match on acc16, add64, mul16, mac32, sram_buffer and fifo_2p. The SRAM pins are clean too: 52 and 108 pins probed, no opens or shorts.

Full-flow benchmarks, OpenROAD on 16 threads, are in the main README (`tools/benchmark_flow.sh`). The older one-thread-each comparison below (`tools/compare_droute_openroad.sh`, before the three choices above):

| Design | Time | IHP DRC | Wire | Vias |
|---|---|---|---|---|
| mul16 (1,514 nets) | fastpnr 1.3 s, TritonRoute 251 s: **190× faster** | 0 / 0 | fastpnr +2.0% | fastpnr +24% |
| mac32 (6,556 nets) | fastpnr 8.1 s, TritonRoute 1,116 s (18.6 min): **138× faster** | 0 / 0 | fastpnr +1.3% | fastpnr +24% |

Both layouts are judged by the same IHP KLayout DRC deck, and fastpnr's also passes LVS.
TritonRoute uses fewer vias and slightly less wire. Part of the difference is that it also routes on Metal1. fastpnr never does, so every pin costs a Via1 and every turn a via. Raising our via cost barely changes the count.

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

Lower HPWL is not by itself proof of a better layout. Detailed routing (above) and timing decide that.

## Speed

Floorplanning `mac32` (6,490 cells) takes 66 ms in total. Most of that is reading the 1.7 MB cell LEF and the netlist; the floorplan itself takes 0.6 ms.

OpenROAD takes 285 ms in total and 64 ms for the floorplan step itself. Its run does the same work: `initialize_floorplan`, `make_tracks`, `insert_tiecells`, `place_pins` and `pdngen` with the same grid. OpenROAD also has to read the Liberty file, because `insert_tiecells` needs it.

## Next

- Fewer vias: short Metal1 connections inside and between neighbouring cells.
- GPU routing: batches of nets whose bounding boxes do not overlap, one thread block per net. This needs the CUDA toolkit.
- Extraction (SPEF) and timing-driven routing with lsta.
- Metal fill for the density rules, and GDS output.

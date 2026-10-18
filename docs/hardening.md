# GF180 hardening investigation

The [first full-controller GDS run](https://github.com/j-xu0/ttfg-verilog/actions/runs/39098872381)
for PR #5 failed in OpenROAD global placement (`GPL-0301`). RTL tests and docs
passed. The lower interface/planner PRs passed GDS because the full motion logic
was not connected to their top levels.

This is an area overflow, not a functional-test failure or a fatal lint error:

| Placement metric | Failed 1x1 run |
| --- | ---: |
| Core area | 51,966.970 um² |
| Movable cell area before pin-density adjustment | 145,526.394 um² |
| Fixed cell area | 1,527.859 um² |
| Movable area after pin-density adjustment | 168,852.553 um² |
| Reported utilization | 334.765% |
| Placement target | 60% |

Increasing placement density cannot fit a design exceeding 100% utilization.
The allocation is changed from 1x1 to **3x2 (six tiles)**, retaining the 60%
placement target and the 10 MHz / 100 ns clock constraint. The functional RTL
and source list are unchanged by this fix.

The official GF180 templates used by the failing run came from
[tt-support-tools at 01d5d281](https://github.com/TinyTapeout/tt-support-tools/tree/01d5d2814fa9dd61e9d211e0b235a4a592a9316a/tech/gf180mcuD/def).
Template die dimensions and the existing core margins give this comparison:

| Allocation | Die dimensions (um) | Estimated core area (um²) | Adjusted-area/core estimate |
| --- | --- | ---: | ---: |
| 1x2 | 346.64 × 325.36 | 107,931.40 | 156.44% |
| 2x2 | 711.20 × 325.36 | 223,686.49 | 75.49% |
| 3x2 | 1,075.76 × 325.36 | 339,441.58 | 49.74% |

These estimates reuse the failed run's adjusted movable area and exclude the
new floorplan's fixed-cell/blockage changes. They select 3x2 as the smallest
listed allocation with headroom under the existing 60% target; they do not
prove completed placement, routing, timing, or signoff. A new full GF180 run
must establish those results. Returning this controller to 1x1 would require a
substantial architectural area reduction; that is separate from correcting the
provisional floorplan allocation.

Local validation for this configuration-only fix checks the selected template
size and estimated headroom, unchanged clock constraint, and diff formatting.
RTL suites already passed on the identical logic. GitHub's GDS workflow must
rerun hardening, precheck, and gate-level tests against the new allocation.

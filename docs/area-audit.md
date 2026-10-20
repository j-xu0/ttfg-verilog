# Follow-up area audit (before the shared engine)

This records the independent-engine milestone at `4539f9d`. The subsequent
[shared-engine implementation](shared-controller.md) reduces area further to
58,174.9952 um²; the historical comparisons below retain their original values.

The complete controller still does **not** fit the mandatory 1x1 tile. This audit
reduces controlled GF180 mapped area from **71,420.8320 to 61,318.5216 um²**
(14.1%) without removing a feature or reducing numeric precision. The physical
core is 51,966.970 um²; the new cells alone occupy 118.0% of that area, before
placement, clock-tree and routing overhead. Keep the PR in draft.

These are synthesis measurements using the same Yosys/ABC procedure and filtered
GF180 Liberty as the previous comparison. Standalone module areas are useful
for comparing a change within a module; they are not additive because full-top
flattening shares constants and removes unused outputs. The source/library
hashes and full-top totals are in [area-measurement.json](area-measurement.json).

| Module | Before (um²) | After (um²) | Exact change |
| --- | ---: | ---: | --- |
| Planner | 29,420.0704 | 24,013.2928 | Stream both operands and destinations through rotating registers; decode operations from the existing state instead of saving an instruction. |
| PI | 18,259.6736 | 16,595.7120 | Remove redundant wide rotation muxes, scratch reset/abort muxes and duplicate busy state. Subtract only integer position bits; retain the fractional bits exactly. |
| STEP/DIR | 10,716.9664 | 8,308.8320 | Share mutually exclusive pulse timers and overlapping setup timers; fold negative-magnitude correction into phase addition; prune redundant modulo reset gates. |
| SPI | 7,347.3344 | 6,901.7088 | Receive scratch needs neither reset nor frame-start clear: all 16 bits are overwritten before a frame may commit. |
| Encoder | 3,422.3168 | 2,820.8320 | Share one increment/decrement toggle chain and decode Gray transitions directly. |
| Timebase | 1,865.9200 | 1,865.9200 | Keep the simpler existing counter after measuring alternatives. |

The integrated design now uses 392 flip-flops instead of 415, and 2,480 mapped
cells instead of 3,000. Sequential cell area is still 27,005.3504 um². Thus
storage alone occupies 52.0% of the physical core, while the remaining mapped
logic occupies another 34,313.1712 um². Local arithmetic rewrites have helped,
but storage and register-selection logic are now the main architectural issue.

## Remaining exact opportunities

1. **Share the planner and PI engine and scratch storage.** Both run for less
   than 700/450 clocks respectively within a 10,000-clock control interval.
   Schedule PI sampling before planner workspace changes, then execute both
   with one arithmetic engine and shared temporary registers. Persistent
   position, stopping, integral, command and NCO state must remain distinct
   where their lifetimes overlap. Sharing an adder alone offers little now;
   avoiding duplicated storage and its wide selection/hold muxes matters more.
   This is the strongest next direction, but its saving has not been measured
   and is not a fit guarantee.
2. **Use a common internal magnitude/sign command format.** The PI clamp already
   processes magnitude bits; handing those directly to STEP could avoid
   converting to signed representation and back. It would preserve external
   behavior but requires coordinated module changes and parameter regression.
   The remaining saving is likely modest; no area number is claimed here.
3. **Revisit the stopping-distance representation together with shared storage.**
   Computing the finite sum instead of retaining 29 stopping bits only helps if
   division/product temporaries reuse other registers. A separate arithmetic
   unit could cost more than the storage it replaces. The current accepted
   stopping sum can approach 2^29, so simply narrowing it loses valid moves.

SPI atomic commit inherently needs both the incoming frame and the current
configuration. The absolute encoder, reference, velocity, integral, command and
phase state also have overlapping lifetimes. Further large reductions need a
whole-controller lifetime/storage design, not another independent serial unit.

## Small descopes and rejected experiments

No descope below was applied. Diagnostic variants were synthesized outside the
checkout; their area results do not establish functional or physical validity.

| Option or experiment | Measured result | Assessment |
| --- | --- | --- |
| Fix one PI gain pair | Only 43.904 to 206.3488 um² saved in standalone PI for the two tested pairs, 00/00 and 11/11. | Losing gain selection buys very little. |
| Fix one complete motion profile | Full-top area 54,923.9040 for profile 0, or 54,660.4800 for profile 63. | Even removing all profile selection still exceeds the core before physical overhead. These are two diagnostic points, not a sweep of every profile. |
| Reduce Q16 fractional precision | Not implemented or mapped across the complete controller. Q12 changes velocity quantum from 0.01526 to 0.24414 steps/s at 1 kHz, and rounds acceleration coefficients differently. | A numerical tradeoff requiring a new error budget and regression; preserving the named profiles does not preserve their exact trajectories. No fit claim. |
| Binary down-counting timebase | 1,993.2416 um², worse than 1,865.9200. | Rejected. |
| Special shortened-LFSR timebase | 1,811.0400 um², saving only 54.8800. | Rejected: a special default-only implementation adds complexity for little total saving. |
| PI integral initialization flag instead of resetting its ring | 16,810.8416 um², worse by 215.1296. | Reset savings were outweighed by masking and initialization muxes. Rejected. |
| Planner alternative ROM bit-plane encodings | Both measured encodings increased area. | Keep the generated 64-profile table. Removing a lookup does not necessarily remove logic. |

Small cuts are therefore poor substitutes for the shared-storage redesign.
The design needs at least another 9,351.5516 um² reduction merely to get cell
area below the core, and more for physical headroom. Changing placement density
cannot make the present 118.0% raw cell occupancy fit.

## Validation

- 24 module cases cover all 64 planner profiles, both directions, signed
  endpoints, ignored ticks, captured/stable configurations, aborts, all PI gain
  pairs, saturation/anti-windup and randomized arithmetic. The planner latency
  bound includes the extra in-flight tick stimulus.
- STEP uses an independent original-arithmetic model for cycle-by-cycle
  comparison. Two new parameter cases cover asymmetric setup/hold/pulse timers
  and fractional widths wider than the signed command input.
- Yosys sequential equivalence proves all 22 encoder state/output bits against
  the previous implementation after normalizing asynchronous resets.
- Seven top-level motor/port tests cover missed steps, disturbance recovery,
  abort/fault behavior, signed limits, the 10 kHz ceiling and serial sampling.
- Full-design synthesis/check passes with no runtime divider/modulo cells.

RTL tests and mapped area are separate from placement, routing, STA, DRC and LVS.
The 1x1 GDS check must remain enabled and must pass before claiming a fit.

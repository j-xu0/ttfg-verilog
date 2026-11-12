# Measurement prototypes (not built into the chip)

These files are scratch prototypes used to price architectural options with
`scripts/measure_gf180_area.py`. Nothing here is in `PROJECT_SOURCES` and no
workflow compiles this directory.

* `pi_parallel_full.v` — dedicated parallel Q11 PI with every feature of the
  current shared engine (4 kp/ki codes, feedforward, command clamp +
  `saturated`, integral clamp, anti-windup freeze). Measured 16,429 µm²,
  745 cells (±3 % mapping noise). Compare with the serial PI's marginal of 15,981 µm²: un-sharing
  the PI does **not** save area while a full PI is kept.
* `pi_parallel_p_only.v` — same datapath without the integrator, its clamp and
  the anti-windup freeze. Measured 8,311 µm², 423 cells. This is the only
  measured way to keep a *closed loop* and still fit: planner 12.9 k + this
  8.3 k ≈ 39.9 k top ≈ 92 % CI utilisation.
* `bench/tb_engine_sweep.v` — Icarus bench that ticks `shared_motion_controller`
  `+goal=<steps> +code=<profile>` and prints position/speed/brake-sum per tick
  for diffing against the Python oracle (see `docs/q12-handoff.md` §9). It lives
  in a subdirectory so the `--source-dir experiments` glob above only sees the
  synthesizable prototypes.

Measure with:

```sh
python3 scripts/measure_gf180_area.py --lib <gf180 lib> --yosys <yosys> \
  --abc <yosys-abc> --source-dir experiments --top pi_parallel_full --output /tmp/a
```

# Day 2 — Lookahead Planner Arc (and Removal)

Starting point: day-1 sensor driver at ~2:00–2:04, top ~161. Backup: `torcs_jm_par_v1.py`
(1:59 code, untouched since).

## 1. Planner branch built

`TrackMemory`: ring buffer of `(distRaced, centerMin, diff)` per frame, 150 m
lookahead, tightest-point plan `{speed, track_pos, severity, dist_to_tightest}`.
Physics brake from required deceleration. Old sensor stack kept as cold-start
fallback. Throttle/gears/TC/recovery untouched.

## 2. Planner never fired — distRaced is cumulative

`sev = -1` on every row, both laps. Cause: wrap detection assumed `distRaced`
resets per lap. It doesn't — lap 2 continued 3617 → 7226. Buffer promoted never.
Fix: key everything on `distFromStart` (per-lap, wraps 3600 → 0). Added `lapDist`
telemetry column to prove wraps.

## 3. Planner sign inverted

Lap 2 lined up on the INSIDE (trace: `trackPos -0.19 → -0.97` into a wall on a
right-hander entry). Root cause: comment claimed "right-hander → diff > 0".
Wrong. Right-hander → right beams hit inside wall (short), left beams look past
(long) → `diff = right − left` is NEGATIVE. Verified against the SCR manual
(`+1 = left edge`, `-1 = right edge`): outside of a left-hander (`diff > 0`)
is negative trackPos. One-line fix (`sgn = -1 if diff > 0`), verified
outside → apex → outside in test. Note: the sensor fallback was already
correct (`-diff * 0.08`) — which is why lap 1 always worked and lap 2 didn't.

## 4. Equilibrium stall at 0.44 — pursuit steering

Car reached `trackPos 0.44` toward a ±0.75 line target and stopped closing:
angle term and position term canceled to net ~0 steer. Fix: lead angle
(`desired_angle = 0.30 * err`) when plan active — deliberately point
off-parallel to get there. Fallback law untouched.

## 5. 150 m coast — velocity profile target

Planner published raw corner speed (60) 150 m out → throttle fully lifted,
`apply_brakes` never fired (only ~1.3 m/s² needed over 150 m) → long slow
coast, slow entries. Fix: target follows `v(d) = sqrt(v_c² + 2·8·d)` braking
profile; brake fires on profile deviation. Same curve both sides, no more
coast-then-panic shape.

## 6. Planner killed — garbage playback

Lap 2 drove off track and pinned at a wall (`trackPos = 3.2`) with
reverse/forward oscillation. Cause: lap-1 buffer holds lap-1's
centre-hugging line; planner plays it back as truth on lap 2. A planner is
only as good as its map, and our map was drawn by a bad driver.
Decision: delete `TrackMemory`/`compute_plan`/`get_plan`, sharp-entry and
exit-flatten blocks, latch/ramp/near-gate state. Sensor fallback (F1 wide-fan
target, entry-apex-outside line) became the whole driver. Lap 1 ≈ 1:50 held;
laps 2+ match instead of dying.

## 7. Post-kill fixes (kept)

- Narrow-fan target (±1.7° sets distance; ±12° beams grazed walls and
  oscillated target 88–109). Direction still from wide fan.
- Line: entry → apex flip → unwind from single frames, continuous magnitude,
  EMA 0.92 + deadband + PB-calm divisor after killing a ±0.5 straight-line
  oscillation across three rounds (gain boost → divisor → deadband → EMA).
- Upshift 9000 → 12000 (redline ~13k; pulls past 12176 loaded). 14000
  rejected: plateau at ~12200 is a power ceiling, not a shift prompt.
- Proportional TC + 25% floor (6th@9166 doing 132 was all smoke).
- Gear downshift guards (5th lived 13 rows at 116–121 → 4↔5 hunt capped top
  at 121; wider high-gear guard fixed it).
- Wall-pin package: launch override gated on-track, TC stuck-spin kill
  (speed<5 + rpm>10000 → cut), windowed no-progress detector (rate form —
  level form provably never fires on creep). Verified: creep fires ~100,
  hairpin/start silent.

## 8. Known limits (accepted)

- Top speed ~195–202: 6th-gear power-vs-drag ceiling (~12200 rpm). No code
  changes it; different car XML would.
- `CORNER_BRAKE_SENSOR_IDX`/`CORNER_BRAKE_DISTANCE` constants are unused
  leftovers from the sensor-brake era. Harmless; remove if touching that area.
- Single-lap races get no planner benefit (needs a learning lap) — moot now
  that the planner is gone.

## Numbers

2:20 → 2:06 → 2:02(inv) → 2:04 → 2:00:07 → 1:59 → 1:55.89 (PB, sign fix) →
regressions (2:22 fan, 2:12 latch) → ~1:50s. Top: 84 → 104 → 121 → 161 →
197 → 216.6. Min: reverse finishes → positive laps. Damage: 381 → 0.

---

# Day 3 — Sensor-driver hardening → PB 1:38:34

Session after a short break. Starting point: sensor-only driver (~1:50s) after
the day-2 planner kill. All work in `torcs_jm_par.py`; race backup untouched.

## 9. Fallback centre/side split (DeepSeek batch, corrected)

Diagnosis right (wide-fan `min(track[2:17])` read the ±12° track edge ~33 m and
capped straights at ~122), gate wrong: `d_side < d_center*0.5` still fires on
straights (33 < 100). Fix: centre fan (±1°) by default, side fan only when
`d_side < d_center*0.5 AND asym > 15` (`asym = |d_left − d_right|`). Straight:
both sides ~equal, asym ~0 → centre (300). Corner entry: inside short, outside
long → side. Mock: straight 300.0 (was ~122), corner 72.9.

## 10. Yaw-first brakes + throttle cut + off-track 40 (applied as proposed)

`apply_brakes`: yaw check moved above the `speed < 20` early return (spins at
low speed still need brake), 1.0 above 1.0 rad. `calculate_throttle`: return
0.0 above threshold. Off-track cap 80 → 40 in both `_fallback_target` and
`get_adaptive_target`. Gear anti-hunt NOT applied as written (`GEAR_SPEEDS[g]`
is the next gear's floor — 150 for 5th — and would lug the limiter); applied
milder form (`GEAR_SPEEDS[g-1]` + angle gate). Launch override gated on yaw +
on-track; reverse steers 0.0 straight.

## 11. ±19° fan, stronger brakes, yaw 0.25 stepped

Side fan widened ±12° → ±19° (hairpin wall sits beside the car first; asym
gate still guards straights). Fallback brake `(over−3)/45 cap 0.85` →
`(over−2)/18 cap 0.95` (at over=20: 0.38 → 0.95; deadband 3 → 2 to match).
Plan brake `(a−3)/15` → `(a−2)/10`, guard 3.0 → 2.0. Yaw via lowered default
`BRAKE_THRESHOLD 0.35 → 0.25` with relative steps (0.5/0.8/1.0) so
`--parameters` sliders stay live. Launch 0.3 → 0.2. Reverse `tp*0.4`
deferred. Mock: hairpin target 62.8, over=109 → brake 0.95.

## 12. Plan `dist <= 2.0` dead-zone fix

Plan branch skipped braking when the tightest point was 0–2 m ahead — exactly
when braking is most needed (planner goes live mid-lap, first "tightest" is
0.2 m ahead, car 18 over, brake 0.0 → S-crash at 20700). Fix: skip only when
at/below target or `dist < 0.0`; floor `dist_eff = max(dist, 0.5)`.
Verified: 161.8 vs 143.4 @ 0.2 m → a_needed ≈ 432 → 0.95.

## 13. Planner deleted (second kill, final)

S-crash root cause on re-inspection: single-tightest `sgn` flipped frame to
frame on S-turns (both corners in window → 20 Hz steering oscillation), and
0.95 plan brake locked fronts (~0.66 g, no steering authority). Options were
average-diff `sgn`, 0.75 plan cap, or full removal. Chose removal: fallback
drove lap 1 cleanly and gives 0.83 where the plan gave 0.95 for the same
over=17. Deleted `TrackMemory`/`compute_plan`/`get_plan`, plan branches in
target/line/steer/brake, memory update, telemetry plan columns → stubs, then
stubs removed in cleanup (§17). Fallback is now the single code path.

## 14. Centre-gated ±45° widening

S-wall appeared in ±45° (minTrack 7.1) before ±19° caught it (d_side 48 @
20700). Fix: `d_center < 100` → fan ±45°, else ±19° (off-centre ±45° wall
touches at 5–10 m can't false-trigger on straights). Mock: straight 300,
S-turn ~62, symmetric corner stays centre-based via asym gate.

## 15. Stall deadlock → speed gates → hysteresis

Stall at speed ~0 with residual yaw: throttle cut (0) + yaw brake (0.5+) +
launch blocked (angle gate) = braked statue until stucktimer. Fix in two
steps: (a) gate yaw cut/brake on `speed > 5`, drop launch yaw gate, loosen
launch to `trackPos < 1.5`; (b) hysteresis after 4↔5 oscillation observed —
yaw gates at 15, launch releases at 10. Bands: 0–10 launch, 10–15 coast (no
interference), 15+ yaw protection. At 15 km/h 0.8 brake can't zero the car in
one step, so angle unwinds instead of oscillating.

## 16. Threshold 0.40 + soft tiers + 0.65 cap → PB 1:38:34

Telemetry showed 0.25 fires on normal S-turn yaw (0.26–0.43): `BRAKE_THRESHOLD`
0.25 → 0.40 (23° is a spin, 14° is cornering). Tiers softened 0.5/0.8/1.0 →
0.2/0.4/0.7 (offsets +0.15/+0.35) to keep front grip. TC chatter (accel
0.25↔1.0, crawl 14→50): cap 0.65 for 0 < speed < 30 (skipped at 0 for
breakaway), launch 1.0 → `max(accel, 0.7)`. "Final turn at 70" confirmed not a
bug (target 61–68, wall 2.3 m, brake 0.06–0.12, release to 300 on exit).
Result: PB 1:38:34, smooth S-turn.

## 17. Cleanup pass (no logic change)

Deleted `get_plan`/`compute_plan` stubs, `drive_example`, `CORNER_BRAKE_*`
(§8 leftover, now gone), inlined `get_race_line_target` into
`calculate_steering`. Telemetry docstring 25 → 10 steps (matches `% 10`).
Trimmed changelog-style comments to one-line "why this number" each;
`drive_modular` comments untouched. Verified: compiles, imports, grep for
dead names clean, mock outputs byte-identical (300 / 62.1 / 0.95 / tiers /
0.65 / 1.0 / 4→5). Driver logic ≈ 238 lines core, ~373 with loop + params;
file 1097.

## Numbers (day 3)

~1:50s → **1:38:34 (PB)**. Straight target 300 (was ~122). S-turn carried
40–50 instead of 15. Stall recovery: five-minute statue → sub-second
correction. Damage 0 held.

---

# Day 4 — 1:38:34 → PB 1:36:10 (brake later, TC looser)

Starting point: 1:38:34 sensor driver after the §17 cleanup. All work in
`torcs_jm_par.py`; race backup untouched. Note: the Day 3 text above (§9–17)
is still uncommitted working-tree draft; the commits below are what HEAD
(`aae4378`) actually contains.

## 18. Bulk cleanup (116825b, no intentional logic change)

608+/493− reformat/dead-code pass, single file. Setup for the PB run below;
no constants touched on purpose.

## 19. PB 1:37:94 — later braking, stricter side fan (9ce53b1)

- Centre fan `min` → `median` (one glitchy beam no longer sets distance).
- Side fan gated harder: `+ d_center < 150 + |trackPos| < 0.5` — no side
  grabs mid-corner or when already off-line.
- Braking decel 8 → 11 in the `v(d)` profile (brake later, carry entry speed).
- `over = speed − (target − 5)`: brake curve starts 5 km/h before target.
- 0.65 low-speed cap now applies at speed 0 too (static breakaway is where
  wheels spin most freely).
- TC launch fix: stays active below 10 km/h, bypass only at genuine
  standstill in the first 5 steps (the old `speed < 10` blanket bypass let
  launch wheelspin straight through).

## 20. Dead-code cleanup (8c456a8, "minimal only")

`__repr__` unreachable tail, impossible `g == −1`, inner `import os`,
`if cooldown: pass/elif` → explicit flag, redundant launch guards.
Behavior-identical by construction (every removed branch provably
unreachable); compiles clean.

## 21. Trail-brake taper → PB 1:36:13 (030a30c)

`apply_brakes(S)` → `(S, R)`: over-target brake tapered by live steer
demand (`× max(0.5, 1 − |steer|·0.4)`, floored at 50%). Yaw tiers untouched
(return before the taper). Untried lever until now: full brake + full steer
can't be requested at once, fronts keep authority mid-corner.

## 22. Looser TC → PB 1:36:10 (aae4378)

Slip threshold 2.0 → 3.5, cut range 5.0 → 6.0, floor 25% → 40%. Still
catches genuine wheelspin, stops holding quarter-throttle through exits.

## 23. Tried and reverted same session (no live lap)

Line EMA 0.92 → 0.75 (~240 ms → ~70 ms lag) applied, then reverted —
single-variable discipline, steer smoothing untouched throughout. Braking
decel 11 → 13 applied, then reverted. Neither saw a timed lap.

## 24. Calibration-table branch (unmerged)

`precomputed-calibration-table`: static `track_calibration.csv` (454 rows,
0→3618 m, built offline from this lap's actuals) as safety-gated override
in front of untouched fallback (minTrack < 2.0 / drift > 0.35 /
out-of-range + 15 m gap cap → defer, never blend; off-track branch stays
100% reactive). Offline replay finding: the shared 0.92 EMA lets rejected
ticks pollute `_prev_line_target`, so re-accepted table values steer against
~240 ms of stale history — prime suspect for non-convergence in complexes.
Per-tick `table_debug.csv` logger left in the tree for a live run. HEAD back
on `main` 1:36:10.

## Numbers (day 4)

1:38:34 → 1:37:94 → 1:36:13 → **1:36:10 (PB)**. Three PBs, two from
single-parameter changes (taper, TC).

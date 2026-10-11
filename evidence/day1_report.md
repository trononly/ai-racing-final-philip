# Day 1 — Corkscrew Solo Driver Report

Goal: finish a clean lap solo, then get fast. Car: `torcs_jm_par.py`, `drive_modular()` only.
No launcher talk here — driving logic only.

## 0. Baseline (what we started with)

- `TARGET_SPEED = 100` — safe but loses ~10s on straights. First change: 120 → 150 → 160.
- `BRAKE_THRESHOLD = 0.9` rad (~51°) — almost never braked on heading alone.
- `CORNER_BRAKE_DISTANCE = 30.0`, sensors `[6,7,11,12]` narrow ±1.7° — too late for corkscrew at speed.
- `GEAR_SPEEDS = [0,20,40,80,100,180]` then `[0,60,100,140,190,220]` — tall 3rd (90→130), never saw 4th on short straight.
- `shift_gears()` stateless — at 189.9 vs 190.1 flicked 4↔5 every step. The grinding you heard.
- `calculate_steering()` fixed gain — same steer at 60 and 150. Fast = flick off.
- `calculate_throttle()` incremental +0.4/-0.2, penalty `steer*2.5` signed (sped up one way).
- `apply_brakes()` treated off-track `-1` sensor as corner → full brake + `accel*(1-brake)` = grind to 0 and sit there.
- `STUCK_THRESHOLD = 100` (~2s), reverse 50 steps at 0.5 — too slow to trigger, too weak to escape.
- `fancyout()` crashed on `--debug` with `KeyError: 'stucktimer'` (first frames miss keys).

## 1. Gear hunting — three attempts, two failures

Attempt 1 (failed): speed hysteresis (+4 up / -8 down). Still flicked 4↔5 at 190 boundary.
Attempt 2 (failed half): RPM up 9500 + speed fallback. Flick moved to 3↔4 @85 and 4↔5 @115 —
RPM said up, speed said down next step. Loop.
Attempt 3 (stuck with it): RPM up 9000, RPM down 3800–4000, plus speed downshift.
Final form: up purely on revs, down on revs OR `GEAR_SPEEDS[g-1]-15` with `rpm<6500` guard.
Result: 1→6 once each on straight, holds 6th. Sliders live again (hardcoded mins made them dead).
Lesson: up and down must never disagree on consecutive steps. 15 km/h guard + rev guard does that.

Failure learned: 9500 redline bounced off limiter in 3rd and capped us at 104. 9000 fixed it.
Later failure: pure RPM stranded us in 6th into hairpins (6th @90 still ~4000rpm, `<3500` never fired).
Fix: per-gear minimums, then slider-driven minimums.

## 2. Drove off, sat at 0 revving in 1st

Causes stacked:
- Centering 0.25 gives -0.25 at edge, -0.5 at `trackPos` 2.0. Nothing.
- Fix: 1.5x pull past 0.7, 2x +0.5 past 1.0.
- `-1` sensors → full brake → `accel*(1-brake)` → crawl to stop. Fix: ignore `<=0` readings, light 0.15 off-track only.
- Angle brake 0.5 + TC cut stayed on at 0 speed. Fix: no brake under 20, no TC cut under 10, launch override
  (`speed<5` → brake 0, accel 1.0, gear 1).
- Throttle cap was over-aggressive first (`>1.0→60`, `>0.7→90`, `steer*30`). Held us at ~80. Reverted:
  only `>1.0→80`, penalty `abs(steer)*10`.

## 3. Regression to ~80 (my fault)

Capped + penalized too hard. Reverted as above. Lesson: 0.7 is normal mid-corner, don't cap there.

## 4. Adaptive target — why fixed speed can't work here

Fixed 120/150 + constant braking = top 84, crawl 15 in corners.
- v1: `50+dist*0.8`, min 55, min-of-3 beams → top 104 (65m clearance → 102).
- v2: `60+dist*1.0`, min 65, dead-ahead beam → ~125. Hill clipping one side beam had falsely capped us.
- v3 (current): wide fan `track[6:13]` ±2.5°, hairpin tier sub-25m `52+dist*0.6` min 55,
  else `60+dist*1.0` min 60. Center-only missed the final side hairpin until 24m.
Progression: 84 → 104 → 121 → 161 top.

## 5. Steering

Added speed sensitivity: `steer / (1+speed*0.008)`. Same input at 150 is ~half of 60.
Fixed last-turn exit flick. Tradeoff: slightly lazy turn-in at speed — accepted for stability.

## 6. Brakes — the 15 crawl and the 381 hit

- 15 crawl bug: braked whenever gap<45 even when already slow. Fix: if `speed < target-5`, no brake.
  Slow+brake rows went to ~2.
- Scaling: light touch when slightly over, hard when way over. `0.6→0.85` max, ramp `/40→/25`,
  +0.3 bonus past 40 over. Telemetry showed hairpin entry 120 @24m gap with brake 0.28 — caused
  the 381-damage overshoot. Now ~0.6 there.
- Distance `30→45→55`, fan widened to `[4,5,6,7,11,12,13,14]` for earlier side-hairpin detection.

## 7. Stuck / reverse — longest loop

- Threshold `100→60`, steps `50→80→35`. 80 was too long, 35 + cooldown is right.
- Accel `0.5→1.0→0.6→0.4` (19,636rpm scream at -38 showed 0.6 too much). Brake past -15 in reverse.
- Added off-track slow-stop trigger (`|tp|>1.2` + `<5` → reverse) then capped it (endless loop).
- Added backwards-stop: `speed<-2` → brake 0.8 in 1st, never keep reversing.
- Added 150-step no-reverse cooldown after any reverse. Killed reverse-to-finish (-101 min).
  Worst reverse now brief: off at -2.5, -38 for 2 rows, forward in ~5 rows. Still need zero.

## 8. Debug crash

`fancyout()` assumed all keys present. Added `--missing--` guard + safe `rpm/gear/skid/slip` access.
Cosmetic, no speed effect.

## 9. Traction control

Disabled cut under 10 km/h (launch/escape needs spin). Tested off fully — off-track went 1.2%→4%,
slower. Left on.

## 10. Results (clean unless noted)

- Start: ~2:20.7x, top ~84, crawl 15s, wall pins at 0.
- Smoothed: 2:20.44.
- Breakthrough: 2:06.83, top 121, min 50, 0 damage (short gears + RPM 9000).
- Invalid fast: 2:02.55, top 161, min -101, damage 381 (reverse across line, hairpin too hot).
- Clean: 2:07.98, top 161, min -40, 0 damage (2-row reverse only).
- Now: 2:04.34, corners 50–60, top 161. Braking early enough to survive, late enough to carry.

Telemetry (`telemetry.csv`, every 25 steps, absolute path): step,speedX,target,centerDist,brake,accel,gear,rpm,trackPos,angle.
Proved: 104 was gear+sensor cap, 120@24m/0.28 caused the big hit, 6th-strand caused the cave.

## 11. Current behavior and known debts

- Corners 50–60, brake-then-turn (grip, robotic, fast). Drift (TC off, throttle-on sideways) is slower here.
- Final sharp still the limiter. Wide fan + 55m braking just went in, unproven over multiple laps.
- Reverse still triggers ~once/lap (grass, no wall). Min negative must go positive for a truly clean optimal lap.
- Gear sliders live again (downshift path). Up stays RPM.

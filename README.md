# AI Racing League - Final Submission

## Author and members
Individual project. Torcs sensor driver developed with IBM Bob assistance, human-tested.

## Problem statement
Base TORCS driver crashed or crawled on Corkscrew. Built sensor-only reactive driver that brakes late, holds racing line, and finishes clean. Best: 1:36.10, damage 0, standing-start 1 lap.

## Environment and setup
- OS: Windows
- Python: 3.14.6 (also ran 3.11.8, no dependencies for driver)
- TORCS: IBM challenge bundle, `wtorcs.exe`, Road/Corkscrew, driver `scr_server 1` only, Laps 1
- Required local files: `torcs_jm_par.py` only. No pip install.
- Code SHA (short): 1fc2e424f6ff

## Run
From `gym_torcs` folder, start TORCS first to blue waiting screen (Quick Race > Configure > Corkscrew > scr_server 1 > 1 lap > New Race). Then in second terminal:
```
py torcs_jm_par.py
```
Stop with Ctrl+C. Client must start after simulator. Race mode uses defaults in file: TARGET_SPEED 300, STEER_GAIN 22, CENTERING_GAIN 0.20, BRAKE_THRESHOLD 0.40. Practice-mode sliders via `--parameters` JSON stay live.

## Results
Track Corkscrew, scr_server 1, 1 lap standing start, damage 0 unless noted:
- Baseline ~2:20 / damage mixed
- 2:04.34 top 161 clean (Day 1)
- ~1:50s clean after planner kill (Day 2)
- 1:38:34 clean (Day 3)
- 1:37.94 / 1:36:13 / 1:36:10 clean PB (Day 4, commits 9ce53b1/030a30c/aae4378)
See `experiment_results.csv` and `evidence/day1_report.md`, `evidence/day2_report.md`, `evidence/telemetry.csv` (step log every 10 steps, not lap_data.csv schema).

## Enhancements
1. Centre-median + asymmetric side-fan target with `v(d)=sqrt(vc^2+2*11*d)` braking profile - fixed straights capped at ~122, now 300 on straights. Evidence: Day 3 PB 1:38:34.
2. Yaw-tier + over-target brakes with trail-brake taper (`x max(0.5,1-|steer|*0.4)`) - full brake + full steer no longer requested together. Evidence: 1:37.94 -> 1:36:13.
3. Looser traction control (slip 3.5/range 6.0/40% floor, active at launch) + 0.65 low-speed cap + RPM 12000 gears with anti-hunt - stops exit wheelspin hold. Evidence: 1:36:13 -> 1:36:10.
Full history in `CHANGELOG.md`.

## Use of IBM Bob
Bob proposed batches (fan split, hysteresis, taper values); I reviewed diffs, corrected gates (e.g. asym>15, |trackPos|<0.5), reverted EMA 0.92->0.75 and decel 11->13 without laps, kept only timed gains. All laps verified in TORCS.

## Demonstration
https://drive.google.com/file/d/1AYkXQDLTlXWDD553WnwIZWVjVwA16zed/view?usp=sharing (2:00 Corkscrew run + implementations voiceover)

## Limitations
- No `lap_data.csv`/`lap_summary.csv` in official schema; evidence is `telemetry.csv` + reports + commit history.
- Variance ~0.55s run-to-run noted; PB is single best, not averaged.
- Top speed ~195-202 is 6th-gear power ceiling, not fixable in code.
- Calibration-table branch tried offline, not merged (EMA pollution suspect).

## Credits and licensing
Based on gym-torcs / snakeoil SCR client. Challenge kit files belong to organisers. My edits in `torcs_jm_par.py` only.

# CHANGELOG.md

- R01 baseline ~2:20 - why: start point - obs: crawl 15s top 84 - kept as ref
- R02 2:04.34 - RPM up/down + adaptive target v3 + speed steer - predicted faster straights - actual 2:04.34 dmg 0 top 161 - kept
- R03 ~1:50s - deleted TrackMemory planner (garbage playback + S-sign flip) - predicted stable laps 2+ - actual ~1:50s clean - kept
- R04 1:38:34 - centre/side split asym>15 + yaw-first + 0.40 soft tiers + 0.65 cap - predicted straights 300 S-turn 40-50 - actual 1:38:34 dmg 0 - kept
- R05 1:37.94 - median fan + d_center<150 + |tp|<0.5 gate + decel 8->11 - predicted later braking - actual 1:37.94 - kept
- R06 1:36:13 - trail-brake taper x max(0.5 1-|steer|*0.4) - predicted keep front grip - actual 1:36:13 - kept
- R07 1:36:10 FINAL - TC 2.0->3.5 / 5.0->6.0 / 25%->40% - predicted less exit hold - actual 1:36:10 dmg 0 - kept
- Reverted without lap: EMA 0.92->0.75, decel 11->13 - single-variable discipline.

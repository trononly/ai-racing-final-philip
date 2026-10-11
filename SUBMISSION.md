# SUBMISSION.md

Name: Philip Machinga
Project Members: individual
GitHub Link: https://github.com/trononly/ai-racing-final-philip
Final snapshot: https://github.com/trononly/ai-racing-final-philip/commit/6f1d429ee288342701df123c6c96ee78a9264085
Video Demonstration: https://drive.google.com/file/d/1AYkXQDLTlXWDD553WnwIZWVjVwA16zed/view?usp=sharing

## Problem statement

Base TORCS car on Corkscrew either crashed (damage 381) or crawled (~2:20 at 84 top). Built sensor-only driver in `torcs_jm_par.py` that finishes clean and fast: 1:36.10 damage 0.

## Enhancements

1. Look-ahead target from centre-median + asymmetric side fan with physics braking profile - straights 300, corners slowed correctly.
2. Yaw-first brakes + over-target braking with trail-brake taper for steering authority.
3. Launch/traction/gear package: 0.65 low-speed cap, proportional TC, RPM-based gears with guards, stuck recovery.

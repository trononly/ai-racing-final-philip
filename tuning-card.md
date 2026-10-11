# tuning-card.md (for torcs_jm_par.py)

| setting | current | controls | if raise | if lower | risk |
|---|---|---|---|---|---|
| TARGET_SPEED | 300 | straight ceiling | faster straights / hotter entries | slower but safer | medium |
| STEER_GAIN | 22 | angle response | sharper turn-in / oscillation | lazy / miss apex | medium |
| CENTERING_GAIN | 0.20 | line pull | tighter to line / twitch | drift wide | low |
| BRAKE_THRESHOLD | 0.40 | yaw-brake onset | later brake / spin risk | early brake / slow | high |
| GEAR_SPEEDS | 0/50/80/110/150/185 | downshift points | lug / hunt | rev limit | medium |
| TC slip 3.5/6.0/40% | on | wheelspin cut | more spin / faster exit | bog down | medium |

# `cal` command reference — v5h C5 main board

Send as JSON to the C5, keys freely combinable, omitted keys unchanged:

```powershell
Invoke-RestMethod -Uri http://192.168.1.140/command -Method Post -Body '{"command":"cal","recoil_tq":0.8,"recoil_vel":10}'
```

(or the same JSON over WebSocket :81, or browser F12 console via `fetch`).

**All changes are RAM-only** — instant effect, reverted by reboot. Bake keepers
into `MotorControl.cpp` defaults. Out-of-range values are silently ignored
(each setter clamps; ranges below). Units: revolutions / rev/s / N·m; on this
drivetrain **1 N·m ≈ 5 lbf at the handle** and a full stroke ≈ 5 revs.

## Strength feel
| key | default | range | meaning |
|---|---|---|---|
| `force_scale` | 0.40 | (0..1] | screen-lb → torque anchor, MIDPOINT-anchored: = 0.32 (pull-measured) × (1+K_TRUE 0.25). Holds/slow motion read the set weight; independent of `fric_k` — do NOT re-anchor when tuning it |
| `fric_k` | 0.17 | [0..0.5] | directional friction comp: boosts lowering / trims pulling by ±k (≈70% of the measured 0.25 capstan loss — never exceed measured; overcomp = negative damping) |
| `fric_band` | 0.6 | rev/s | pull speed at which the comp blend saturates |
| `zero_m1` | 0.1 | [0..1] | keep-taut floor torque (N·m) at ~0 target force |
| `recoil_tq` | 0.8 | [0..2] | 0 lb self-stow pull, N·m (~4 lbf) |
| `recoil_vel` | 8 | (0..50] | 0 lb stow speed, rev/s (slows to ~2 near the dock automatically) |
| `zone` | 1.0 | (0..10] | dock zone (revs): force tapers + speed drops inside it |
| `sleep_s` | 10 | (0..3600] | parked-and-still seconds before the axis sleeps |
| `wake_d` | 0.25 | (0..1] | revs of strap movement that wake from sleep |
| `soft_max` | 0.5 | [0..2] | legacy ODrive push-out bound — **inert on moteus**, kept for compat |
| `con_pct` | 0 | [0..100] | concentric-only unloading: % of force removed when not actively pulling out (also settable from the strength screen's CON label) |
| `con_lo` / `con_hi` | 0.3 / 1.5 | rev/s | pull-velocity band over which force blends back in |

## Weight-update gate (pull-to-confirm, revs of travel)
| key | default | meaning |
|---|---|---|
| `delta_x` | 1.1 | ramp length: force climbs old→new over this much pull |
| `delta_y` | 0.3 | extra hold past the ramp before the new weight latches |
| `deadband` | 0.15 | pull needed to start the ramp after arming |
| `cancel` | 0.4 | backtrack that cancels a pending increase |
| `pull_sign` | +1 | ±1: flip if position decreases when pulling out |

## Drop-catch (anti-runaway)
| key | default | meaning |
|---|---|---|
| `drop_vmin` | 6 | reel-in rev/s where force shedding starts |
| `drop_vmax` | 15 | reel-in rev/s where only the floor remains (> vmin) |
| `drop_restore` | 1.5 | revs of pull-out to fully restore shed force |

## Row model
| key | default | range | meaning |
|---|---|---|---|
| `row_kc` | 2.5 | (0..100] | catch stiffness: N·m per rev/s of overspeed at gear 5 |
| `row_inertia` | 5 | (0..100] | virtual wheel inertia — bigger = longer glide, steadier drive force |
| `row_drag_base` | 0.15 | (0..10] | decay coefficient the 1-10 screen damper scales |
| `row_return` | 0.6 | [0..2] | recovery tension N·m — also the strap's chase muscle (they are the same thing in this scheme) |
| `row_max` | 10 | (0..100] | ceiling on rendered row force |
| `row_zone` | 0.5 | (0..10] | pinch-only dock taper (revs) in row mode |

(Gear and drag are user settings — tap the numbers on the row screen.)

## Torque matrix (advanced)
`{"row":N,"values":[force_N, vel_delta, -Fn, -Fm, +Fn, +Fm]}` — hot-edit row
N (0-5) of the force→torque lookup. Row 0 is the zero/keep-taut row.

## Utility / probes
| key | default | meaning |
|---|---|---|
| `test_torque` | — | stream a constant reel-in torque (N·m, clamp ±3) for 10 s — the felt-force calibration probe. 0 cancels. Needs strength mode active. |
| `debug` | true | `[wg]`/`[row]` 4 Hz serial telemetry on/off |
| `can_quiet` | false | mute ALL CAN TX (mandatory during moteus calibration/tview driving) |
| `brs` | true | CAN-FD 5 Mbps data phase; false = whole bus at 1 Mbps (signal-integrity fallback) |
| `reel_in_vel` | 20 | chase velocity setpoint magnitude, rev/s (rarely touched) |
| `vel_limit` | 25 | away-from-dock velocity ceiling, rev/s (rarely touched) |

## Recipes
```json
{"command":"cal","test_torque":1.0}                       // feel exactly 1 Nm for 10 s (luggage-scale it)
{"command":"cal","force_scale":0.38}                      // trim screen-lb accuracy
{"command":"cal","recoil_tq":0.8,"recoil_vel":10}         // stronger, faster 0 lb stow
{"command":"cal","row_return":1.5,"row_kc":3.0}           // firmer row recovery + catch
{"command":"cal","delta_x":1.5,"deadband":0.2}            // longer pull-to-confirm ramp
{"command":"cal","can_quiet":true}                        // silence C5 for bench work
```

The response echoes `force_scale`, `zero_m1`, `brs`, `can_quiet` as a landing
confirmation.

---

# Full command vocabulary (beyond `cal`)

Same transport for all: HTTP `POST /command` or WebSocket :81.

| command | body | effect |
|---|---|---|
| `strength` | `{"command":"strength","weight_lbs":10}` (or `weight_kg`) | set a weight — identical path to the screen numpad, pull-to-confirm gate applies |
| `idle` | `{"command":"idle"}` | weight 0 / off (recoil-stow, then sleep) — also exits row mode |
| `row` | `{"command":"row","type":"on","gear":5,"drag":5}` (`"off"` to leave) | row mode from the app side, same as entering/leaving the row screen |
| `cal` | see tables above | live tuning, incl. `can_quiet` (CAN mute) and `brs` |
| `pulse` | `{"command":"pulse","type":"on","duration":0,"strength":50,"frequency":8}` | sine force modulation on top of the set weight (haptic texture) |
| `force` | `{"command":"force","type":"constant","strength":100,...}` | force profile: `off` / `constant` (percent of set weight) / `linear` (ramps with position) |
| `detent` | accepted | **currently inert** — the detent stage is commented out of the force pipeline |
| `mode` | `{"command":"mode","type":"strength"}` | bookkeeping label only (status/metrics reporting) |

Read-only endpoints:
- `GET /metrics` — full JSON: position, velocity, force, voltage, current, IMU, `row_watts`, `row_spm`, latency timestamps
- `GET /health` — `{"status":"OK","uptime":...}`
- WebSocket :81 also **pushes** a metrics frame every ~400 ms — subscribe and you get live telemetry with no polling.

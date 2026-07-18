# Digital Weight — device web API (for the app's device-utility page)

The C5 main board runs two servers on the local network:

| | |
|---|---|
| HTTP | `http://<device-ip>/` (port 80) — commands + one-shot reads |
| WebSocket | `ws://<device-ip>:81` — same commands + pushed telemetry every ~400 ms |
| mDNS name | `digital-weight.local` (unreliable from browsers — prefer the IP, shown in the `[hb]` serial line and your router's DHCP table) |

CORS is wide open on the device (`Access-Control-Allow-Origin: *`, OPTIONS
preflight handled), so any page — including localhost dev servers — can call
it directly from browser JS.

## ⚠️ The HTTPS gotcha (read first)

The device speaks plain `http://` and `ws://` — it has no TLS. Browsers
**block** those from pages served over `https://` (mixed content). The
production app on GitHub Pages is HTTPS, so the device utility will NOT work
there as-is. Options, best first:

1. **Run the utility from a localhost dev server** (`http://localhost:5173`
   etc.) — http page → http device = allowed, and this matches when you'd
   actually use it (same LAN as the machine).
2. Serve/open the utility page itself over plain http or `file://`.
3. (Later, if it must live in the prod app: a tiny local bridge or allowing
   the browser's per-site insecure-content exception — both clunky; prefer 1.)

## Commands (HTTP POST `/command`, body = JSON; or send the same JSON as a WS text frame)

| command | body example | effect |
|---|---|---|
| `strength` | `{"command":"strength","weight_lbs":10}` (or `weight_kg`, converted) | set weight — the physical pull-to-confirm gate still applies. **The device's canonical weight unit is POUNDS**: `active_weight`/`pending_weight` in /metrics and the WS broadcast are lb, and the HTTP ack echoes `weight_lbs`. Do NOT convert reads from kg. |
| `idle` | `{"command":"idle"}` | weight 0 / off (also exits row mode) — make this the big red button |
| `row` | `{"command":"row","type":"on","gear":5,"drag":5}` / `"off"` | enter/leave row mode |
| `cal` | `{"command":"cal","recoil_tq":0.8,"can_quiet":true}` | live tuning — full key list in CAL_REFERENCE.md. RAM-only; reboot reverts |
| `cal` (concentric) | `{"command":"cal","con_pct":100}` | concentric-only unloading, 0-100: % of force removed when not actively pulling out (0 = normal both directions; keep-taut/recoil floor always remains). `con_lo`/`con_hi` (rev/s) tune where "pulling" begins/ends. Same control the strength screen's % label sets. |
| `pulse` | see **Force profiles** below | haptic sine ripple on the set weight |
| `force` | see **Force profiles** below | force-vs-position profile (`off`/`constant`/`linear`) |
| `detent` | see **Force profiles** below | tactile notches along the stroke |
| `mode` | `{"command":"mode","type":"strength"}` | status label bookkeeping only |

## Force profiles

All position-based profiles use **revs of pull from the dock** (a full
stroke on this machine is ~5 revs) and multiply the SET weight — the safety
stack (gate, drop-catch, concentric, dock taper) applies after, unchanged.
Profiles persist until changed (RAM; reboot reverts to `constant`/100%).

```json
// constant (the default) — 100% of the set weight everywhere
{"command":"force","type":"constant","strength":100}

// CHAINS — starts at 40% off the floor, full weight by 4 revs of pull
{"command":"force","type":"linear","start_strength":40,"strength":100,
 "start_position":0.3,"saturation_position":4.0}

// RESISTANCE BAND — light at the bottom, overloads past 100% at the top
{"command":"force","type":"linear","start_strength":15,"strength":140,
 "start_position":0.2,"saturation_position":4.5}

// DETENTS — tactile clicks: force dips strength% at every step_position
// revs, starting start_position revs from the dock, for total_steps notches
{"command":"detent","type":"on","strength":40,"start_position":0.5,
 "step_position":0.8,"total_steps":5}
{"command":"detent","type":"off"}

// PULSE — sine ripple, strength% of a 0.4x-weight envelope at frequency Hz
{"command":"pulse","type":"on","strength":50,"frequency":8,"duration":0}
{"command":"pulse","type":"off"}
```

Notes: `strength` above 100 is allowed on `linear` (that's the band
overload). Below `start_position` the linear profile holds
`start_strength`%, not zero. Profiles compose in order: force profile →
detents → pulse.

HTTP responses are small JSON (`{"result":"strength","weight_kg":4.5,...}`).
The `cal` response echoes `force_scale`, `zero_m1`, `brs`, `can_quiet`.
NOTE: there is no read-back for most cal values — a tuning UI should treat
"what I last sent" as its state, seeded from the defaults table in
CAL_REFERENCE.md.

## Reads

- `GET /metrics` — `{position, velocity, force, voltage, current,
  accelerometer_*, gyro_*, virtual_velocity, row_watts, row_spm,
  active_weight, pending_weight, weight_pending, status, t_*_ms latency
  stamps}` — `weight_pending=1` means a commanded weight awaits the user's
  confirming pull; show "N lb pending" until it clears.
- `GET /health` — `{"status":"OK","uptime":<ms>}`

## WebSocket telemetry & RTT

Connect to `ws://<ip>:81`. You receive a metrics frame (`{"type":"metrics",
...}` — same fields as `/metrics`) every ~400 ms without asking. Any command
you send gets an ack frame; include `"client_ts": Date.now()` in a command
and the ack echoes it — subtract for live round-trip latency, handy for a
connection-quality indicator. (Broadcasts pause ~250 ms after a command to
keep the command path snappy.)

## Ready-to-paste JS

```js
// ---- config ----
const DEVICE = localStorage.getItem('dw_ip') || '192.168.1.140';

// ---- one-shot command over HTTP ----
async function dwCommand(body) {
  const r = await fetch(`http://${DEVICE}/command`, {
    method: 'POST',
    body: JSON.stringify(body),
  });
  return r.json();
}
// examples:
// dwCommand({command:'strength', weight_lbs: 10})
// dwCommand({command:'idle'})
// dwCommand({command:'cal', can_quiet: true})     // mute CAN (bench mode)
// dwCommand({command:'cal', recoil_tq: 0.8, recoil_vel: 10})

// ---- live telemetry over WebSocket, with auto-reconnect ----
function dwConnect(onMetrics, onStatus = () => {}) {
  let ws, alive = false;
  const open = () => {
    ws = new WebSocket(`ws://${DEVICE}:81`);
    ws.onopen = () => { alive = true; onStatus('connected'); };
    ws.onclose = () => { alive = false; onStatus('disconnected'); setTimeout(open, 2000); };
    ws.onmessage = (e) => {
      const m = JSON.parse(e.data);
      if (m.type === 'metrics') onMetrics(m);
      else if (m.client_ts) onStatus(`rtt ${Date.now() - m.client_ts} ms`);
    };
  };
  open();
  return {
    send: (body) => alive && ws.send(JSON.stringify({ ...body, client_ts: Date.now() })),
    isAlive: () => alive,
  };
}

// usage:
// const dw = dwConnect(m => {
//   ui.voltage.textContent = m.voltage.toFixed(1) + 'V';
//   ui.force.textContent   = m.force.toFixed(0) + 'N';
//   ui.watts.textContent   = m.row_watts.toFixed(0);
//   chart.push(m.position, m.velocity);
// });
// dw.send({command:'strength', weight_lbs: 15});
```

## Utility-page design notes

- **Zero-weight button**: prominent, one tap, sends `{command:'idle'}`. It is
  the software e-stop (force physically releases; strap self-stows).
- **CAN mute toggle** (`cal can_quiet`): label it clearly — while muted the
  machine is fully inert (screen commands queue but nothing moves) and the
  screen's voltage goes stale. It exists for moteus-calibration bench
  sessions. State is echoed in every `cal` response.
- **Cal sliders**: send on release (not on drag) — commands coalesce on the
  device but there's no need to spam. Group per CAL_REFERENCE.md sections.
  Add a "copy JSON" button that exports the session's accumulated cal deltas
  — that's exactly what gets baked into firmware defaults afterwards.
- **Persistence**: cal is RAM-only. A reboot silently reverts — consider
  showing uptime from `/health` and warning when it resets (uptime went
  backwards = device rebooted = your tuning session is gone from the device).
- Weight changes still require the user to pull into them (safety gate) —
  reflect it: while `weight_pending=1`, show `pending_weight` as "awaiting
  pull-to-confirm" next to `active_weight` (both in /metrics and the WS
  broadcast).
- **Concentric % has two writers and no read-back**: the strength screen's
  % label and the app's `con_pct` both set the same device value; last
  writer wins, and the screen label does NOT update when the app changes it
  (it shows the screen's own last-set value). Treat the app's slider the
  same way — it reflects what the app last sent. Applies instantly, no
  confirmation pull needed (it only ever unloads).

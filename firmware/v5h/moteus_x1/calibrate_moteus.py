#!/usr/bin/env python
"""
moteus-x1 calibration + configuration for the digital weight machine.

Motor: 2x CubeMars RI115-PH KV40, windings wired in parallel and rotors
mechanically locked on one shaft. To the moteus this looks like ONE motor:
  - Kv 40 (same shaft -> same back-EMF as a single motor)
  - 20 pole pairs / 40 poles (calibration discovers this; we verify it)
  - HALF the impedance of one motor: ~67 mOhm / ~122 uH phase-to-phase
  - doubled current headroom: combined ~11 Nm rated / ~32 Nm peak, but the
    x1 itself is the steady-state bound (~25 A continuous without cooling)
Pole pairs are NOT entered by hand anywhere — moteus maps electrical to
mechanical position itself during --calibrate. If the pair's phases were
misaligned it would still "calibrate", but the motor would run weak and
hot from circulating current — the poles/resistance checks below are the
tripwire for that.

Usage (fdcanusb attached, moteus powered from its DC bus):
  python calibrate_moteus.py               # full run: calibrate + config + dump
  python calibrate_moteus.py --skip-cal    # re-apply config/limits only
  python calibrate_moteus.py --spin-test   # add a gentle supervised spin at the end

SAFETY: calibration and the spin test rotate the motor. The shaft must be
COMPLETELY FREE — strap/spool disconnected or free to turn with nothing
attached. The script stops and asks before anything moves.

BUS DISCIPLINE while this script runs: close tview (it holds the fdcanusb
port), and MUTE the C5 main board — it is an active commander that wakes on
any shaft movement (calibration spins the shaft a full turn) and would fight
moteus_tool mid-measurement. The C5 is hardwired, so mute it over WiFi:
    send {"command":"cal","can_quiet":true}  to ws://digital-weight.local:81
and unmute with can_quiet:false (or reboot the C5) afterwards. The script
sniffs the bus before calibrating and warns if anything is still talking.
If the diagnostic channel is jammed by leftover telemetry from a killed
tview session, power-cycle the moteus (that state is volatile).

The fdcanusb COM port is exclusive: this script opens it for its own
queries and RELEASES it while moteus_tool runs as a subprocess, so the two
never fight over the port.

Requires: pip install moteus  (>= 1.0, Python 3.13 install — use `python`,
not the Microsoft Store `python3`).
"""

import argparse
import asyncio
import datetime
import math
import subprocess
import sys
import time

import moteus

# ------------------------------ Parameters -----------------------------------
TARGET_ID = 1

# Calibration behavior
CAL_FORCE_KV = 40.0    # nameplate Kv: moteus then only spins +-360 deg (gentle)
CAL_MOTOR_POWER = 15.0 # W during encoder mapping; default 7.5 is marginal for
                       # a locked pair — raise further if rotation is ragged

# Electrical expectations (parallel pair) for post-cal sanity checks
EXPECT_POLES = 40            # 20 pole pairs
EXPECT_RESISTANCE_OHM = 0.033  # ~phase (wye-equivalent) for 67 mOhm line-line
RESISTANCE_TOLERANCE = 0.6   # +-60%: lead/connector resistance adds up

# Machine limits written to the controller (the ABSOLUTE force/speed ceiling —
# the C5 firmware modulates torque beneath these, it can never exceed them).
# Kt of the pair is ~0.24-0.28 Nm per TOTAL amp, so 40 A ~= 10-11 Nm.
MAX_CURRENT_A = 40.0   # combined rated current of the pair; x1 handles peaks
MAX_POWER_W = 900.0    # electrical power bound
MAX_VELOCITY = 40.0    # rev/s guard; the C5 never commands above 25
SUPPLY_VOLTAGE = 48.0  # your DC bus. max_voltage sits above it so eccentric
                       # regen (user lowering = energy INTO the bus) engages
                       # flux braking instead of overvoltage-faulting a PSU
                       # that can't absorb it.

# Failsafe: if the C5's 500 Hz command stream dies, the moteus drops to
# timeout mode after default_timeout_s. Mode 0 (stopped) = freewheel = the
# strap goes slack — the safe failure for a weight machine.
DEFAULT_TIMEOUT_S = 0.3
TIMEOUT_MODE = 0  # stopped

CONFIG = [
    ("id.id", str(TARGET_ID)),
    ("servo.max_current_A", f"{MAX_CURRENT_A}"),
    ("servo.max_power_W", f"{MAX_POWER_W}"),
    ("servo.max_velocity", f"{MAX_VELOCITY}"),
    ("servo.max_voltage", f"{SUPPLY_VOLTAGE + 6.0}"),
    ("servo.default_timeout_s", f"{DEFAULT_TIMEOUT_S}"),
    ("servo.timeout_mode", str(TIMEOUT_MODE)),
    # Output direction: the C5's negative "reel-in" velocity must pull AWAY
    # from the user. Sign measured on the bench 2026-07-13 (stock +1 pulled
    # toward the user on this build). Flips position/velocity/torque
    # coherently at the controller, so C5 conventions stay untouched.
    ("motor_position.output.sign", "-1"),
    # Winch/strap: rotation is unbounded, no position stops. This must be
    # applied BEFORE calibration: finite limits + an uncalibrated encoder's
    # arbitrary position make the inductance stage fault with 39
    # (start-outside-limit). NOTE the group: position limits live under
    # `servopos`, not `servo`.
    ("servopos.position_min", "nan"),
    ("servopos.position_max", "nan"),
    # No motor thermistor is wired (the RI115-PH's integrated sensor is
    # unused). Thermal protection is the x1's own FET temperature derating,
    # which bounds sustained current below the pair's 40 A combined rating
    # anyway — MAX_CURRENT_A above is the winding protection, keep it honest.
    ("servo.enable_motor_temperature", "0"),
]


class Link:
    """Owns the fdcanusb port. Only one owner at a time: close() before any
    moteus_tool subprocess, reopen after."""

    def __init__(self):
        self.transport = moteus.Fdcanusb()
        self.controller = moteus.Controller(id=TARGET_ID, transport=self.transport)
        self.stream = moteus.Stream(self.controller)

    def close(self):
        self.transport.close()


VERBOSE = False


def run_moteus_tool(args_list, check=True):
    cmd = [sys.executable, "-m", "moteus.moteus_tool", "--target", str(TARGET_ID)] + args_list
    if VERBOSE:
        cmd.append("--verbose")
    print(f"\n>>> {' '.join(cmd)}")
    return subprocess.run(cmd, check=check)


def confirm(prompt):
    answer = input(f"{prompt} [y/N] ").strip().lower()
    if answer != "y":
        print("Aborted.")
        sys.exit(1)


# Diagnostic-channel commands await an "OK" from the moteus; if that channel
# is jammed (leftover tview telemetry, another host on the bus) the await
# hangs forever. Every command here gets a timeout and one flush-and-retry.
async def stream_command(stream, data, timeout=3.0):
    for attempt in range(2):
        try:
            return await asyncio.wait_for(stream.command(data), timeout)
        except asyncio.TimeoutError:
            if attempt == 0:
                print(f"  no response to {data!r} — flushing channel and retrying...")
                try:
                    await asyncio.wait_for(stream.flush_read(), 1.0)
                except Exception:
                    pass
    return None


def diag_channel_dead():
    print("\nThe moteus is not answering on the diagnostic channel.")
    print("In order of likelihood:")
    print("  1. tview/another session left telemetry streaming — power-cycle the moteus")
    print("  2. the C5 main board is powered and commanding — power it off for calibration")
    print("  3. tview is still open and holding the fdcanusb — close it")
    sys.exit(1)


async def conf_set(stream, name, value):
    try:
        result = await stream_command(stream, f"conf set {name} {value}".encode("latin1"))
    except Exception as e:
        # The moteus answered with an error (bad name/value for this firmware
        # version) — warn and continue rather than aborting the whole run.
        print(f"  WARNING: conf set {name} {value} rejected ({e})")
        if "position_min" in name or "position_max" in name:
            print("           position limits MUST be cleared before calibration")
            print("           (fault 39) — fix this one in tview before proceeding")
        return False
    if result is None:
        diag_channel_dead()
    print(f"  conf set {name} {value}")
    return True


async def conf_get(stream, name):
    result = await stream_command(stream, f"conf get {name}".encode("latin1"))
    return result.decode("latin1").strip() if result is not None else None


async def main():
    parser = argparse.ArgumentParser(description="Calibrate/configure the moteus-x1")
    parser.add_argument("--skip-cal", action="store_true",
                        help="skip motor calibration, only apply config + dump")
    parser.add_argument("--spin-test", action="store_true",
                        help="finish with a gentle 2 rev/s spin for 3 s")
    parser.add_argument("--verbose", action="store_true",
                        help="pass --verbose to moteus_tool (logs every CAN exchange)")
    args = parser.parse_args()
    global VERBOSE
    VERBOSE = args.verbose

    link = Link()

    # ---- Preflight: prove the link and show the board state ----
    print("Checking controller link...")
    state = await link.controller.query()
    print(f"  mode={int(state.values[moteus.Register.MODE])} "
          f"bus={state.values[moteus.Register.VOLTAGE]:.1f}V "
          f"temp={state.values[moteus.Register.TEMPERATURE]:.0f}C "
          f"fault={int(state.values[moteus.Register.FAULT])}")
    print("Stopping axis (d stop)...")
    if await stream_command(link.stream, b"d stop") is None:
        diag_channel_dead()

    # ---- Machine configuration FIRST: clearing the position limits is a
    # calibration prerequisite (fault 39), and none of these depend on
    # calibration results. Persist immediately so a retry starts clean. ----
    print("\nApplying machine configuration...")
    for name, value in CONFIG:
        await conf_set(link.stream, name, value)
    print("Persisting to flash (conf write)...")
    if await stream_command(link.stream, b"conf write", timeout=6.0) is None:
        diag_channel_dead()

    # ---- Calibration (moteus_tool subprocess needs the port to itself) ----
    if not args.skip_cal:
        # Bus-quiet sniff: we are silent and the moteus never speaks
        # unsolicited, so ANY frame in this window is another commander —
        # in this machine, the C5. Its traffic breaks calibration (it wakes
        # on shaft movement and streams position commands over the test).
        # Flush first: our own config exchanges above leave straggler reply
        # frames queued in the transport, which would read back instantly
        # as phantom "traffic".
        print("\nSniffing the bus for other hosts (1.5 s)...")
        try:
            await asyncio.wait_for(link.transport.flush_read_queue(), 1.0)
        except Exception:
            pass
        try:
            frame = await asyncio.wait_for(link.transport.read(), 1.5)
            fid = getattr(frame, "arbitration_id", None)
            fid_text = f" (CAN id {fid:#x})" if isinstance(fid, int) else ""
            print(f"  TRAFFIC DETECTED{fid_text} — another host is on the bus.")
            print("  id 0x84xx/0x01xx = the C5 (host 4); a stream of frames = tview")
            print("  telemetry left running (power-cycle the moteus). To mute the C5:")
            print('    {"command":"cal","can_quiet":true}   -> ws://digital-weight.local:81')
            confirm("Continue anyway (NOT recommended)?")
        except asyncio.TimeoutError:
            print("  bus quiet — good.")

        print("\nCALIBRATION will spin the motor up to a full turn each way.")
        confirm("Is the shaft COMPLETELY free (no strap, no spool load)?")

        link.close()  # release COM port for the subprocess
        time.sleep(0.5)
        run_moteus_tool([
            "--calibrate",
            "--cal-force-kv", str(CAL_FORCE_KV),
            "--cal-motor-power", str(CAL_MOTOR_POWER),
        ])
        time.sleep(0.5)
        link = Link()  # take the port back

        # Sanity-check what calibration measured. Wrong pole count or a
        # resistance far off the parallel-pair expectation usually means a
        # wiring/phasing problem, not a bad motor. NOTE: at this point the
        # calibration itself already succeeded AND persisted (moteus_tool
        # conf-writes its results), so everything below degrades to warnings
        # — a wedged reopened port must not read as a failed calibration.
        poles = await conf_get(link.stream, "motor.poles")
        if poles is None:
            # Windows COM reopen after the subprocess is flaky; one clean
            # close/reopen usually recovers it.
            print("  reopened port unresponsive — cycling it once...")
            link.close()
            time.sleep(1.0)
            link = Link()
            poles = await conf_get(link.stream, "motor.poles")
        res = await conf_get(link.stream, "motor.resistance_ohm")
        print(f"\nMeasured: motor.poles={poles} motor.resistance_ohm={res}")
        if poles is None and res is None:
            print("  (readback unavailable — check poles/resistance in the REPORT")
            print("   JSON moteus_tool printed above; calibration itself is stored)")
        try:
            if int(float(poles)) != EXPECT_POLES:
                print(f"  WARNING: expected {EXPECT_POLES} poles (20 pole pairs). "
                      "Re-check phase wiring/alignment of the pair before trusting this.")
            r = float(res)
            lo = EXPECT_RESISTANCE_OHM * (1 - RESISTANCE_TOLERANCE)
            hi = EXPECT_RESISTANCE_OHM * (1 + RESISTANCE_TOLERANCE)
            if not (lo <= r <= hi):
                print(f"  WARNING: resistance outside {lo:.3f}-{hi:.3f} ohm for the "
                      "parallel pair. One motor may not be contributing (check "
                      "connections) or leads are adding resistance.")
        except (TypeError, ValueError):
            print("  WARNING: could not parse measured values — verify in tview.")

    # ---- Optional supervised spin ----
    if args.spin_test:
        print("\nSPIN TEST: position mode, 2 rev/s, 1 Nm cap, 3 seconds.")
        confirm("Shaft still free?")
        try:
            for _ in range(300):  # 3 s at 100 Hz — same recipe the C5 streams
                await link.controller.set_position(position=math.nan, velocity=2.0,
                                                   maximum_torque=1.0, query=False)
                await asyncio.sleep(0.01)
            state = await link.controller.query()
            print(f"  spinning: velocity={state.values[moteus.Register.VELOCITY]:.2f} rev/s "
                  f"torque={state.values[moteus.Register.TORQUE]:.2f} Nm "
                  f"bus={state.values[moteus.Register.VOLTAGE]:.1f}V")
        finally:
            await link.controller.set_stop()
        print("  stopped.")

    # ---- Dump the final config next to this script, dated ----
    # (In-process `conf enumerate` — same content moteus_tool --dump-config
    # produces, without another COM-port handoff.)
    stamp = datetime.date.today().isoformat()
    dump_name = f"config_dump_{stamp}.txt"
    print(f"\nDumping config (conf enumerate) to {dump_name}...")
    dump = await stream_command(link.stream, b"conf enumerate", timeout=20.0)
    if dump is None:
        print("  WARNING: dump failed (port/channel unresponsive). Calibration and")
        print("  config are already persisted on the controller. Grab the dump")
        print("  manually after a moteus power-cycle:")
        print(f"    python -m moteus.moteus_tool --target {TARGET_ID} --dump-config > {dump_name}")
    else:
        with open(dump_name, "w") as f:
            f.write(dump.decode("latin1"))
        print(f"\nDone. Config saved to {dump_name} — commit it alongside the firmware.")
    print("Next: hand-verify the force recipe in tview (d pos nan 2 1), then the C5 takes over.")


if __name__ == "__main__":
    asyncio.run(main())

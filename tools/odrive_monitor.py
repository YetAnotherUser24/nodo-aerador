"""
odrive_monitor.py -- Live ODrive telemetry via USB

Compare directly against the ESP32 (env:esp32-s3-verify) serial output.

Usage:
    pip install odrive
    python tools/odrive_monitor.py          # human-readable table
    python tools/odrive_monitor.py --csv    # CSV mode, same columns as ESP32

CSV columns (identical to verify_odrive.cpp):
    t_s, rpm, vbus_v, ibus_a, power_w, torque_nm, iq_a

The ODrive can be connected to USB (this script) AND UART (ESP32) simultaneously.
The USB uses the Fibre protocol -- it does NOT interfere with UART ASCII traffic.
"""

import time
import sys

try:
    import odrive
except ImportError:
    print("ERROR: odrive package not installed. Run:  pip install odrive")
    sys.exit(1)

TABLE_MODE = "--table" in sys.argv
CSV_MODE   = "--csv" in sys.argv
POLL_HZ    = 2
SEP        = "-" * 68


def safe(fn, default=float("nan")):
    try:
        return fn()
    except Exception:
        return default


def fmt(v, spec=".4f"):
    return "nan" if v != v else format(v, spec)


def main():
    print("Searching for ODrive via USB... (Ctrl+C to quit)", file=sys.stderr)
    odrv = odrive.find_any(timeout=10)
    if odrv is None:
        print("ERROR: No ODrive found.", file=sys.stderr)
        sys.exit(1)

    fw = getattr(odrv, "fw_version_major", "?")
    hw = getattr(odrv, "hw_version_major", "?")
    print(f"Connected: ODrive HW v{hw} / FW v{fw}", file=sys.stderr)

    if CSV_MODE:
        print("t_s,rpm,vbus_v,ibus_a,power_w,torque_nm,iq_a")
    elif not TABLE_MODE:
        print("Format: rpm:<val> RPM, vbus:<val> V, ibus:<val> A, power:<val> W, torque:<val> Nm, iq:<val> A", file=sys.stderr)

    interval = 1.0 / POLL_HZ
    t_start  = time.time()

    try:
        while True:
            t0 = time.time()

            vel   = safe(lambda: odrv.axis0.vel_estimate)
            rpm   = vel * 60.0 if vel == vel else float("nan")
            vbus  = safe(lambda: odrv.vbus_voltage)
            ibus  = safe(lambda: odrv.ibus)
            torq  = safe(lambda: odrv.axis0.motor.torque_estimate)
            iq    = safe(lambda: odrv.axis0.motor.foc.Iq_measured)
            pw    = vbus * ibus if (vbus == vbus and ibus == ibus) else float("nan")
            temp  = safe(lambda: odrv.axis0.fet_thermistor.temperature)
            state = safe(lambda: odrv.axis0.current_state, default=-1)
            errs  = safe(lambda: odrv.axis0.active_errors,  default=-1)
            t_s   = round(time.time() - t_start, 2)

            if CSV_MODE:
                print(f"{t_s},{fmt(rpm,'.2f')},{fmt(vbus,'.3f')},{fmt(ibus,'.4f')},{fmt(pw,'.3f')},{fmt(torq,'.4f')},{fmt(iq,'.4f')}")
                sys.stdout.flush()
            elif TABLE_MODE:
                state_str = {1: "IDLE", 8: "CLOSED_LOOP"}.get(
                    int(state) if state != -1 else -1, f"STATE_{state}"
                )
                print(f"\n[{time.strftime('%H:%M:%S')}]  ODrive USB  (compare with ESP32)")
                print(SEP)
                print(f"  {'rpm  (vel_estimate x60)':<32} {fmt(rpm,'.1f'):>8}  RPM")
                print(f"  {'vbus_v':<32} {fmt(vbus,'.3f'):>8}  V")
                print(f"  {'ibus_a  (DC bus, noisy)':<32} {fmt(ibus,'.4f'):>8}  A")
                print(f"  {'power_w  (vbus x ibus)':<32} {fmt(pw,'.3f'):>8}  W")
                print(f"  {'torque_nm  (torque_estimate)':<32} {fmt(torq,'.4f'):>8}  Nm")
                print(f"  {'iq_a  (foc.Iq_measured)':<32} {fmt(iq,'.4f'):>8}  A")
                print(f"  {'fet_temp_c':<32} {fmt(temp,'.1f'):>8}  C")
                print(f"  {'state':<32} {state_str:>8}")
                print(f"  {'active_errors':<32} {errs:#010x}")
                print(SEP)
            else:
                # Format: field:value unit
                print(
                    f"rpm:{fmt(rpm,'.2f')} RPM, "
                    f"vbus:{fmt(vbus,'.3f')} V, "
                    f"ibus:{fmt(ibus,'.4f')} A, "
                    f"power:{fmt(pw,'.3f')} W, "
                    f"torque:{fmt(torq,'.4f')} Nm, "
                    f"iq:{fmt(iq,'.4f')} A"
                )
                sys.stdout.flush()

            time.sleep(max(0.0, interval - (time.time() - t0)))

    except KeyboardInterrupt:
        print("\nStopped.", file=sys.stderr)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
import argparse
import time

try:
    import Jetson.GPIO as GPIO
except ImportError as exc:
    raise SystemExit("Jetson.GPIO not available. Install it or run on a Jetson.") from exc


def main():
    parser = argparse.ArgumentParser(description="Toggle BNO085 reset pin via Jetson GPIO.")
    parser.add_argument("--pin", type=int, default=17, help="BOARD pin number (default: 17)")
    parser.add_argument("--active-low", action="store_true", default=True, help="Active-low reset (default: true)")
    parser.add_argument("--pulse-ms", type=int, default=500, help="Reset pulse width in ms (default: 500)")
    parser.add_argument("--recover-ms", type=int, default=500, help="Recovery delay in ms (default: 500)")
    args = parser.parse_args()

    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)

    inactive = GPIO.HIGH if args.active_low else GPIO.LOW
    active = GPIO.LOW if args.active_low else GPIO.HIGH

    GPIO.setup(args.pin, GPIO.OUT, initial=inactive)
    GPIO.output(args.pin, active)
    time.sleep(args.pulse_ms / 1000.0)
    GPIO.output(args.pin, inactive)
    time.sleep(args.recover_ms / 1000.0)

    GPIO.cleanup(args.pin)


if __name__ == "__main__":
    main()

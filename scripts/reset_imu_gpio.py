#!/usr/bin/env python3
import argparse
import time
import sys
import os

# Try importing Jetson.GPIO, but allow fallback to sysfs
try:
    import Jetson.GPIO as GPIO
    GPIO_AVAILABLE = True
except (ImportError, RuntimeError) as e:
    GPIO_AVAILABLE = False
    print(f"Warning: Jetson.GPIO not available ({e}), using sysfs fallback", file=sys.stderr)


# BOARD pin to sysfs GPIO mapping for Jetson Orin Nano
# Reference: https://jetsonhacks.com/nvidia-jetson-orin-nano-gpio-header-pinout/
BOARD_TO_GPIO = {
    7: 106,   # GPIO09 / PQ.06
    11: 112,  # UART1_RTS / PR.00
    12: 32,   # I2S2_CLK / PE.00
    13: 84,   # SPI1_SCK / PL.04
    15: 85,   # GPIO12 / PL.05
    16: 36,   # SPI1_CS1 / PE.04
    18: 35,   # SPI1_CS0 / PE.03
    19: 34,   # SPI1_MOSI / PE.02
    21: 33,   # SPI1_MISO / PE.01
    22: 106,  # GPIO01 / PQ.06 (duplicate - confirm your schematic)
    23: 84,   # SPI1_CLK / PL.04 (duplicate)
    24: 35,   # SPI1_CS0 / PE.03 (duplicate)
    26: 36,   # SPI1_CS1 / PE.04 (duplicate)
    29: 134,  # GPIO21 / PBB.00
    31: 136,  # GPIO11 / PBB.02
    32: 63,   # GPIO07 / PH.07
    33: 106,  # GPIO13 / PQ.06
    35: 113,  # I2S2_FS / PR.01
    36: 118,  # UART1_CTS / PR.06
    37: 107,  # GPIO26 / PQ.07
    38: 32,   # I2S2_DIN / PE.00
    40: 108,  # I2S2_DOUT / PQ.00
}


def reset_via_sysfs(gpio_num, active_low, pulse_ms, recover_ms):
    """Reset using sysfs GPIO (no library needed)."""
    gpio_path = f"/sys/class/gpio/gpio{gpio_num}"
    export_path = "/sys/class/gpio/export"
    unexport_path = "/sys/class/gpio/unexport"
    
    # Export GPIO
    if not os.path.exists(gpio_path):
        with open(export_path, 'w') as f:
            f.write(str(gpio_num))
        time.sleep(0.1)
    
    # Set direction to output with inactive value
    inactive = "0" if active_low else "1"
    active = "1" if active_low else "0"
    
    with open(f"{gpio_path}/direction", 'w') as f:
        f.write("out")
    with open(f"{gpio_path}/value", 'w') as f:
        f.write(inactive)
    
    # Toggle reset
    with open(f"{gpio_path}/value", 'w') as f:
        f.write(active)
    time.sleep(pulse_ms / 1000.0)
    with open(f"{gpio_path}/value", 'w') as f:
        f.write(inactive)
    time.sleep(recover_ms / 1000.0)
    
    # Unexport
    with open(unexport_path, 'w') as f:
        f.write(str(gpio_num))


def reset_via_jetson_gpio(board_pin, active_low, pulse_ms, recover_ms):
    """Reset using Jetson.GPIO library."""
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)
    
    inactive = GPIO.HIGH if active_low else GPIO.LOW
    active = GPIO.LOW if active_low else GPIO.HIGH
    
    GPIO.setup(board_pin, GPIO.OUT, initial=inactive)
    GPIO.output(board_pin, active)
    time.sleep(pulse_ms / 1000.0)
    GPIO.output(board_pin, inactive)
    time.sleep(recover_ms / 1000.0)
    
    GPIO.cleanup(board_pin)


def main():
    parser = argparse.ArgumentParser(description="Toggle BNO085 reset pin via Jetson GPIO.")
    parser.add_argument("--pin", type=int, default=7, help="BOARD pin number (default: 7 for GPIO09)")
    parser.add_argument("--active-low", action="store_true", default=True, help="Active-low reset (default: true)")
    parser.add_argument("--pulse-ms", type=int, default=100, help="Reset pulse width in ms (default: 100)")
    parser.add_argument("--recover-ms", type=int, default=500, help="Recovery delay in ms (default: 500)")
    parser.add_argument("--use-sysfs", action="store_true", help="Force sysfs method (bypass Jetson.GPIO)")
    args = parser.parse_args()
    
    if args.use_sysfs or not GPIO_AVAILABLE:
        # Use sysfs fallback
        if args.pin not in BOARD_TO_GPIO:
            print(f"Error: Unknown BOARD pin {args.pin}. Valid pins: {list(BOARD_TO_GPIO.keys())}", file=sys.stderr)
            sys.exit(1)
        gpio_num = BOARD_TO_GPIO[args.pin]
        print(f"Using sysfs GPIO {gpio_num} (BOARD pin {args.pin})")
        try:
            reset_via_sysfs(gpio_num, args.active_low, args.pulse_ms, args.recover_ms)
            print("✓ Reset complete")
        except PermissionError:
            print("✗ Permission denied. Run with sudo:", file=sys.stderr)
            print(f"  sudo python3 {sys.argv[0]}", file=sys.stderr)
            sys.exit(1)
    else:
        # Use Jetson.GPIO library
        print(f"Using Jetson.GPIO (BOARD pin {args.pin})")
        try:
            reset_via_jetson_gpio(args.pin, args.active_low, args.pulse_ms, args.recover_ms)
            print("✓ Reset complete")
        except Exception as e:
            print(f"✗ Jetson.GPIO failed: {e}", file=sys.stderr)
            print("Retrying with sysfs fallback...", file=sys.stderr)
            if args.pin not in BOARD_TO_GPIO:
                print(f"Error: Unknown BOARD pin {args.pin}", file=sys.stderr)
                sys.exit(1)
            gpio_num = BOARD_TO_GPIO[args.pin]
            reset_via_sysfs(gpio_num, args.active_low, args.pulse_ms, args.recover_ms)
            print("✓ Reset complete (via sysfs)")


if __name__ == "__main__":
    main()

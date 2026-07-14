#!/usr/bin/env python3
"""
enipturnt.py

EtherNet/IP version of modturnt for an Allen-Bradley MicroLogix 1100.

This follows the same working logic as enipturnt_simple.py:
    - Write remote command bits in the B3 file, e.g. B3:0/0.
    - Do NOT write directly to physical outputs O:0/x.
    - Read O:0/x back only to confirm what the PLC actually did.

Why B3 bits?
    The PLC ladder program owns the O file every scan. If a remote script writes
    O:0/x directly, the ladder can immediately overwrite it. Instead, add B3
    bits as parallel branches in ladder logic:

        I:0/0  OR  B3:0/0   -->  O:0/0
        I:0/1  OR  B3:0/1   -->  O:0/1
        I:0/2  OR  B3:0/2   -->  O:0/2
        I:0/3  OR  B3:0/3   -->  O:0/3
        I:0/4  OR  B3:0/4   -->  O:0/4

Requires:
    python -m pip install pycomm3

Examples:
    python enipturnt.py 192.168.0.31 --check-only
    python enipturnt.py 192.168.0.31 --mode sequential --once
    python enipturnt.py 192.168.0.31 --mode random
    python enipturnt.py 192.168.0.31 --mode random --count 20
"""

import argparse
import random
import sys
import time
from typing import Dict, Iterable, List, Sequence, Tuple

try:
    from pycomm3 import SLCDriver
except ImportError:
    SLCDriver = None


DEFAULT_HOST = "192.168.0.31"
DEFAULT_LIGHTS = {
    "light0": "B3:0/0",
    "light1": "B3:0/1",
    "light2": "B3:0/2",
    "light3": "B3:0/3",
    "light4": "B3:0/4",
}


def parse_lights(value: str) -> Tuple[str, ...]:
    """Parse a comma-separated list of light names or indexes."""
    items = tuple(item.strip() for item in value.split(",") if item.strip())
    if not items:
        raise argparse.ArgumentTypeError("at least one light is required")

    normalized = []
    for item in items:
        if item.isdigit():
            item = "light{}".format(item)
        if item not in DEFAULT_LIGHTS:
            raise argparse.ArgumentTypeError(
                "unknown light {!r}; use light0-light4 or 0-4".format(item)
            )
        normalized.append(item)
    return tuple(normalized)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Remote MicroLogix 1100 light blinker using EtherNet/IP B3 command bits.",
        epilog=(
            "Examples:\n"
            "  python enipturnt.py 192.168.0.31 --check-only\n"
            "  python enipturnt.py 192.168.0.31 --mode sequential --once\n"
            "  python enipturnt.py 192.168.0.31 --mode random\n"
            "  python enipturnt.py 192.168.0.31 --mode random --count 20\n"
            "  python enipturnt.py 192.168.0.31 --lights 0,1,4 --mode sequential\n"
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "host",
        nargs="?",
        default=DEFAULT_HOST,
        help="MicroLogix 1100 IP address (default: {})".format(DEFAULT_HOST),
    )
    parser.add_argument(
        "--mode",
        choices=("sequential", "random"),
        default="sequential",
        help="Blink mode: sequential modturnt-style loop or random blinking (default: sequential)",
    )
    parser.add_argument(
        "--lights",
        type=parse_lights,
        default=tuple(DEFAULT_LIGHTS.keys()),
        help="Comma-separated lights to control: 0-4 or light0-light4 (default: 0,1,2,3,4)",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=1.0,
        help="Sequential mode delay between writes in seconds (default: 1.0)",
    )
    parser.add_argument(
        "--count",
        type=int,
        help="Random mode: number of ON events before exiting. Default loops forever.",
    )
    parser.add_argument(
        "--min-on",
        type=float,
        default=0.10,
        help="Random mode minimum ON duration in seconds (default: 0.10)",
    )
    parser.add_argument(
        "--max-on",
        type=float,
        default=0.80,
        help="Random mode maximum ON duration in seconds (default: 0.80)",
    )
    parser.add_argument(
        "--min-gap",
        type=float,
        default=0.05,
        help="Random mode minimum OFF gap in seconds (default: 0.05)",
    )
    parser.add_argument(
        "--max-gap",
        type=float,
        default=0.60,
        help="Random mode maximum OFF gap in seconds (default: 0.60)",
    )
    parser.add_argument(
        "--once",
        action="store_true",
        help="Sequential mode: run one on/off cycle and exit.",
    )
    parser.add_argument(
        "--check-only",
        action="store_true",
        help="Connect and read current O:0/x output states, then exit without writing.",
    )
    parser.add_argument(
        "--seed",
        type=int,
        help="Optional random seed for repeatable random mode.",
    )
    args = parser.parse_args()

    if args.count is not None and args.count < 1:
        parser.error("--count must be 1 or greater")
    if args.interval < 0:
        parser.error("--interval cannot be negative")
    if min(args.min_on, args.max_on, args.min_gap, args.max_gap) < 0:
        parser.error("timing values cannot be negative")
    if args.min_on > args.max_on:
        parser.error("--min-on cannot be greater than --max-on")
    if args.min_gap > args.max_gap:
        parser.error("--min-gap cannot be greater than --max-gap")

    return args


def require_pycomm3() -> bool:
    """Return True if pycomm3 is available; otherwise print install help."""
    if SLCDriver is not None:
        return True
    print("ERROR: pycomm3 is not installed.")
    print("Install it with: python -m pip install pycomm3")
    return False


def light_output_address(name: str) -> str:
    """Convert light name/B3 address to matching physical output address."""
    return "O:0/" + DEFAULT_LIGHTS[name].split("/")[-1]


def response_failed(result) -> bool:
    """pycomm3 responses are truthy on success; also check .error when present."""
    return (not result) or getattr(result, "error", None)


def set_light(plc: SLCDriver, name: str, state: bool) -> None:
    """Turn a light on/off remotely by writing its B3 command bit."""
    address = DEFAULT_LIGHTS[name]
    result = plc.write((address, 1 if state else 0))
    if response_failed(result):
        raise RuntimeError("failed to write {} ({}): {}".format(name, address, result))
    print("{} ({}) set to {}".format(name, address, "ON" if state else "OFF"))


def read_light(plc: SLCDriver, name: str):
    """Read the actual physical output state from O:0/x for confirmation."""
    output_address = light_output_address(name)
    result = plc.read(output_address)
    if response_failed(result):
        raise RuntimeError("failed to read {}: {}".format(output_address, result))
    print("{} = {}".format(output_address, result.value))
    return result.value


def read_lights(plc: SLCDriver, lights: Sequence[str]) -> None:
    """Read actual physical output states for a group of lights."""
    for name in lights:
        read_light(plc, name)


def turn_all_off(plc: SLCDriver, lights: Sequence[str]) -> None:
    """Clear all selected B3 command bits."""
    for name in lights:
        try:
            set_light(plc, name, False)
        except Exception as exc:
            print("warning: could not turn {} off: {}".format(name, exc))


def run_sequential(plc: SLCDriver, lights: Sequence[str], interval: float, once: bool) -> None:
    """Classic modturnt-style loop: turn each light ON, then each light OFF."""
    print("Sequential mode on {}; Ctrl+C to stop.".format(", ".join(lights)))
    while True:
        for name in lights:
            set_light(plc, name, True)
            read_light(plc, name)
            time.sleep(interval)

        for name in lights:
            set_light(plc, name, False)
            read_light(plc, name)
            time.sleep(interval)

        if once:
            break


def run_random(
    plc: SLCDriver,
    lights: Sequence[str],
    count: int,
    min_on: float,
    max_on: float,
    min_gap: float,
    max_gap: float,
) -> None:
    """Independent random blink schedules using B3 command bits."""
    now = time.monotonic()
    states: Dict[str, bool] = {name: False for name in lights}
    next_toggle: Dict[str, float] = {
        name: now + random.uniform(min_gap, max_gap)
        for name in lights
    }
    blink_count = 0

    print("Random mode on {}; Ctrl+C to stop.".format(", ".join(lights)))
    while True:
        if count is not None and blink_count >= count and not any(states.values()):
            break

        name = min(next_toggle, key=next_toggle.get)
        sleep_for = next_toggle[name] - time.monotonic()
        if sleep_for > 0:
            time.sleep(sleep_for)

        if states[name]:
            states[name] = False
            set_light(plc, name, False)
            read_light(plc, name)
            next_toggle[name] = time.monotonic() + random.uniform(min_gap, max_gap)
        else:
            if count is not None and blink_count >= count:
                next_toggle[name] = float("inf")
                continue
            blink_count += 1
            on_time = random.uniform(min_on, max_on)
            states[name] = True
            print("blink {}: {} ON for {:.2f}s".format(blink_count, name, on_time))
            set_light(plc, name, True)
            read_light(plc, name)
            next_toggle[name] = time.monotonic() + on_time


def run(args: argparse.Namespace) -> int:
    if not require_pycomm3():
        return 1

    if args.seed is not None:
        random.seed(args.seed)

    print("Connecting to MicroLogix 1100 at {}...".format(args.host))
    print("Remote command bits:")
    for name in args.lights:
        print("  {} -> {} -> {}".format(name, DEFAULT_LIGHTS[name], light_output_address(name)))

    try:
        with SLCDriver(args.host) as plc:
            if args.check_only:
                print("\nConnected. Reading actual output states only:")
                read_lights(plc, args.lights)
                return 0

            if args.mode == "random":
                run_random(plc, args.lights, args.count, args.min_on, args.max_on, args.min_gap, args.max_gap)
            else:
                run_sequential(plc, args.lights, args.interval, args.once)

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
        return 130
    except Exception as exc:
        print("\nERROR: {}".format(exc))
        return 1
    finally:
        # Best effort cleanup: leave selected remote B3 command bits OFF.
        try:
            with SLCDriver(args.host) as plc:
                turn_all_off(plc, args.lights)
        except Exception:
            pass

    return 0


def main() -> int:
    return run(parse_args())


if __name__ == "__main__":
    sys.exit(main())

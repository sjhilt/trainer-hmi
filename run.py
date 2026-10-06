#!/usr/bin/env python3
"""
run.py — Launch GasPot + either the desktop HMI or the web HMI together.

Usage:
    python run.py                  # desktop HMI + GasPot
    python run.py --web            # web HMI + GasPot
    python run.py --web --port 8080
    python run.py --no-gaspot      # HMI only, no GasPot
    python run.py --gaspot-only    # GasPot only, no HMI

All GasPot and HMI arguments are forwarded.  GasPot runs in a background
thread so everything stays in one terminal window / one Ctrl-C.
"""

import argparse
import os
import signal
import subprocess
import sys
import time

# ── Paths ────────────────────────────────────────────────────────────
# GasPot is a LOCAL dependency cloned into this project directory.
# It is listed in .gitignore so it does not pollute the trainer-hmi repo.
#
# First-time setup — clone GasPot into the project root:
#   cd <this-repo>
#   git clone https://github.com/sjhilt/GasPot.git GasPot
#
# The Widget Factory config (gaspot-widget-factory.ini) IS tracked in
# this repo and is passed to GasPot via --config so we never need to
# modify GasPot's own config.ini.
SCRIPT_DIR      = os.path.dirname(os.path.abspath(__file__))
GASPOT_DIR      = os.path.join(SCRIPT_DIR, "GasPot")
GASPOT_PY       = os.path.join(GASPOT_DIR, "GasPot.py")
GASPOT_CONFIG   = os.path.join(SCRIPT_DIR, "gaspot-widget-factory.ini")
MFG_HMI_PY     = os.path.join(SCRIPT_DIR, "mfg_hmi.py")
WEB_HMI_PY     = os.path.join(SCRIPT_DIR, "web_hmi", "app.py")

PYTHON = sys.executable


def main():
    parser = argparse.ArgumentParser(
        description="Launch GasPot + HMI together.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "Examples:\n"
            "  python run.py                    Desktop HMI + GasPot\n"
            "  python run.py --web              Web HMI + GasPot\n"
            "  python run.py --web --port 8080  Web HMI on port 8080 + GasPot\n"
            "  python run.py --no-gaspot        HMI only (no GasPot)\n"
            "  python run.py --gaspot-only      GasPot only (no HMI)\n"
        ),
    )
    parser.add_argument("--web", action="store_true",
                        help="Use the web HMI instead of the desktop Tkinter HMI")
    parser.add_argument("--no-gaspot", action="store_true",
                        help="Skip launching GasPot (HMI only)")
    parser.add_argument("--gaspot-only", action="store_true",
                        help="Launch GasPot only (no HMI)")
    parser.add_argument("--gaspot-host", default="127.0.0.1",
                        help="GasPot listen address (default: 127.0.0.1)")
    parser.add_argument("--gaspot-port", type=int, default=10001,
                        help="GasPot TLS listen port (default: 10001)")
    # HMI-specific
    parser.add_argument("--port", type=int, default=5000,
                        help="Web HMI port (default: 5000, web mode only)")
    parser.add_argument("--host", default="127.0.0.1",
                        help="Web HMI bind address (default: 127.0.0.1; web mode only)")
    parser.add_argument("--poll-ms", type=int, default=500,
                        help="PLC polling interval ms (default: 500)")
    parser.add_argument("--process-ms", type=int, default=300,
                        help="Process step interval ms (default: 300)")
    parser.add_argument("--source-ip", default=None,
                        help="Local IP to bind Modbus connection to")

    args = parser.parse_args()

    procs = []

    # ── GasPot ───────────────────────────────────────────────────────
    if not args.no_gaspot:
        if not os.path.isfile(GASPOT_PY):
            print(f"[run] WARNING: GasPot not found at {GASPOT_PY}")
            print("[run]          Clone it locally first:")
            print("[run]            git clone https://github.com/sjhilt/GasPot.git GasPot")
            print("[run]          Continuing without GasPot.")
        else:
            gaspot_cmd = [PYTHON, GASPOT_PY, "--config", GASPOT_CONFIG]
            print(f"[run] Starting GasPot: {' '.join(gaspot_cmd)}")
            procs.append(subprocess.Popen(
                gaspot_cmd,
                cwd=GASPOT_DIR,
                stdout=sys.stdout,
                stderr=sys.stderr,
            ))
            # Give GasPot a moment to bind
            time.sleep(1.0)

    if args.gaspot_only:
        if procs:
            print("[run] GasPot running.  Press Ctrl+C to stop.")
            try:
                procs[0].wait()
            except KeyboardInterrupt:
                pass
            finally:
                _cleanup(procs)
            return
        else:
            print("[run] No GasPot process to run.")
            return

    # ── HMI ──────────────────────────────────────────────────────────
    if args.web:
        hmi_cmd = [
            PYTHON, WEB_HMI_PY,
            "--host", args.host,
            "--port", str(args.port),
            "--process-ms", str(args.process_ms),
            "--gaspot-host", args.gaspot_host,
            "--gaspot-port", str(args.gaspot_port),
        ]
        if args.source_ip:
            hmi_cmd += ["--source-ip", args.source_ip]
        if args.no_gaspot:
            hmi_cmd.append("--disable-gaspot")
    else:
        hmi_cmd = [
            PYTHON, MFG_HMI_PY,
            "--poll-ms", str(args.poll_ms),
            "--process-ms", str(args.process_ms),
            "--gaspot-host", args.gaspot_host,
            "--gaspot-port", str(args.gaspot_port),
        ]
        if args.source_ip:
            hmi_cmd += ["--source-ip", args.source_ip]
        if args.no_gaspot:
            hmi_cmd.append("--disable-gaspot")

    print(f"[run] Starting HMI: {' '.join(hmi_cmd)}")
    procs.append(subprocess.Popen(
        hmi_cmd,
        cwd=SCRIPT_DIR,
        stdout=sys.stdout,
        stderr=sys.stderr,
    ))

    # ── Wait ─────────────────────────────────────────────────────────
    print("[run] All processes started.  Press Ctrl+C to stop everything.")
    try:
        # Wait for the HMI process (last one added) to exit
        procs[-1].wait()
    except KeyboardInterrupt:
        pass
    finally:
        _cleanup(procs)


def _cleanup(procs):
    """Terminate all child processes gracefully."""
    for p in procs:
        if p.poll() is None:
            print(f"[run] Stopping PID {p.pid}...")
            p.terminate()
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                p.kill()
    print("[run] All processes stopped.")


if __name__ == "__main__":
    main()

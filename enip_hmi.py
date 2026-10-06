#!/usr/bin/env python3
"""
enip_hmi.py

Simple HMI for the MicroLogix 1100 EtherNet/IP/B3 remote-light demo.

This uses the same working logic as enipturnt_simple.py and enipturnt.py:
    - Write remote command bits B3:0/x.
    - Do NOT write directly to physical outputs O:0/x.
    - Read physical inputs I:0/x for switch/button status.
    - Read actual outputs O:0/x for light confirmation.

Assumed ladder pattern:
    I:0/0 OR B3:0/0  --> O:0/0
    I:0/1 OR B3:0/1  --> O:0/1
    I:0/2 OR B3:0/2  --> O:0/2
    I:0/3 OR B3:0/3  --> O:0/3
    I:0/4 OR B3:0/4  --> O:0/4

Configured controls:
    0/0, 0/1, 0/4 = toggle switches
    0/2, 0/3      = momentary push buttons

Requires:
    python -m pip install pycomm3

Run:
    python enip_hmi.py 192.168.0.31
"""

import argparse
import random
import sys
import tkinter as tk
from tkinter import messagebox

from hmi_branding import apply_window_branding, configure_process_branding

try:
    from pycomm3 import SLCDriver
except ImportError:
    SLCDriver = None


DEFAULT_HOST = "192.168.0.31"

DEVICES = [
    {"idx": 0, "name": "Light 0", "kind": "toggle", "input": "I:0/0", "command": "B3:0/0", "output": "O:0/0"},
    {"idx": 1, "name": "Light 1", "kind": "toggle", "input": "I:0/1", "command": "B3:0/1", "output": "O:0/1"},
    {"idx": 2, "name": "Light 2", "kind": "push",   "input": "I:0/2", "command": "B3:0/2", "output": "O:0/2"},
    {"idx": 3, "name": "Light 3", "kind": "push",   "input": "I:0/3", "command": "B3:0/3", "output": "O:0/3"},
    {"idx": 4, "name": "Light 4", "kind": "toggle", "input": "I:0/4", "command": "B3:0/4", "output": "O:0/4"},
]

BG = "#121722"
PANEL = "#1f2635"
TEXT = "#f2f4f8"
MUTED = "#9aa5b1"
OFF = "#303846"
GREEN = "#00d26a"
RED = "#ff3b4f"
AMBER = "#ffb000"
ERR = "#ff5555"
BLUE = "#3d7eff"


def parse_args():
    parser = argparse.ArgumentParser(
        description="MicroLogix 1100 HMI using B3 remote command bits over EtherNet/IP.",
        epilog=(
            "Examples:\n"
            "  python enip_hmi.py 192.168.0.31\n"
            "  python enip_hmi.py 192.168.0.31 --poll-ms 250\n"
            "  python enip_hmi.py 192.168.0.31 --kitt-ms 120\n\n"
            "Controls:\n"
            "  B3:0/0, B3:0/1, B3:0/4 = toggle controls\n"
            "  B3:0/2, B3:0/3 = momentary push-button controls"
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
        "--poll-ms",
        type=int,
        default=500,
        help="Polling interval in milliseconds (default: 500)",
    )
    parser.add_argument(
        "--kitt-ms",
        type=int,
        default=150,
        help="KITT scanner step delay in milliseconds (default: 150)",
    )
    parser.add_argument(
        "--random-min-ms",
        type=int,
        default=150,
        help="Random mode minimum delay between actions in milliseconds (default: 150)",
    )
    parser.add_argument(
        "--random-max-ms",
        type=int,
        default=900,
        help="Random mode maximum delay between actions in milliseconds (default: 900)",
    )
    parser.add_argument(
        "--pulse-min-ms",
        type=int,
        default=120,
        help="Random mode minimum push-button pulse in milliseconds (default: 120)",
    )
    parser.add_argument(
        "--pulse-max-ms",
        type=int,
        default=450,
        help="Random mode maximum push-button pulse in milliseconds (default: 450)",
    )
    args = parser.parse_args()
    if args.random_min_ms < 1 or args.random_max_ms < 1:
        parser.error("random timing values must be at least 1 ms")
    if args.pulse_min_ms < 1 or args.pulse_max_ms < 1:
        parser.error("pulse timing values must be at least 1 ms")
    if args.random_min_ms > args.random_max_ms:
        parser.error("--random-min-ms cannot be greater than --random-max-ms")
    if args.pulse_min_ms > args.pulse_max_ms:
        parser.error("--pulse-min-ms cannot be greater than --pulse-max-ms")
    return args


def normalize_response(response):
    return response if isinstance(response, list) else [response]


def response_failed(response):
    return (not response) or getattr(response, "error", None)


class EnipHMI:
    def __init__(self, root, host, poll_ms, kitt_ms, random_min_ms, random_max_ms, pulse_min_ms, pulse_max_ms):
        self.root = root
        self.host = host
        self.poll_ms = poll_ms
        self.kitt_ms = kitt_ms
        self.random_min_ms = random_min_ms
        self.random_max_ms = random_max_ms
        self.pulse_min_ms = pulse_min_ms
        self.pulse_max_ms = pulse_max_ms
        self.plc = None
        self.connected = False

        self.input_states = [False] * len(DEVICES)
        self.command_states = [False] * len(DEVICES)
        self.output_states = [False] * len(DEVICES)
        self.rows = []
        self.random_active = False
        self.random_job = None
        self.kitt_active = False
        self.kitt_job = None
        self.kitt_index = 0
        self.kitt_direction = 1

        self.root.title("MicroLogix 1100 ENIP HMI - {}".format(host))
        apply_window_branding(self.root)
        self.root.configure(bg=BG)
        self.root.resizable(False, False)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self.build_ui()
        self.connect()
        self.poll()

    def build_ui(self):
        tk.Label(
            self.root,
            text="MicroLogix 1100 ENIP HMI",
            font=("Segoe UI", 20, "bold"),
            bg=BG,
            fg=TEXT,
        ).pack(pady=(16, 4))

        self.status_var = tk.StringVar(value="Connecting...")
        self.status_label = tk.Label(
            self.root,
            textvariable=self.status_var,
            font=("Segoe UI", 10),
            bg=BG,
            fg=MUTED,
        )
        self.status_label.pack(pady=(0, 10))

        header = tk.Frame(self.root, bg=BG)
        header.pack(padx=18, pady=(0, 4), fill="x")
        for col, text, width in [
            (0, "Physical Input", 22),
            (1, "Remote Command (B3)", 24),
            (2, "Actual Output", 18),
        ]:
            tk.Label(
                header,
                text=text,
                width=width,
                font=("Segoe UI", 11, "bold"),
                bg=BG,
                fg=TEXT,
            ).grid(row=0, column=col, padx=8)

        panel = tk.Frame(self.root, bg=PANEL, padx=12, pady=12)
        panel.pack(padx=18, pady=6)

        for row_idx, device in enumerate(DEVICES):
            self.add_device_row(panel, row_idx, device)

        self.random_btn_var = tk.StringVar(value="▶ Random")
        self.random_btn = tk.Button(
            self.root,
            textvariable=self.random_btn_var,
            font=("Segoe UI", 12, "bold"),
            fg="#ffffff",
            bg="#4a4a8a",
            activebackground="#6a6aaa",
            activeforeground="#ffffff",
            relief=tk.FLAT,
            padx=22,
            pady=6,
            cursor="hand2",
            command=self.toggle_random,
        )
        self.random_btn.pack(pady=(8, 4))

        random_timing = tk.Frame(self.root, bg=BG)
        random_timing.pack(pady=(2, 6))

        self.random_min_var = tk.StringVar(value=str(self.random_min_ms))
        self.random_max_var = tk.StringVar(value=str(self.random_max_ms))
        self.pulse_min_var = tk.StringVar(value=str(self.pulse_min_ms))
        self.pulse_max_var = tk.StringVar(value=str(self.pulse_max_ms))

        for col, (label, var) in enumerate([
            ("Rand min ms", self.random_min_var),
            ("Rand max ms", self.random_max_var),
            ("Pulse min ms", self.pulse_min_var),
            ("Pulse max ms", self.pulse_max_var),
        ]):
            tk.Label(
                random_timing,
                text=label,
                font=("Segoe UI", 8),
                bg=BG,
                fg=MUTED,
            ).grid(row=0, column=col, padx=4)
            tk.Entry(
                random_timing,
                textvariable=var,
                width=8,
                justify="center",
                font=("Segoe UI", 9),
            ).grid(row=1, column=col, padx=4)

        self.kitt_btn_var = tk.StringVar(value="▶ KITT Mode")
        self.kitt_btn = tk.Button(
            self.root,
            textvariable=self.kitt_btn_var,
            font=("Segoe UI", 12, "bold"),
            fg="#ffffff",
            bg="#8a1f2d",
            activebackground="#bc2f44",
            activeforeground="#ffffff",
            relief=tk.FLAT,
            padx=22,
            pady=6,
            cursor="hand2",
            command=self.toggle_kitt,
        )
        self.kitt_btn.pack(pady=(4, 4))

        tk.Label(
            self.root,
            text="Writes B3:0/x command bits only. Outputs O:0/x are read back for confirmation.",
            font=("Segoe UI", 9),
            bg=BG,
            fg=MUTED,
        ).pack(pady=(8, 14))

    def add_device_row(self, parent, row_idx, device):
        # Input indicator
        input_frame = tk.Frame(parent, bg=PANEL)
        input_frame.grid(row=row_idx, column=0, padx=8, pady=7, sticky="w")
        input_canvas = tk.Canvas(input_frame, width=70, height=42, bg=PANEL, highlightthickness=0)
        input_canvas.pack(side=tk.LEFT, padx=(0, 8))

        if device["kind"] == "toggle":
            input_track = input_canvas.create_rectangle(6, 12, 64, 32, fill=OFF, outline="#5d6b80", width=2)
            input_knob = input_canvas.create_oval(10, 9, 34, 35, fill=MUTED, outline="")
            input_shapes = (input_track, input_knob)
        else:
            input_button = input_canvas.create_oval(18, 5, 52, 39, fill=OFF, outline="#5d6b80", width=3)
            input_shapes = (input_button,)

        input_var = tk.StringVar(value="{} OFF".format(device["input"]))
        tk.Label(input_frame, textvariable=input_var, width=16, anchor="w", bg=PANEL, fg=TEXT).pack(side=tk.LEFT)

        # Command control
        command_frame = tk.Frame(parent, bg=PANEL)
        command_frame.grid(row=row_idx, column=1, padx=8, pady=7, sticky="w")
        command_canvas = tk.Canvas(command_frame, width=76, height=46, bg=PANEL, highlightthickness=0, cursor="hand2")
        command_canvas.pack(side=tk.LEFT, padx=(0, 8))

        if device["kind"] == "toggle":
            cmd_track = command_canvas.create_rectangle(7, 14, 69, 36, fill=OFF, outline="#5d6b80", width=2)
            cmd_knob = command_canvas.create_oval(11, 10, 39, 40, fill=MUTED, outline="")
            command_canvas.bind("<Button-1>", lambda _event, idx=row_idx: self.toggle_command(idx))
            cmd_shapes = (cmd_track, cmd_knob)
        else:
            cmd_button = command_canvas.create_oval(19, 5, 57, 43, fill=OFF, outline="#5d6b80", width=3)
            command_canvas.bind("<ButtonPress-1>", lambda _event, idx=row_idx: self.set_command(idx, True))
            command_canvas.bind("<ButtonRelease-1>", lambda _event, idx=row_idx: self.set_command(idx, False))
            cmd_shapes = (cmd_button,)

        command_var = tk.StringVar(value="{} OFF".format(device["command"]))
        tk.Label(command_frame, textvariable=command_var, width=17, anchor="w", bg=PANEL, fg=TEXT).pack(side=tk.LEFT)

        # Output indicator
        output_frame = tk.Frame(parent, bg=PANEL)
        output_frame.grid(row=row_idx, column=2, padx=8, pady=7, sticky="w")
        output_canvas = tk.Canvas(output_frame, width=46, height=46, bg=PANEL, highlightthickness=0)
        output_canvas.pack(side=tk.LEFT, padx=(0, 8))
        output_lamp = output_canvas.create_oval(7, 7, 39, 39, fill=OFF, outline="#5d6b80", width=3)
        output_var = tk.StringVar(value="{} OFF".format(device["output"]))
        tk.Label(output_frame, textvariable=output_var, width=13, anchor="w", bg=PANEL, fg=TEXT).pack(side=tk.LEFT)

        self.rows.append({
            "input_canvas": input_canvas,
            "input_shapes": input_shapes,
            "input_var": input_var,
            "command_canvas": command_canvas,
            "command_shapes": cmd_shapes,
            "command_var": command_var,
            "output_canvas": output_canvas,
            "output_lamp": output_lamp,
            "output_var": output_var,
        })

    def connect(self):
        if SLCDriver is None:
            self.connected = False
            self.status_var.set("pycomm3 missing - run: python -m pip install pycomm3")
            self.status_label.configure(fg=ERR)
            return

        self.disconnect()
        try:
            self.plc = SLCDriver(self.host)
            self.plc.open()
            self.connected = True
            self.status_var.set("Connected to {}".format(self.host))
            self.status_label.configure(fg=GREEN)
        except Exception as exc:
            self.connected = False
            self.status_var.set("Connection failed: {}".format(exc))
            self.status_label.configure(fg=ERR)
            self.disconnect()

    def disconnect(self):
        if self.plc is not None:
            try:
                self.plc.close()
            except Exception:
                pass
        self.plc = None
        self.connected = False

    def read_bit(self, address):
        result = self.plc.read(address)
        if response_failed(result):
            raise RuntimeError("failed to read {}: {}".format(address, result))
        return bool(result.value)

    def write_bit(self, address, state):
        result = self.plc.write((address, 1 if state else 0))
        if response_failed(result):
            raise RuntimeError("failed to write {}={}: {}".format(address, state, result))

    def poll(self):
        if not self.connected:
            self.connect()

        if self.connected:
            try:
                for idx, device in enumerate(DEVICES):
                    self.input_states[idx] = self.read_bit(device["input"])
                    self.command_states[idx] = self.read_bit(device["command"])
                    self.output_states[idx] = self.read_bit(device["output"])
                self.render()
            except Exception as exc:
                self.status_var.set("Read lost: {}".format(exc))
                self.status_label.configure(fg=ERR)
                self.disconnect()

        self.root.after(self.poll_ms, self.poll)

    def render(self):
        for idx, device in enumerate(DEVICES):
            row = self.rows[idx]
            self.render_input(row, device, self.input_states[idx])
            self.render_command(row, device, self.command_states[idx])
            self.render_output(row, device, self.output_states[idx])

    def render_input(self, row, device, state):
        if device["kind"] == "toggle":
            track, knob = row["input_shapes"]
            row["input_canvas"].itemconfig(track, fill=GREEN if state else OFF)
            row["input_canvas"].coords(knob, 38, 9, 62, 35) if state else row["input_canvas"].coords(knob, 10, 9, 34, 35)
            row["input_canvas"].itemconfig(knob, fill="#ffffff" if state else MUTED)
            row["input_var"].set("{} {}".format(device["input"], "ON" if state else "OFF"))
        else:
            button = row["input_shapes"][0]
            row["input_canvas"].itemconfig(button, fill=AMBER if state else OFF)
            row["input_var"].set("{} {}".format(device["input"], "PRESSED" if state else "RELEASED"))

    def render_command(self, row, device, state):
        if device["kind"] == "toggle":
            track, knob = row["command_shapes"]
            row["command_canvas"].itemconfig(track, fill=BLUE if state else OFF)
            row["command_canvas"].coords(knob, 37, 10, 65, 40) if state else row["command_canvas"].coords(knob, 11, 10, 39, 40)
            row["command_canvas"].itemconfig(knob, fill="#ffffff" if state else MUTED)
            row["command_var"].set("{} {}".format(device["command"], "ON" if state else "OFF"))
        else:
            button = row["command_shapes"][0]
            row["command_canvas"].itemconfig(button, fill=BLUE if state else OFF)
            row["command_var"].set("{} {}".format(device["command"], "PRESSED" if state else "RELEASED"))

    def render_output(self, row, device, state):
        row["output_canvas"].itemconfig(row["output_lamp"], fill=RED if state else OFF)
        row["output_var"].set("{} {}".format(device["output"], "ON" if state else "OFF"))

    def set_command(self, idx, state):
        if not self.connected:
            return
        device = DEVICES[idx]
        try:
            self.write_bit(device["command"], state)
            self.command_states[idx] = state
            self.render_command(self.rows[idx], device, state)
            self.status_var.set("Set {} -> {}".format(device["command"], "ON" if state else "OFF"))
            self.status_label.configure(fg=GREEN)
        except Exception as exc:
            self.status_var.set("Write failed: {}".format(exc))
            self.status_label.configure(fg=ERR)
            self.disconnect()

    def toggle_command(self, idx):
        self.set_command(idx, not self.command_states[idx])

    def toggle_random(self):
        """Start or stop random B3 command activity."""
        if self.random_active:
            self.random_active = False
            self.random_btn_var.set("▶ Random")
            self.random_btn.configure(bg="#4a4a8a")
            if self.random_job is not None:
                self.root.after_cancel(self.random_job)
                self.random_job = None
            self.status_var.set("Random mode stopped")
            self.status_label.configure(fg=GREEN)
        else:
            if self.kitt_active:
                self.stop_kitt(clear_commands=True, quiet=True)
            self.random_active = True
            self.random_btn_var.set("■ Stop Random")
            self.random_btn.configure(bg="#aa3333")
            self.status_var.set("Random mode active")
            self.status_label.configure(fg=AMBER)
            self.random_tick()

    def get_ms_range(self, min_var, max_var, fallback_min, fallback_max, label):
        """Read and validate a min/max millisecond range from HMI entries."""
        try:
            min_ms = int(min_var.get())
            max_ms = int(max_var.get())
            if min_ms < 1 or max_ms < 1:
                raise ValueError("values must be at least 1")
            if min_ms > max_ms:
                raise ValueError("min cannot be greater than max")
            return min_ms, max_ms
        except ValueError as exc:
            self.status_var.set("Invalid {} timing: {}; using {}-{} ms".format(label, exc, fallback_min, fallback_max))
            self.status_label.configure(fg=ERR)
            min_var.set(str(fallback_min))
            max_var.set(str(fallback_max))
            return fallback_min, fallback_max

    def random_delay_range(self):
        return self.get_ms_range(self.random_min_var, self.random_max_var, self.random_min_ms, self.random_max_ms, "random delay")

    def pulse_delay_range(self):
        return self.get_ms_range(self.pulse_min_var, self.pulse_max_var, self.pulse_min_ms, self.pulse_max_ms, "pulse")

    def random_tick(self):
        """Randomly change one B3 command bit, then schedule the next action."""
        if not self.random_active:
            return

        if not self.connected:
            self.status_var.set("Random paused - not connected")
            self.status_label.configure(fg=ERR)
            self.random_job = self.root.after(1000, self.random_tick)
            return

        idx = random.randrange(len(DEVICES))
        device = DEVICES[idx]

        try:
            if device["kind"] == "push":
                # Momentary devices get a short ON pulse, then OFF.
                self.write_bit(device["command"], True)
                self.command_states[idx] = True
                self.render_command(self.rows[idx], device, True)
                pulse_min_ms, pulse_max_ms = self.pulse_delay_range()
                pulse_ms = random.randint(pulse_min_ms, pulse_max_ms)
                self.root.after(pulse_ms, lambda i=idx: self.random_release_push(i))
            else:
                # Toggle devices randomly latch ON or OFF.
                new_state = random.choice([True, False])
                self.write_bit(device["command"], new_state)
                self.command_states[idx] = new_state
                self.render_command(self.rows[idx], device, new_state)

            self.status_var.set("Random wrote {}".format(device["command"]))
            self.status_label.configure(fg=AMBER)
        except Exception as exc:
            self.status_var.set("Random write failed: {}".format(exc))
            self.status_label.configure(fg=ERR)
            self.disconnect()

        random_min_ms, random_max_ms = self.random_delay_range()
        delay_ms = random.randint(random_min_ms, random_max_ms)
        self.random_job = self.root.after(delay_ms, self.random_tick)

    def random_release_push(self, idx):
        """Release a momentary B3 push-button bit after a random pulse."""
        if not self.connected:
            return
        device = DEVICES[idx]
        if device["kind"] != "push":
            return
        try:
            self.write_bit(device["command"], False)
            self.command_states[idx] = False
            self.render_command(self.rows[idx], device, False)
        except Exception as exc:
            self.status_var.set("Random release failed: {}".format(exc))
            self.status_label.configure(fg=ERR)
            self.disconnect()

    def stop_random(self, quiet=False):
        """Stop random mode and cancel its scheduled job."""
        self.random_active = False
        self.random_btn_var.set("▶ Random")
        self.random_btn.configure(bg="#4a4a8a")
        if self.random_job is not None:
            try:
                self.root.after_cancel(self.random_job)
            except Exception:
                pass
            self.random_job = None
        if not quiet:
            self.status_var.set("Random mode stopped")
            self.status_label.configure(fg=GREEN)

    def toggle_kitt(self):
        """Start or stop KITT scanner mode."""
        if self.kitt_active:
            self.stop_kitt(clear_commands=True)
        else:
            if self.random_active:
                self.stop_random(quiet=True)
            self.kitt_active = True
            self.kitt_index = 0
            self.kitt_direction = 1
            self.kitt_btn_var.set("■ Stop KITT")
            self.kitt_btn.configure(bg="#cc1f3a")
            self.status_var.set("KITT mode active")
            self.status_label.configure(fg=RED)
            self.kitt_tick()

    def stop_kitt(self, clear_commands=True, quiet=False):
        """Stop KITT mode and optionally clear all B3 command bits."""
        self.kitt_active = False
        self.kitt_btn_var.set("▶ KITT Mode")
        self.kitt_btn.configure(bg="#8a1f2d")
        if self.kitt_job is not None:
            try:
                self.root.after_cancel(self.kitt_job)
            except Exception:
                pass
            self.kitt_job = None

        if clear_commands and self.connected:
            try:
                for idx, device in enumerate(DEVICES):
                    self.write_bit(device["command"], False)
                    self.command_states[idx] = False
                self.render()
            except Exception as exc:
                self.status_var.set("KITT clear failed: {}".format(exc))
                self.status_label.configure(fg=ERR)
                self.disconnect()
                return

        if not quiet:
            self.status_var.set("KITT mode stopped")
            self.status_label.configure(fg=GREEN)

    def kitt_tick(self):
        """Sweep one active B3 command bit back and forth like the KITT scanner."""
        if not self.kitt_active:
            return

        if not self.connected:
            self.status_var.set("KITT paused - not connected")
            self.status_label.configure(fg=ERR)
            self.kitt_job = self.root.after(1000, self.kitt_tick)
            return

        try:
            for idx, device in enumerate(DEVICES):
                state = idx == self.kitt_index
                self.write_bit(device["command"], state)
                self.command_states[idx] = state
            self.render()
            self.status_var.set("KITT mode: {}".format(DEVICES[self.kitt_index]["command"]))
            self.status_label.configure(fg=RED)
        except Exception as exc:
            self.status_var.set("KITT write failed: {}".format(exc))
            self.status_label.configure(fg=ERR)
            self.disconnect()
            self.kitt_job = self.root.after(1000, self.kitt_tick)
            return

        if self.kitt_index >= len(DEVICES) - 1:
            self.kitt_direction = -1
        elif self.kitt_index <= 0:
            self.kitt_direction = 1
        self.kitt_index += self.kitt_direction

        self.kitt_job = self.root.after(self.kitt_ms, self.kitt_tick)

    def close(self):
        if self.random_job is not None:
            try:
                self.root.after_cancel(self.random_job)
            except Exception:
                pass
            self.random_job = None
        if self.kitt_job is not None:
            try:
                self.root.after_cancel(self.kitt_job)
            except Exception:
                pass
            self.kitt_job = None
        if self.kitt_active:
            self.stop_kitt(clear_commands=True, quiet=True)
        self.disconnect()
        self.root.destroy()


def main():
    args = parse_args()
    configure_process_branding()
    root = tk.Tk()

    if SLCDriver is None:
        messagebox.showerror(
            "Missing dependency",
            "pycomm3 is not installed.\n\nRun:\npython -m pip install pycomm3",
        )

    EnipHMI(root, args.host, args.poll_ms, args.kitt_ms, args.random_min_ms, args.random_max_ms, args.pulse_min_ms, args.pulse_max_ms)
    root.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())

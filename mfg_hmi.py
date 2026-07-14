#!/usr/bin/env python3
"""
mfg_hmi.py

All-in-one Manufacturing Process HMI

Connects to all four PLCs simultaneously and presents a unified
manufacturing-themed dashboard.  A "Run Process" button starts a
simulated production cycle that exercises the lights across all PLCs
in patterns that look like a running assembly line.

PLCs:
    MicroLogix 1100 @ 192.168.0.30  (ENIP/SLC)  B3:1/0-4
    MicroLogix 1100 @ 192.168.0.31  (ENIP/SLC)  B3:0/0-4
    Siemens S7-1200 @ 192.168.0.2   (S7comm)    M0.0-4
    Phoenix Contact @ 192.168.0.3   (Modbus/TCP) Coils 0-1

Requires:
    python -m pip install pycomm3 python-snap7

Run:
    python mfg_hmi.py
"""

import argparse
import random
import socket
import struct
import sys
import threading
import time
import tkinter as tk
from tkinter import messagebox

# ---------- Optional imports ----------
try:
    from pycomm3 import SLCDriver
except ImportError:
    SLCDriver = None

try:
    import snap7
    from snap7.util import get_bool, set_bool
    try:
        from snap7.type import Areas
    except ImportError:
        from snap7.types import Areas
except ImportError:
    snap7 = None
    get_bool = None
    set_bool = None
    Areas = None

# ======================================================================
# Colour palette
# ======================================================================
BG       = "#0d1117"
PANEL    = "#161b22"
HEADER   = "#21262d"
TEXT     = "#f0f6fc"
MUTED    = "#8b949e"
OFF      = "#30363d"
GREEN    = "#3fb950"
RED      = "#f85149"
AMBER    = "#d29922"
BLUE     = "#58a6ff"
TEAL     = "#39d2c0"
ERR      = "#ff7b72"
PROCESS_RUN  = "#238636"
PROCESS_STOP = "#da3633"

# ======================================================================
# PLC definitions
# ======================================================================
MLGX_30_DEVICES = [
    {"idx": i, "kind": k, "input": "I:0/{}".format(i), "command": "B3:1/{}".format(i), "output": "O:0/{}".format(i)}
    for i, k in [(0,"toggle"),(1,"toggle"),(2,"push"),(3,"push"),(4,"toggle")]
]
MLGX_31_DEVICES = [
    {"idx": i, "kind": k, "input": "I:0/{}".format(i), "command": "B3:0/{}".format(i), "output": "O:0/{}".format(i)}
    for i, k in [(0,"toggle"),(1,"toggle"),(2,"push"),(3,"push"),(4,"toggle")]
]
S7_DEVICES = [
    {"idx": i, "kind": k, "input": "I0.{}".format(i), "command": "M0.{}".format(i), "output": "Q0.{}".format(i)}
    for i, k in [(0,"toggle"),(1,"toggle"),(2,"push"),(3,"push"),(4,"toggle")]
]
MODBUS_DEVICES = [
    {"idx": 0, "kind": "toggle", "label": "Coil 0"},
    {"idx": 1, "kind": "toggle", "label": "Coil 1"},
]

PLCS = [
    {"name": "MicroLogix .30",   "ip": "192.168.0.30", "proto": "enip",   "devices": MLGX_30_DEVICES, "colour": BLUE},
    {"name": "MicroLogix .31",   "ip": "192.168.0.31", "proto": "enip",   "devices": MLGX_31_DEVICES, "colour": GREEN},
    {"name": "Siemens S7-1200",  "ip": "192.168.0.2",  "proto": "s7",     "devices": S7_DEVICES,      "colour": TEAL},
    {"name": "Phoenix Contact",  "ip": "192.168.0.3",  "proto": "modbus", "devices": MODBUS_DEVICES,  "colour": AMBER},
]


# ======================================================================
# Connection helpers
# ======================================================================

def response_failed(response):
    return (not response) or getattr(response, "error", None)


class EnipConn:
    """EtherNet/IP SLC connection."""
    def __init__(self, host):
        self.host = host
        self.plc = None

    @property
    def connected(self):
        return self.plc is not None

    def connect(self):
        self.close()
        if SLCDriver is None:
            raise RuntimeError("pycomm3 not installed")
        self.plc = SLCDriver(self.host)
        self.plc.open()

    def close(self):
        if self.plc is not None:
            try:
                self.plc.close()
            except Exception:
                pass
        self.plc = None

    def read_bit(self, addr):
        r = self.plc.read(addr)
        if response_failed(r):
            raise RuntimeError("read {} failed: {}".format(addr, r))
        return bool(r.value)

    def write_bit(self, addr, state):
        r = self.plc.write((addr, 1 if state else 0))
        if response_failed(r):
            raise RuntimeError("write {}={} failed: {}".format(addr, state, r))


def parse_s7_bit_address(address):
    address = address.strip().upper()
    prefix = address[0]
    byte_text, bit_text = address[1:].split(".", 1)
    byte_idx, bit_idx = int(byte_text), int(bit_text)
    if prefix in ("I", "E"):
        area = Areas.PE
    elif prefix in ("Q", "A"):
        area = Areas.PA
    else:
        area = Areas.MK
    return area, byte_idx, bit_idx


class S7Conn:
    """Siemens S7 connection via snap7."""
    def __init__(self, host, rack=0, slot=1, port=102):
        self.host = host
        self.rack = rack
        self.slot = slot
        self.port = port
        self.client = None

    @property
    def connected(self):
        if self.client is None:
            return False
        try:
            return bool(self.client.get_connected())
        except Exception:
            return False

    def connect(self):
        self.close()
        if snap7 is None:
            raise RuntimeError("python-snap7 not installed")
        self.client = snap7.client.Client()
        try:
            self.client.connect(self.host, self.rack, self.slot, self.port)
        except TypeError:
            self.client.connect(self.host, self.rack, self.slot, tcpport=self.port)

    def close(self):
        if self.client is not None:
            try:
                self.client.disconnect()
            except Exception:
                pass
            try:
                self.client.destroy()
            except Exception:
                pass
        self.client = None

    def read_bit(self, addr):
        area, byte_idx, bit_idx = parse_s7_bit_address(addr)
        data = self.client.read_area(area, 0, byte_idx, 1)
        return bool(get_bool(data, 0, bit_idx))

    def write_bit(self, addr, state):
        area, byte_idx, bit_idx = parse_s7_bit_address(addr)
        data = self.client.read_area(area, 0, byte_idx, 1)
        set_bool(data, 0, bit_idx, bool(state))
        self.client.write_area(area, 0, byte_idx, data)


class ModbusConn:
    """Raw Modbus/TCP connection."""
    def __init__(self, host, port=502, source_ip=None, timeout=3.0):
        self.host = host
        self.port = port
        self.source_ip = source_ip
        self.timeout = timeout
        self.sock = None
        self._tx = 0

    @property
    def connected(self):
        return self.sock is not None

    def connect(self):
        self.close()
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(self.timeout)
        if self.source_ip:
            s.bind((self.source_ip, 0))
        s.connect((self.host, self.port))
        self.sock = s

    def close(self):
        if self.sock is not None:
            try:
                self.sock.close()
            except Exception:
                pass
        self.sock = None

    def _sr(self, pkt):
        self.sock.sendall(pkt)
        return self.sock.recv(4096)

    def read_coils(self, start=0, count=2, uid=1):
        self._tx = (self._tx + 1) & 0xFFFF
        pkt = struct.pack(">HHHBBHH", self._tx, 0, 6, uid, 0x01, start, count)
        resp = self._sr(pkt)
        if len(resp) < 10:
            raise ValueError("Short modbus response")
        if resp[7] & 0x80:
            raise ValueError("Modbus exception 0x{:02x}".format(resp[8]))
        bc = resp[8]
        bits = []
        for b in resp[9:9+bc]:
            for i in range(8):
                bits.append(bool(b & (1 << i)))
        return bits[:count]

    def write_coil(self, coil, enabled, uid=1):
        self._tx = (self._tx + 1) & 0xFFFF
        val = 0xFF00 if enabled else 0x0000
        pkt = struct.pack(">HHHBBHH", self._tx, 0, 6, uid, 0x05, coil, val)
        resp = self._sr(pkt)
        if resp[7] & 0x80:
            raise ValueError("Modbus write exception 0x{:02x}".format(resp[8]))


# ======================================================================
# CLI
# ======================================================================
def parse_args():
    p = argparse.ArgumentParser(
        description="All-in-one Manufacturing Process HMI for 4 PLCs.",
        epilog=(
            "PLCs:\n"
            "  MicroLogix @ .30 (ENIP, B3:1/0-4)\n"
            "  MicroLogix @ .31 (ENIP, B3:0/0-4)\n"
            "  Siemens S7-1200 @ .2  (S7comm, M0.0-4)\n"
            "  Phoenix Contact @ .3  (Modbus, Coils 0-1)\n"
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("--poll-ms", type=int, default=500, help="Polling interval ms (default: 500)")
    p.add_argument("--process-ms", type=int, default=300, help="Process step interval ms (default: 300)")
    p.add_argument("--source-ip", help="Local IP to bind Modbus connection to")
    return p.parse_args()


# ======================================================================
# Main HMI
# ======================================================================
class MfgHMI:
    def __init__(self, root, args):
        self.root = root
        self.poll_ms = args.poll_ms
        self.process_ms = args.process_ms
        self.source_ip = args.source_ip

        # Build connections
        self.conns = {}
        self.conn_status = {}
        for plc in PLCS:
            ip = plc["ip"]
            proto = plc["proto"]
            if proto == "enip":
                self.conns[ip] = EnipConn(ip)
            elif proto == "s7":
                self.conns[ip] = S7Conn(ip)
            elif proto == "modbus":
                self.conns[ip] = ModbusConn(ip, source_ip=self.source_ip)
            self.conn_status[ip] = "Disconnected"

        # State
        self.output_lamps = {}   # (plc_ip, dev_idx) -> canvas lamp id
        self.output_states = {}  # (plc_ip, dev_idx) -> bool
        self.command_states = {} # (plc_ip, dev_idx) -> bool

        self.process_active = False
        self.process_job = None
        self.process_step = 0

        self.root.title("Manufacturing Process HMI")
        self.root.configure(bg=BG)
        self.root.resizable(False, False)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self.build_ui()
        self.connect_all()
        self.poll()

    def build_ui(self):
        # Title
        tk.Label(self.root, text="⚙ Manufacturing Process HMI", font=("Segoe UI", 22, "bold"), bg=BG, fg=TEXT).pack(pady=(16, 2))

        self.status_var = tk.StringVar(value="Initializing...")
        self.status_label = tk.Label(self.root, textvariable=self.status_var, font=("Segoe UI", 10), bg=BG, fg=MUTED)
        self.status_label.pack(pady=(0, 8))

        # Main frame for PLC panels
        main = tk.Frame(self.root, bg=BG)
        main.pack(padx=12, pady=4)

        for col_idx, plc in enumerate(PLCS):
            self.build_plc_panel(main, col_idx, plc)

        # Process controls
        ctrl = tk.Frame(self.root, bg=BG)
        ctrl.pack(pady=(10, 4))

        self.process_btn_var = tk.StringVar(value="▶ Run Process")
        self.process_btn = tk.Button(
            ctrl, textvariable=self.process_btn_var, font=("Segoe UI", 13, "bold"),
            fg="#ffffff", bg=PROCESS_RUN, activebackground="#2ea043",
            activeforeground="#ffffff", relief=tk.FLAT, padx=28, pady=8,
            cursor="hand2", command=self.toggle_process,
        )
        self.process_btn.pack(side=tk.LEFT, padx=8)

        self.speed_var = tk.StringVar(value=str(self.process_ms))
        tk.Label(ctrl, text="Step ms:", font=("Segoe UI", 9), bg=BG, fg=MUTED).pack(side=tk.LEFT, padx=(16, 4))
        tk.Entry(ctrl, textvariable=self.speed_var, width=6, justify="center", font=("Segoe UI", 10)).pack(side=tk.LEFT)

        # Process stage display
        self.stage_var = tk.StringVar(value="Process idle")
        self.stage_label = tk.Label(self.root, textvariable=self.stage_var, font=("Segoe UI", 12, "bold"), bg=BG, fg=MUTED)
        self.stage_label.pack(pady=(6, 2))

        # Process progress bar
        self.progress_canvas = tk.Canvas(self.root, width=700, height=24, bg=PANEL, highlightthickness=0)
        self.progress_canvas.pack(pady=(2, 4))
        self.progress_bg = self.progress_canvas.create_rectangle(2, 2, 698, 22, fill=OFF, outline="")
        self.progress_bar = self.progress_canvas.create_rectangle(2, 2, 2, 22, fill=GREEN, outline="")

        tk.Label(self.root, text="Connects to all 4 PLCs — simulates a running manufacturing process",
                 font=("Segoe UI", 8), bg=BG, fg=MUTED).pack(pady=(4, 12))

    def build_plc_panel(self, parent, col_idx, plc):
        frame = tk.Frame(parent, bg=PANEL, padx=10, pady=10, relief=tk.FLAT, bd=0)
        frame.grid(row=0, column=col_idx, padx=6, pady=4, sticky="n")

        # PLC header
        hdr = tk.Frame(frame, bg=HEADER, padx=8, pady=4)
        hdr.pack(fill="x", pady=(0, 6))
        tk.Label(hdr, text=plc["name"], font=("Segoe UI", 11, "bold"), bg=HEADER, fg=plc["colour"]).pack(side=tk.LEFT)

        status_var = tk.StringVar(value="●")
        status_lbl = tk.Label(hdr, textvariable=status_var, font=("Segoe UI", 10), bg=HEADER, fg=RED)
        status_lbl.pack(side=tk.RIGHT)

        # Store for updating
        if not hasattr(self, "plc_status_widgets"):
            self.plc_status_widgets = {}
        self.plc_status_widgets[plc["ip"]] = (status_var, status_lbl)

        ip_lbl = tk.Label(frame, text=plc["ip"], font=("Segoe UI", 8), bg=PANEL, fg=MUTED)
        ip_lbl.pack(pady=(0, 4))

        # Lights
        for dev in plc["devices"]:
            self.build_light(frame, plc, dev)

    def build_light(self, parent, plc, dev):
        ip = plc["ip"]
        idx = dev["idx"]

        row = tk.Frame(parent, bg=PANEL)
        row.pack(fill="x", pady=2)

        # Label
        if plc["proto"] == "modbus":
            label_text = dev.get("label", "Coil {}".format(idx))
        else:
            label_text = dev.get("output", dev.get("command", ""))
        tk.Label(row, text=label_text, font=("Segoe UI", 9), bg=PANEL, fg=TEXT, width=10, anchor="w").pack(side=tk.LEFT, padx=(0, 4))

        # Lamp
        canvas = tk.Canvas(row, width=32, height=32, bg=PANEL, highlightthickness=0)
        canvas.pack(side=tk.LEFT, padx=2)
        lamp = canvas.create_oval(4, 4, 28, 28, fill=OFF, outline="#555", width=2)

        self.output_lamps[(ip, idx)] = (canvas, lamp)
        self.output_states[(ip, idx)] = False
        self.command_states[(ip, idx)] = False

    def connect_all(self):
        ok = 0
        for plc in PLCS:
            ip = plc["ip"]
            conn = self.conns[ip]
            try:
                conn.connect()
                self.conn_status[ip] = "Connected"
                self.update_plc_indicator(ip, True)
                ok += 1
            except Exception as exc:
                self.conn_status[ip] = "Failed: {}".format(exc)
                self.update_plc_indicator(ip, False)
                conn.close()
        self.status_var.set("{}/{} PLCs connected".format(ok, len(PLCS)))
        self.status_label.configure(fg=GREEN if ok == len(PLCS) else AMBER if ok > 0 else ERR)

    def update_plc_indicator(self, ip, connected):
        if ip in self.plc_status_widgets:
            sv, lbl = self.plc_status_widgets[ip]
            sv.set("●")
            lbl.configure(fg=GREEN if connected else RED)

    def reconnect(self, ip):
        conn = self.conns[ip]
        try:
            conn.connect()
            self.conn_status[ip] = "Connected"
            self.update_plc_indicator(ip, True)
            return True
        except Exception:
            self.update_plc_indicator(ip, False)
            conn.close()
            return False

    def poll(self):
        for plc in PLCS:
            ip = plc["ip"]
            conn = self.conns[ip]
            if not conn.connected:
                self.reconnect(ip)
                continue

            try:
                if plc["proto"] in ("enip", "s7"):
                    for dev in plc["devices"]:
                        key = (ip, dev["idx"])
                        self.output_states[key] = conn.read_bit(dev["output"])
                elif plc["proto"] == "modbus":
                    bits = conn.read_coils(0, len(plc["devices"]))
                    for dev in plc["devices"]:
                        key = (ip, dev["idx"])
                        self.output_states[key] = bits[dev["idx"]]
                self.update_plc_indicator(ip, True)
            except Exception:
                self.update_plc_indicator(ip, False)
                conn.close()

        self.render_lamps()
        self.root.after(self.poll_ms, self.poll)

    def render_lamps(self):
        for (ip, idx), (canvas, lamp) in self.output_lamps.items():
            state = self.output_states.get((ip, idx), False)
            # Find colour for this PLC
            colour = GREEN
            for plc in PLCS:
                if plc["ip"] == ip:
                    colour = plc["colour"]
                    break
            canvas.itemconfig(lamp, fill=colour if state else OFF)

    # ---------- Write helpers ----------
    def write_command(self, plc, dev_idx, state):
        ip = plc["ip"]
        conn = self.conns[ip]
        if not conn.connected:
            return
        dev = plc["devices"][dev_idx]
        try:
            if plc["proto"] in ("enip", "s7"):
                conn.write_bit(dev["command"], state)
            elif plc["proto"] == "modbus":
                conn.write_coil(dev["idx"], state)
            self.command_states[(ip, dev_idx)] = state
        except Exception:
            conn.close()
            self.update_plc_indicator(ip, False)

    def clear_all_commands(self):
        for plc in PLCS:
            for dev in plc["devices"]:
                self.write_command(plc, dev["idx"], False)

    # ---------- Process simulation ----------
    def get_step_ms(self):
        try:
            v = int(self.speed_var.get())
            return max(50, v)
        except ValueError:
            return self.process_ms

    def toggle_process(self):
        if self.process_active:
            self.stop_process()
        else:
            self.start_process()

    def start_process(self):
        self.process_active = True
        self.process_step = 0
        self.process_btn_var.set("■ Stop Process")
        self.process_btn.configure(bg=PROCESS_STOP)
        self.stage_var.set("Process starting...")
        self.stage_label.configure(fg=GREEN)
        self.process_tick()

    def stop_process(self):
        self.process_active = False
        if self.process_job is not None:
            try:
                self.root.after_cancel(self.process_job)
            except Exception:
                pass
            self.process_job = None
        self.process_btn_var.set("▶ Run Process")
        self.process_btn.configure(bg=PROCESS_RUN)
        self.stage_var.set("Process stopped")
        self.stage_label.configure(fg=MUTED)
        self.progress_canvas.coords(self.progress_bar, 2, 2, 2, 22)
        self.clear_all_commands()

    def process_tick(self):
        if not self.process_active:
            return

        # Total outputs across all PLCs
        total_outputs = sum(len(plc["devices"]) for plc in PLCS)  # 5+5+5+2 = 17
        cycle_len = total_outputs * 2 + 4  # forward sweep + reverse sweep + pauses

        step = self.process_step % cycle_len
        phase = self.process_step // cycle_len
        stage_names = [
            "Raw Material Loading",
            "Machining Station A",
            "Machining Station B",
            "Quality Inspection",
            "Assembly",
            "Final Test",
            "Packaging",
            "Shipping",
        ]
        stage_name = stage_names[phase % len(stage_names)]

        # Update progress bar
        progress_frac = (step + 1) / cycle_len
        bar_width = int(696 * progress_frac)
        self.progress_canvas.coords(self.progress_bar, 2, 2, 2 + bar_width, 22)

        # Build a flat list of all (plc, dev_idx) pairs
        all_points = []
        for plc in PLCS:
            for dev in plc["devices"]:
                all_points.append((plc, dev["idx"]))

        # Forward sweep: light one at a time
        if step < total_outputs:
            self.stage_var.set("▶ {} — Step {}/{}".format(stage_name, step + 1, total_outputs))
            self.stage_label.configure(fg=GREEN)
            for i, (plc, didx) in enumerate(all_points):
                self.write_command(plc, didx, i == step)
        # Pause: all on
        elif step < total_outputs + 2:
            self.stage_var.set("● {} — Processing...".format(stage_name))
            self.stage_label.configure(fg=AMBER)
            for plc, didx in all_points:
                self.write_command(plc, didx, True)
        # Reverse sweep
        elif step < total_outputs * 2 + 2:
            rev_step = step - (total_outputs + 2)
            rev_idx = total_outputs - 1 - rev_step
            self.stage_var.set("◀ {} — Return {}/{}".format(stage_name, rev_step + 1, total_outputs))
            self.stage_label.configure(fg=BLUE)
            for i, (plc, didx) in enumerate(all_points):
                self.write_command(plc, didx, i == rev_idx)
        # Pause: all off
        else:
            self.stage_var.set("○ {} — Complete ✓".format(stage_name))
            self.stage_label.configure(fg=TEAL)
            for plc, didx in all_points:
                self.write_command(plc, didx, False)

        self.process_step += 1
        self.process_job = self.root.after(self.get_step_ms(), self.process_tick)

    # ---------- Cleanup ----------
    def close(self):
        if self.process_job is not None:
            try:
                self.root.after_cancel(self.process_job)
            except Exception:
                pass
        self.clear_all_commands()
        for conn in self.conns.values():
            conn.close()
        self.root.destroy()


def main():
    args = parse_args()
    root = tk.Tk()

    missing = []
    if SLCDriver is None:
        missing.append("pycomm3")
    if snap7 is None:
        missing.append("python-snap7")
    if missing:
        messagebox.showwarning(
            "Missing dependencies",
            "Some PLC connections may not work.\n\nMissing:\n" + "\n".join("  • " + m for m in missing) +
            "\n\nInstall with:\npython -m pip install " + " ".join(missing),
        )

    MfgHMI(root, args)
    root.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
# mfg_hmi.py - Tkinter HMI for the 4 PLC trainer setup
# pip install pycomm3 python-snap7

import argparse
import random
import queue
import re
import socket
import struct
import sys
import threading
import time
import tkinter as tk
from collections import deque
from datetime import datetime
from tkinter import messagebox, ttk

from hmi_branding import apply_window_branding, configure_process_branding

# optional deps
try:
    from pycomm3 import SLCDriver
except ImportError:
    SLCDriver = None

try:
    import snap7
    from snap7.util import get_bool, set_bool
    try:
        from snap7.type import Areas, Parameter
    except ImportError:
        from snap7.types import Areas, Parameter
except ImportError:
    snap7 = None
    get_bool = None
    set_bool = None
    Areas = None
    Parameter = None

PLC_CONNECT_TIMEOUT = 0.75
PLC_RETRY_INITIAL_SECONDS = 2.0
PLC_RETRY_MAX_SECONDS = 30.0

# colors
BG       = "#c8ccd0"
PANEL    = "#e6e6e6"
HEADER   = "#2f3b45"
TEXT     = "#111111"
MUTED    = "#4f555a"
OFF      = "#697178"
GREEN    = "#2f7d32"
RED      = "#8b2f2f"
AMBER    = "#9a6a20"
BLUE     = "#2d5f88"
TEAL     = "#2e6f73"
ERR      = "#8b2f2f"
PROCESS_RUN  = "#2f6f3e"
PROCESS_STOP = "#8b2f2f"

# PLC defs
MLGX_30_DEVICES = [
    {"idx": i, "kind": k, "label": label, "input": "I:0/{}".format(i), "command": "B3:1/{}".format(i), "output": "O:0/{}".format(i)}
    for i, k, label in [
        (0,"toggle","Conveyor"),
        (1,"toggle","Entry Eye"),
        (2,"push","Pusher"),
        (3,"push","Gate"),
        (4,"toggle","Part Load"),
    ]
]
MLGX_31_DEVICES = [
    {"idx": i, "kind": k, "label": label, "input": "I:0/{}".format(i), "command": "B3:0/{}".format(i), "output": "O:0/{}".format(i)}
    for i, k, label in [
        (0,"toggle","Spindle"),
        (1,"toggle","Clamp"),
        (2,"push","Feed"),
        (3,"push","Coolant"),
        (4,"toggle","Cycle Done"),
    ]
]
S7_DEVICES = [
    {"idx": i, "kind": k, "label": label, "input": "I0.{}".format(i), "command": "M0.{}".format(i), "output": "Q0.{}".format(i)}
    for i, k, label in [
        (0,"toggle","Arm Extend"),
        (1,"toggle","Gripper"),
        (2,"push","Press"),
        (3,"push","Rotate"),
        (4,"toggle","Assembled"),
    ]
]
MODBUS_DEVICES = [
    {"idx": 0, "kind": "toggle", "label": "Spray"},
    {"idx": 1, "kind": "toggle", "label": "QC Pass"},
]

PLCS = [
    {"name": "MicroLogix .30",   "ip": "192.168.0.30", "proto": "enip",   "devices": MLGX_30_DEVICES, "colour": BLUE},
    {"name": "MicroLogix .31",   "ip": "192.168.0.31", "proto": "enip",   "devices": MLGX_31_DEVICES, "colour": GREEN},
    {"name": "Siemens S7-1200",  "ip": "192.168.0.2",  "proto": "s7",     "devices": S7_DEVICES,      "colour": TEAL},
    {"name": "Phoenix Contact",  "ip": "192.168.0.3",  "proto": "modbus", "devices": MODBUS_DEVICES,  "colour": AMBER},
]


# connection classes

def response_failed(response):
    return (not response) or getattr(response, "error", None)


class EnipConn:
    def __init__(self, host, timeout=PLC_CONNECT_TIMEOUT):
        self.host = host
        self.timeout = timeout
        self.plc = None
        self.lock = threading.RLock()

    @property
    def connected(self):
        return self.plc is not None

    def connect(self):
        with self.lock:
            self.close()
            if SLCDriver is None:
                raise RuntimeError("pycomm3 not installed")
            plc = SLCDriver(self.host)
            plc.socket_timeout = self.timeout
            if not plc.open():
                plc.close()
                raise RuntimeError("EtherNet/IP connection failed")
            self.plc = plc

    def close(self):
        with self.lock:
            plc, self.plc = self.plc, None
            if plc is not None:
                try:
                    plc.close()
                except Exception:
                    pass

    def read_bit(self, addr):
        with self.lock:
            r = self.plc.read(addr)
            if response_failed(r):
                raise RuntimeError("read {} failed: {}".format(addr, r))
            return bool(r.value)

    def write_bit(self, addr, state):
        with self.lock:
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
    def __init__(self, host, rack=0, slot=1, port=102, timeout=PLC_CONNECT_TIMEOUT):
        self.host = host
        self.rack = rack
        self.slot = slot
        self.port = port
        self.timeout = timeout
        self.client = None
        self.lock = threading.RLock()

    @property
    def connected(self):
        if self.client is None:
            return False
        try:
            return bool(self.client.get_connected())
        except Exception:
            return False

    def connect(self):
        with self.lock:
            self.close()
            if snap7 is None:
                raise RuntimeError("python-snap7 not installed")
            client = snap7.client.Client()
            timeout_ms = max(100, int(self.timeout * 1000))
            for parameter in (Parameter.PingTimeout, Parameter.SendTimeout, Parameter.RecvTimeout):
                client.set_param(parameter, timeout_ms)
            try:
                client.connect(self.host, self.rack, self.slot, self.port)
            except TypeError:
                client.connect(self.host, self.rack, self.slot, tcpport=self.port)
            if not client.get_connected():
                client.destroy()
                raise RuntimeError("S7 connection failed")
            self.client = client

    def close(self):
        with self.lock:
            client, self.client = self.client, None
            if client is not None:
                try:
                    client.disconnect()
                except Exception:
                    pass
                try:
                    client.destroy()
                except Exception:
                    pass

    def read_bit(self, addr):
        area, byte_idx, bit_idx = parse_s7_bit_address(addr)
        with self.lock:
            data = self.client.read_area(area, 0, byte_idx, 1)
            return bool(get_bool(data, 0, bit_idx))

    def write_bit(self, addr, state):
        area, byte_idx, bit_idx = parse_s7_bit_address(addr)
        with self.lock:
            data = self.client.read_area(area, 0, byte_idx, 1)
            set_bool(data, 0, bit_idx, bool(state))
            self.client.write_area(area, 0, byte_idx, data)


class ModbusConn:
    def __init__(self, host, port=502, source_ip=None, timeout=PLC_CONNECT_TIMEOUT):
        self.host = host
        self.port = port
        self.source_ip = source_ip
        self.timeout = timeout
        self.sock = None
        self._tx = 0
        self.lock = threading.RLock()

    @property
    def connected(self):
        return self.sock is not None

    def connect(self):
        with self.lock:
            self.close()
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(self.timeout)
            try:
                if self.source_ip:
                    s.bind((self.source_ip, 0))
                s.connect((self.host, self.port))
            except Exception:
                s.close()
                raise
            self.sock = s

    def close(self):
        with self.lock:
            sock, self.sock = self.sock, None
            if sock is not None:
                try:
                    sock.close()
                except Exception:
                    pass

    def _sr(self, pkt):
        self.sock.sendall(pkt)
        return self.sock.recv(4096)

    def read_coils(self, start=0, count=2, uid=1):
        with self.lock:
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
        with self.lock:
            self._tx = (self._tx + 1) & 0xFFFF
            val = 0xFF00 if enabled else 0x0000
            pkt = struct.pack(">HHHBBHH", self._tx, 0, 6, uid, 0x05, coil, val)
            resp = self._sr(pkt)
            if resp[7] & 0x80:
                raise ValueError("Modbus write exception 0x{:02x}".format(resp[8]))


# optional GasPot / Veeder-Root TLS tank inventory client
class GasPotTankClient:
    SOH = b"\x01"
    ETX = b"\x03"

    def __init__(self, host="127.0.0.1", port=10001, timeout=0.8):
        self.host = host
        self.port = port
        self.timeout = timeout

    def send_command(self, cmd):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.settimeout(self.timeout)
            sock.connect((self.host, self.port))
            sock.sendall(self.SOH + cmd.encode("ascii") + b"\n")
            response = b""
            while True:
                try:
                    chunk = sock.recv(4096)
                except socket.timeout:
                    break
                if not chunk:
                    break
                response += chunk
                if self.ETX in chunk or len(response) > 65536:
                    break
        return response.decode("ascii", errors="replace").replace("\x01", "").replace("\x03", "")

    def get_inventory(self):
        raw = self.send_command("I20100")
        tanks = []
        for line in raw.splitlines():
            m = re.match(
                r"^\s*(\d+)\s+(\S+)\s+([\d.]+)\s+([\d.]+)\s+([\d.]+)\s+([\d.]+)\s+([\d.]+)\s+([\d.]+)",
                line,
            )
            if not m:
                continue
            volume = int(float(m.group(3)))
            ullage = int(float(m.group(5)))
            capacity = max(1, volume + ullage)
            tanks.append({
                "tank_id": int(m.group(1)),
                "product": m.group(2),
                "volume": volume,
                "tc_volume": int(float(m.group(4))),
                "ullage": ullage,
                "height": float(m.group(6)),
                "water": float(m.group(7)),
                "temperature": float(m.group(8)),
                "fill_pct": round((volume / capacity) * 100.0, 1),
            })
        return tanks


# cli
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
    p.add_argument("--gaspot-host", default="127.0.0.1", help="GasPot host for tank monitoring (default: 127.0.0.1)")
    p.add_argument("--gaspot-port", type=int, default=10001, help="GasPot TLS port (default: 10001)")
    p.add_argument("--gaspot-poll-ms", type=int, default=10000, help="GasPot tank polling interval ms (default: 10000)")
    p.add_argument("--disable-gaspot", action="store_true", help="Disable optional GasPot tank monitoring")
    return p.parse_args()


# main app
class MfgHMI:
    def __init__(self, root, args):
        self.root = root
        self.poll_ms = args.poll_ms
        self.process_ms = args.process_ms
        self.source_ip = args.source_ip
        self.gaspot_poll_ms = args.gaspot_poll_ms
        self.gaspot_client = None if args.disable_gaspot else GasPotTankClient(args.gaspot_host, args.gaspot_port)
        self.gaspot_next_poll = 0
        self.gaspot_connected = False
        self.gaspot_error = "Not polled yet"
        self.tank_readings = []
        self.tank_widgets = {}


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
        self.command_lamps = {}  # requested command / AUTO heartbeat indicator
        self.output_states = {}  # (plc_ip, dev_idx) -> bool
        self.command_states = {} # (plc_ip, dev_idx) -> bool
        self.status_lamp_states = {}  # mirrors random inactive-PLC heartbeat commands
        self.last_seen = {plc["ip"]: 0.0 for plc in PLCS}
        self.plc_errors = {plc["ip"]: "Not connected" for plc in PLCS}
        self.plc_ready_state = {plc["ip"]: False for plc in PLCS}
        self.plc_fresh_seconds = max(2.0, (self.poll_ms / 1000.0) * 3.0)
        self.comm_results = queue.Queue()
        self.comm_stop_event = threading.Event()
        self.comm_thread = None
        self.next_retry = {plc["ip"]: 0.0 for plc in PLCS}
        self.retry_delay = {plc["ip"]: PLC_RETRY_INITIAL_SECONDS for plc in PLCS}

        self.process_active = False
        self.process_job = None
        self.process_step = 0
        self.process_fault_latched = False
        self.process_fault = ""
        self.operating_mode = "AUTO"
        self.event_log = deque(maxlen=200)
        self.event_tree = None
        self.classic_canvas = None
        self.classic_items = {}

        self.root.title("Widget Manufacturing Control System - Trainer Cell 01")
        apply_window_branding(self.root)
        self.root.configure(bg=BG)
        self.root.resizable(False, False)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self.build_ui()
        self.add_event("INFO", "SYSTEM", "Desktop HMI started in AUTO / stopped state")
        self.connect_all()
        self.poll()

    def build_ui(self):
        # Persistent operator status header.
        header = tk.Frame(self.root, bg=HEADER, padx=14, pady=8)
        header.pack(fill="x")
        title_box = tk.Frame(header, bg=HEADER)
        title_box.pack(side=tk.LEFT)
        tk.Label(title_box, text="TRAINER CELL 01 / OPERATOR STATION", font=("Segoe UI", 7, "bold"), bg=HEADER, fg="#aeb8c0").pack(anchor="w")
        tk.Label(title_box, text="Widget Manufacturing Control System", font=("Segoe UI", 15, "bold"), bg=HEADER, fg="#ffffff").pack(anchor="w")

        self.mode_status_var = tk.StringVar(value="MODE AUTO")
        self.run_status_var = tk.StringVar(value="STOPPED")
        self.ready_status_var = tk.StringVar(value="0/4 READY")
        self.header_status_labels = {}
        for key, variable, colour in (
            ("mode", self.mode_status_var, BLUE),
            ("run", self.run_status_var, AMBER),
            ("ready", self.ready_status_var, AMBER),
        ):
            label = tk.Label(header, textvariable=variable, font=("Consolas", 9, "bold"), bg=colour, fg="#ffffff", padx=10, pady=5, relief=tk.RIDGE, bd=1)
            label.pack(side=tk.RIGHT, padx=3)
            self.header_status_labels[key] = label

        self.status_var = tk.StringVar(value="Initializing...")
        self.status_label = tk.Label(self.root, textvariable=self.status_var, font=("Segoe UI", 10), bg=BG, fg=MUTED)
        self.status_label.pack(pady=(6, 2))
        self.interlock_var = tk.StringVar(value="Evaluating start permissives...")
        self.interlock_label = tk.Label(self.root, textvariable=self.interlock_var, font=("Segoe UI", 9, "bold"), bg="#f2dfad", fg="#3f3213", padx=8, pady=4)
        self.interlock_label.pack(fill="x", padx=12, pady=(0, 4))

        style = ttk.Style()
        try:
            style.theme_use("default")
        except Exception:
            pass
        style.configure("TNotebook", background=BG, borderwidth=0)
        style.configure("TNotebook.Tab", padding=(14, 5), font=("Segoe UI", 9, "bold"))

        tabs = ttk.Notebook(self.root)
        tabs.pack(padx=12, pady=4)

        classic_tab = tk.Frame(tabs, bg=BG)
        detail_tab = tk.Frame(tabs, bg=BG)
        event_tab = tk.Frame(tabs, bg=BG)
        tabs.add(classic_tab, text="Process Overview")
        tabs.add(detail_tab, text="PLC I/O Detail")
        tabs.add(event_tab, text="Events & Alarms")

        # Main frame for PLC panels
        main = tk.Frame(detail_tab, bg=BG)
        main.pack(padx=4, pady=6)

        for col_idx, plc in enumerate(PLCS):
            self.build_plc_panel(main, col_idx, plc)

        self.build_classic_view(classic_tab)
        self.build_event_panel(event_tab)
        tabs.select(classic_tab)

        # Process controls
        ctrl = tk.Frame(self.root, bg=BG)
        ctrl.pack(pady=(10, 4))

        self.auto_btn = tk.Button(ctrl, text="AUTO", font=("Segoe UI", 9, "bold"), width=8, command=lambda: self.set_mode("AUTO"))
        self.auto_btn.pack(side=tk.LEFT, padx=(0, 2))
        self.manual_btn = tk.Button(ctrl, text="MANUAL", font=("Segoe UI", 9, "bold"), width=8, command=lambda: self.set_mode("MANUAL"))
        self.manual_btn.pack(side=tk.LEFT, padx=(0, 10))

        self.process_btn_var = tk.StringVar(value="Start Auto")
        self.process_btn = tk.Button(
            ctrl, textvariable=self.process_btn_var, font=("Segoe UI", 10, "bold"),
            fg="#ffffff", bg=PROCESS_RUN, activebackground=PROCESS_RUN,
            activeforeground="#ffffff", relief=tk.RAISED, bd=2, padx=22, pady=6,
            command=self.toggle_process,
        )
        self.process_btn.pack(side=tk.LEFT, padx=8)

        self.reset_btn = tk.Button(
            ctrl, text="Reset Fault", font=("Segoe UI", 10, "bold"),
            fg="#ffffff", bg=ERR, activebackground=ERR,
            activeforeground="#ffffff", relief=tk.RAISED, bd=2, padx=16, pady=6,
            command=self.reset_fault, state=tk.DISABLED,
        )
        self.reset_btn.pack(side=tk.LEFT, padx=8)

        self.speed_var = tk.StringVar(value=str(self.process_ms))
        tk.Label(ctrl, text="Step ms:", font=("Segoe UI", 9), bg=BG, fg=MUTED).pack(side=tk.LEFT, padx=(16, 4))
        tk.Entry(ctrl, textvariable=self.speed_var, width=6, justify="center", font=("Segoe UI", 10)).pack(side=tk.LEFT)

        # Process stage display
        self.stage_var = tk.StringVar(value="Process idle")
        self.stage_label = tk.Label(self.root, textvariable=self.stage_var, font=("Segoe UI", 12, "bold"), bg=BG, fg=MUTED)
        self.stage_label.pack(pady=(6, 2))

        # Process progress bar
        self.progress_canvas = tk.Canvas(self.root, width=700, height=20, bg=PANEL, highlightthickness=1, highlightbackground="#7a7f84")
        self.progress_canvas.pack(pady=(2, 4))
        self.progress_bg = self.progress_canvas.create_rectangle(2, 2, 698, 18, fill="#b7bcc1", outline="")
        self.progress_bar = self.progress_canvas.create_rectangle(2, 2, 2, 18, fill=GREEN, outline="")

        tk.Label(self.root, text="Derived process values are for training visualization; CMD and FB indications are shown separately.",
                 font=("Segoe UI", 8), bg=BG, fg=MUTED).pack(pady=(4, 12))
        self.update_mode_controls()

    def build_plc_panel(self, parent, col_idx, plc):
        frame = tk.Frame(parent, bg=PANEL, padx=10, pady=10, relief=tk.GROOVE, bd=2)
        frame.grid(row=0, column=col_idx, padx=6, pady=4, sticky="n")

        # PLC header
        hdr = tk.Frame(frame, bg=HEADER, padx=8, pady=4)
        hdr.pack(fill="x", pady=(0, 6))
        tk.Label(hdr, text=plc["name"], font=("Segoe UI", 10, "bold"), bg=HEADER, fg="#f2f2f2").pack(side=tk.LEFT)

        status_var = tk.StringVar(value="OFF")
        status_lbl = tk.Label(hdr, textvariable=status_var, font=("Segoe UI", 8, "bold"), bg=HEADER, fg=RED)
        status_lbl.pack(side=tk.RIGHT)

        # Store for updating
        if not hasattr(self, "plc_status_widgets"):
            self.plc_status_widgets = {}
        health_var = tk.StringVar(value="No verified scan")
        self.plc_status_widgets[plc["ip"]] = (status_var, status_lbl, health_var)

        ip_lbl = tk.Label(frame, text=plc["ip"], font=("Segoe UI", 8), bg=PANEL, fg=MUTED)
        ip_lbl.pack(pady=(0, 1))
        tk.Label(frame, textvariable=health_var, font=("Consolas", 7, "bold"), bg=PANEL, fg=MUTED).pack(pady=(0, 4))
        headings = tk.Frame(frame, bg=PANEL)
        headings.pack(fill="x")
        tk.Label(headings, text="POINT", font=("Segoe UI", 7, "bold"), bg=PANEL, fg=MUTED, width=12, anchor="w").pack(side=tk.LEFT)
        tk.Label(headings, text="CMD", font=("Segoe UI", 7, "bold"), bg=PANEL, fg=MUTED, width=5).pack(side=tk.LEFT)
        tk.Label(headings, text="FB", font=("Segoe UI", 7, "bold"), bg=PANEL, fg=MUTED, width=4).pack(side=tk.LEFT)

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
            label_text = dev.get("label", dev.get("output", dev.get("command", "")))
        address = dev.get("command", "Coil {}".format(idx))
        label_box = tk.Frame(row, bg=PANEL)
        label_box.pack(side=tk.LEFT, padx=(0, 4))
        tk.Label(label_box, text=label_text, font=("Segoe UI", 8, "bold"), bg=PANEL, fg=TEXT, width=12, anchor="w").pack(anchor="w")
        tk.Label(label_box, text=address, font=("Consolas", 6), bg=PANEL, fg=MUTED, width=12, anchor="w").pack(anchor="w")

        command = tk.Button(
            row, text="OFF", width=4, font=("Segoe UI", 7, "bold"),
            bg="#d5d8da", fg=TEXT, relief=tk.RAISED, bd=1,
            command=lambda p=plc, i=idx: self.manual_command(p, i), state=tk.DISABLED,
        )
        command.pack(side=tk.LEFT, padx=2)
        self.command_lamps[(ip, idx)] = command

        # Measured PLC output feedback lamp.
        canvas = tk.Canvas(row, width=28, height=28, bg=PANEL, highlightthickness=0)
        canvas.pack(side=tk.LEFT, padx=2)
        lamp = canvas.create_oval(4, 4, 24, 24, fill=OFF, outline="#555", width=2)

        self.output_lamps[(ip, idx)] = (canvas, lamp)
        self.output_states[(ip, idx)] = False
        self.command_states[(ip, idx)] = False
        self.status_lamp_states[(ip, idx)] = False

    def build_event_panel(self, parent):
        frame = tk.Frame(parent, bg=BG, padx=8, pady=8)
        frame.pack(fill="both", expand=True)
        tk.Label(frame, text="OPERATOR EVENT & ALARM JOURNAL", font=("Segoe UI", 11, "bold"), bg=HEADER, fg="#ffffff", padx=8, pady=6).pack(fill="x")
        columns = ("time", "severity", "source", "message")
        tree = ttk.Treeview(frame, columns=columns, show="headings", height=16)
        tree.heading("time", text="TIME")
        tree.heading("severity", text="SEVERITY")
        tree.heading("source", text="SOURCE")
        tree.heading("message", text="MESSAGE")
        tree.column("time", width=145, anchor="w")
        tree.column("severity", width=75, anchor="center")
        tree.column("source", width=120, anchor="w")
        tree.column("message", width=420, anchor="w")
        tree.tag_configure("ALARM", background="#f2cccc")
        tree.tag_configure("WARNING", background="#f4e6bf")
        tree.pack(fill="both", expand=True)
        self.event_tree = tree

    def add_event(self, severity, source, message):
        event = (datetime.now().astimezone().strftime("%Y-%m-%d %H:%M:%S"), str(severity).upper(), source, message)
        self.event_log.appendleft(event)
        self.refresh_event_tree()

    def refresh_event_tree(self):
        if self.event_tree is None:
            return
        for item in self.event_tree.get_children():
            self.event_tree.delete(item)
        for event in self.event_log:
            self.event_tree.insert("", "end", values=event, tags=(event[1],))

    def build_tank_monitor(self, parent):
        frame = tk.Frame(parent, bg="#071007", padx=10, pady=10)
        frame.pack(padx=8, pady=8, fill="both")

        header = tk.Frame(frame, bg="#071007")
        header.pack(fill="x", pady=(0, 8))
        tk.Label(header, text="GASPOT / TLS-350 TANK MONITOR", font=("Consolas", 13, "bold"), bg="#071007", fg="#39ff14").pack(side=tk.LEFT)
        self.tank_status_var = tk.StringVar(value="GasPot tank monitor: waiting for poll")
        tk.Label(header, textvariable=self.tank_status_var, font=("Consolas", 9), bg="#071007", fg="#7cff6b").pack(side=tk.RIGHT)

        grid = tk.Frame(frame, bg="#071007")
        grid.pack()
        for idx in range(6):
            card = tk.Frame(grid, bg="#0b190b", highlightthickness=1, highlightbackground="#1f6f1f", padx=8, pady=6)
            card.grid(row=0, column=idx, padx=4, pady=4, sticky="n")
            title = tk.StringVar(value="TANK --")
            product = tk.StringVar(value="NO DATA")
            tk.Label(card, textvariable=title, font=("Consolas", 10, "bold"), bg="#0b190b", fg="#39ff14").pack()
            tk.Label(card, textvariable=product, font=("Consolas", 8), bg="#0b190b", fg="#7cff6b").pack()
            gauge = tk.Canvas(card, width=54, height=122, bg="#020802", highlightthickness=1, highlightbackground="#39ff14")
            gauge.pack(pady=5)
            gauge.create_rectangle(14, 8, 40, 114, outline="#39ff14", width=2)
            fill = gauge.create_rectangle(16, 112, 38, 112, fill="#39ff14", outline="")
            pct_text = gauge.create_text(27, 62, text="--%", fill="#c8ffc8", font=("Consolas", 8, "bold"))
            values = tk.StringVar(value="VOL ----\nULL ----\nHGT ----\nH2O ----\nTMP ----")
            tk.Label(card, textvariable=values, justify="left", font=("Consolas", 7), bg="#0b190b", fg="#b5ffad").pack()
            self.tank_widgets[idx] = {"title": title, "product": product, "gauge": gauge, "fill": fill, "pct": pct_text, "values": values}

    def poll_gaspot(self, force=False):
        if self.gaspot_client is None:
            self.gaspot_connected = False
            self.gaspot_error = "Disabled"
            self.update_tank_monitor()
            return
        now_ms = int(time.time() * 1000)
        if not force and now_ms < self.gaspot_next_poll:
            return
        self.gaspot_next_poll = now_ms + max(1000, self.gaspot_poll_ms)
        try:
            tanks = self.gaspot_client.get_inventory()
            self.tank_readings = tanks
            self.gaspot_connected = bool(tanks)
            self.gaspot_error = "OK" if tanks else "No tank rows returned"
        except Exception as exc:
            self.gaspot_connected = False
            self.gaspot_error = str(exc)
        self.update_tank_monitor()

    def update_tank_monitor(self):
        if hasattr(self, "tank_status_var"):
            status = "CONNECTED - {} tanks".format(len(self.tank_readings)) if self.gaspot_connected else "DISCONNECTED - {}".format(self.gaspot_error)
            self.tank_status_var.set(status)
        for idx, widgets in self.tank_widgets.items():
            if idx < len(self.tank_readings):
                t = self.tank_readings[idx]
                pct = max(0.0, min(100.0, float(t.get("fill_pct", 0.0))))
                top = 112 - (pct / 100.0) * 104
                fill_colour = "#39ff14" if t.get("water", 0) <= 1.0 else AMBER if t.get("water", 0) <= 1.5 else RED
                widgets["title"].set("TANK {:02d}".format(t["tank_id"]))
                widgets["product"].set(str(t["product"]))
                widgets["gauge"].coords(widgets["fill"], 16, top, 38, 112)
                widgets["gauge"].itemconfig(widgets["fill"], fill=fill_colour)
                widgets["gauge"].itemconfig(widgets["pct"], text="{:.1f}%".format(pct))
                widgets["values"].set(
                    "VOL {:>5} GAL\nTC  {:>5} GAL\nULL {:>5} GAL\nHGT {:>5.2f} IN\nH2O {:>5.2f} IN\nTMP {:>5.1f} F".format(
                        t["volume"], t["tc_volume"], t["ullage"], t["height"], t["water"], t["temperature"]
                    )
                )
            else:
                widgets["title"].set("TANK --")
                widgets["product"].set("NO DATA")
                widgets["gauge"].coords(widgets["fill"], 16, 112, 38, 112)
                widgets["gauge"].itemconfig(widgets["pct"], text="--%")
                widgets["values"].set("VOL ----\nTC  ----\nULL ----\nHGT ----\nH2O ----\nTMP ----")

    def gaspot_tank(self, index):
        return self.tank_readings[index] if index < len(self.tank_readings) else None

    def build_classic_view(self, parent):
        frame = tk.Frame(parent, bg=BG)
        frame.pack(padx=8, pady=8)

        self.classic_canvas = tk.Canvas(
            frame,
            width=780,
            height=380,
            bg="#d6d1d9",
            highlightthickness=1,
            highlightbackground="#7a7f84",
        )
        self.classic_canvas.pack()

        c = self.classic_canvas
        self.classic_items = {
            "stations": {},
            "pipes": {},
            "valves": {},
            "readouts": {},
            "fills": {},
        }

        # Palette for the industrial single-line mimic.
        bg = "#d6d1d9"
        pipe = "#7c858c"
        process_pipe = "#21a8d8"
        return_pipe = "#b69768"
        vessel_edge = "#4b4f54"
        vessel_dark = "#8c9094"
        vessel_mid = "#bfc2c5"
        vessel_light = "#eeeeee"
        readout = "#004cff"
        valve_off = "#c80000"
        valve_on = "#ff1c1c"

        def pipe_line(name, points, colour=process_pipe, width=7):
            item = c.create_line(*points, fill=colour, width=width, capstyle=tk.ROUND, joinstyle=tk.ROUND)
            self.classic_items["pipes"][name] = {"item": item, "base": colour, "width": width}
            return item

        def readout_text(name, x, y, text, anchor="w"):
            item = c.create_text(x, y, text=text, anchor=anchor, font=("Segoe UI", 8, "bold"), fill=readout)
            self.classic_items["readouts"][name] = item
            return item

        def valve(name, x, y):
            item = c.create_oval(x - 7, y - 7, x + 7, y + 7, fill=valve_off, outline="#5a0000", width=1)
            self.classic_items["valves"][name] = item
            return item

        def status_dot(name, x, y):
            item = c.create_oval(x - 6, y - 6, x + 6, y + 6, fill=OFF, outline="#333", width=1)
            return item

        def striped_vessel(x, y, w, h, label, key, fill_colour="#c7d9e8"):
            # Create an old-school SCADA shaded vessel using vertical bands.
            body = []
            body.append(c.create_rectangle(x, y + 16, x + w, y + h - 16, fill=vessel_mid, outline=vessel_edge, width=2))
            body.append(c.create_oval(x, y, x + w, y + 34, fill=vessel_light, outline=vessel_edge, width=2))
            body.append(c.create_oval(x, y + h - 34, x + w, y + h, fill=vessel_dark, outline=vessel_edge, width=2))
            band_w = max(8, w // 7)
            shades = [vessel_dark, vessel_light, "#dcdcdc", vessel_mid, "#f7f7f7", "#9aa0a5"]
            for i, shade in enumerate(shades):
                bx1 = x + i * band_w
                bx2 = min(x + w, bx1 + band_w)
                body.append(c.create_rectangle(bx1, y + 18, bx2, y + h - 18, fill=shade, outline=""))
            c.create_text(x + w / 2, y - 18, text=label, font=("Segoe UI", 9, "bold"), fill=TEXT)
            fill_h = int(h * 0.42)
            fill_item = c.create_rectangle(x + 8, y + h - 22 - fill_h, x + w - 8, y + h - 22, fill=fill_colour, outline="", stipple="gray50")
            dot = status_dot(key, x + 24, y + 28)
            state = c.create_text(x + w / 2, y + h + 25, text="Ready", font=("Segoe UI", 8, "bold"), fill=readout)
            self.classic_items["stations"][key] = {"body": body, "dot": dot, "state": state}
            self.classic_items["fills"][key] = {"item": fill_item, "x1": x + 8, "x2": x + w - 8, "bottom": y + h - 22, "max_h": h - 44}
            return body

        c.create_rectangle(8, 8, 1032, 512, fill=bg, outline="#a5a0a8")
        c.create_text(520, 26, text="WIDGET CELL - INDUSTRIAL SINGLE-LINE OVERVIEW", font=("Segoe UI", 14, "bold"), fill=TEXT)
        c.create_text(520, 45, text="Accumulator, Process Train, Paint Condenser, QC Return and Cooling", font=("Segoe UI", 8), fill=MUTED)

        # Pipework first, behind equipment.
        pipe_line("mlgx30", (28, 152, 110, 152, 110, 116, 170, 116), process_pipe, 7)
        pipe_line("mlgx31", (255, 172, 345, 172, 345, 142, 390, 142), process_pipe, 7)
        pipe_line("s7", (515, 145, 600, 145, 600, 108, 705, 108, 705, 220, 785, 220), process_pipe, 7)
        pipe_line("phx", (770, 270, 900, 270, 900, 345, 958, 345), process_pipe, 7)
        c.create_line(900, 370, 680, 370, 680, 410, 255, 410, 255, 328, 175, 328, fill=return_pipe, width=4, capstyle=tk.ROUND, joinstyle=tk.ROUND)
        c.create_line(215, 112, 215, 62, 260, 62, 260, 30, fill=pipe, width=5, capstyle=tk.ROUND)
        c.create_line(610, 108, 610, 70, 680, 70, fill=pipe, width=5, capstyle=tk.ROUND)
        c.create_line(730, 125, 815, 125, 815, 82, 895, 82, fill=process_pipe, width=5, capstyle=tk.ROUND)
        c.create_line(820, 170, 955, 170, 955, 110, fill=return_pipe, width=3, capstyle=tk.ROUND)

        # Feed/blower and valves.
        c.create_rectangle(22, 136, 72, 152, fill="#9da2a6", outline="#595f64")
        c.create_text(28, 124, text="Blowdown", anchor="w", font=("Segoe UI", 7), fill=TEXT)
        valve("feed", 78, 345)

        # Main process equipment.
        striped_vessel(120, 112, 145, 235, "Accumulator", "load", "#c7d9e8")
        c.create_line(265, 220, 305, 220, fill=pipe, width=4, capstyle=tk.ROUND)
        readout_text("load_pct", 184, 202, "81 %", anchor="center")
        readout_text("load_temp", 220, 253, "200 F")
        readout_text("load_flow", 136, 333, "276 gpm")
        readout_text("load_pressure", 38, 292, "1.8 dPsi")

        striped_vessel(385, 135, 130, 175, "Wash / Machine", "machine", "#b9cfe7")
        c.create_rectangle(378, 125, 522, 138, fill="#5d6268", outline="")
        valve("machine", 410, 322)
        valve("drain", 510, 322)
        readout_text("mach_vac", 385, 104, "16.3 inHg")
        readout_text("mach_temp", 475, 104, "179 F")
        readout_text("mach_flow", 432, 338, "181 gpm", anchor="center")

        striped_vessel(565, 112, 145, 190, "Assembly Effect", "assemble", "#d8dde7")
        c.create_line(638, 302, 638, 356, fill=pipe, width=5, capstyle=tk.ROUND)
        valve("assembly", 638, 358)
        readout_text("asm_vac", 560, 86, "24.5 inHg")
        readout_text("asm_temp", 650, 86, "133 F")
        readout_text("asm_level", 603, 282, "60 %")

        striped_vessel(790, 74, 95, 245, "Paint Condenser", "qc", "#c7d9e8")
        c.create_rectangle(802, 91, 873, 150, fill="#ffffff", outline="", stipple="gray25")
        readout_text("paint_flow", 778, 342, "235 gpm")

        # Secondary condensers and cooling.
        c.create_text(928, 115, text="Sec Chg", font=("Segoe UI", 7), fill=TEXT)
        striped_vessel(905, 126, 48, 140, "", "sec", "#d8dde7")
        readout_text("sec_temp", 880, 150, "105 F")
        readout_text("sec_vac", 878, 168, "-24.1 inHg")
        c.create_text(995, 115, text="InterCnd", font=("Segoe UI", 7), fill=TEXT)
        striped_vessel(975, 126, 36, 135, "", "inter", "#d8dde7")
        c.create_text(1048, 115, text="AfterCnd", font=("Segoe UI", 7), fill=TEXT)
        striped_vessel(1028, 126, 36, 135, "", "after", "#d8dde7")
        c.create_line(955, 285, 1075, 285, fill=process_pipe, width=5, capstyle=tk.ROUND)
        readout_text("qc_temp", 985, 302, "103 F")
        readout_text("qc_out", 1042, 302, "106 F")

        c.create_text(945, 365, text="Cooling Towers", font=("Segoe UI", 8, "bold"), fill=TEXT)
        c.create_polygon(895, 382, 990, 382, 972, 455, 912, 455, fill="#9da2a6", outline="#404850", width=2)
        c.create_rectangle(918, 346, 948, 385, fill=vessel_mid, outline=vessel_edge)
        c.create_rectangle(955, 346, 985, 385, fill=vessel_mid, outline=vessel_edge)
        valve("cool_a", 925, 385)
        valve("cool_b", 970, 385)
        c.create_rectangle(990, 442, 1025, 472, fill="#0fa6d7", outline="#404850")
        readout_text("cool_temp", 1030, 434, "82 F")
        readout_text("cool_pct", 1008, 485, "90%", anchor="center")

        # Bottom WIP tanks and recycle.
        c.create_text(165, 435, text="WIP Tank M10", font=("Segoe UI", 7), fill=TEXT)
        c.create_rectangle(130, 445, 205, 500, fill="#8e8542", outline="#595f2e")
        c.create_rectangle(145, 445, 165, 500, fill="#d0c27c", outline="")
        readout_text("wip_m10", 155, 463, "76 %")
        c.create_text(270, 435, text="WIP Tank R2W", font=("Segoe UI", 7), fill=TEXT)
        c.create_rectangle(235, 445, 315, 500, fill="#8e8542", outline="#595f2e")
        c.create_rectangle(252, 445, 275, 500, fill="#d0c27c", outline="")
        readout_text("wip_r2w", 260, 463, "75 %")
        valve("return", 315, 410)

        # Dotted control lines and small instrument panels.
        c.create_line(215, 118, 215, 240, 360, 240, fill="#7a7f84", width=1, dash=(3, 4))
        c.create_line(638, 112, 638, 62, 820, 62, fill="#7a7f84", width=1, dash=(3, 4))
        c.create_line(930, 126, 930, 78, 1040, 78, fill="#7a7f84", width=1, dash=(3, 4))
        c.create_rectangle(30, 270, 98, 316, fill="#eef0e8", outline="#9aa1a8")
        c.create_rectangle(498, 350, 588, 394, fill="#eef0e8", outline="#9aa1a8")
        readout_text("mid_rate", 510, 368, "189 gpm")
        readout_text("mid_temp", 510, 385, "171 F")
        c.create_rectangle(792, 330, 888, 378, fill="#eef0e8", outline="#9aa1a8")
        readout_text("prod_rate", 806, 350, "74 gpm")
        readout_text("prod_vac", 806, 368, "-20.5 inHg")

        self.classic_items["part"] = c.create_rectangle(165, 332, 194, 353, fill="#b9a064", outline="#5f5030", width=2, state="hidden")
        self.classic_items["part_positions"] = [(165, 332, 194, 353), (418, 290, 447, 311), (625, 278, 654, 299), (830, 252, 859, 273)]
        self.classic_items["part_label"] = c.create_text(520, 476, text="No part in process", font=("Segoe UI", 11, "bold"), fill=MUTED)
        self.classic_items["summary"] = c.create_text(520, 498, text="Built: 0", font=("Segoe UI", 9), fill=MUTED)
        self.classic_items["reject_note"] = c.create_text(520, 44, text="", font=("Segoe UI", 8, "bold"), fill=RED)

        # Draw in a roomy process-coordinate space, then scale to fit the desktop HMI window.
        self.classic_scale = 0.70
        c.scale("all", 0, 0, self.classic_scale, self.classic_scale)
        c.configure(width=780, height=380, scrollregion=(0, 0, 780, 380))

    def update_classic_view(self):
        if self.classic_canvas is None:
            return

        c = self.classic_canvas
        scale = getattr(self, "classic_scale", 1.0)
        station_keys = ["load", "machine", "assemble", "qc"]
        active_station = None
        phase_name = "Idle"
        progress = max(0, min(100, self.process_step))

        if self.process_active and getattr(self, "_widget_plan", None) and self._widget_phase < len(self._widget_plan):
            phase = self._widget_plan[self._widget_phase]
            active_station = phase["station"]
            phase_name = phase["name"]

        # Station equipment, fill levels, and status dots.
        for idx, key in enumerate(station_keys):
            items = self.classic_items.get("stations", {}).get(key)
            if not items:
                continue
            plc = PLCS[idx]
            outputs = [self.output_states.get((plc["ip"], dev["idx"]), False) for dev in plc["devices"]]
            heartbeats = [self.status_lamp_states.get((plc["ip"], dev["idx"]), False) for dev in plc["devices"]]
            station_on = any(outputs)
            heartbeat_on = any(heartbeats) and plc["name"] not in self.unavailable_plcs()
            is_active = active_station == idx
            outline = BLUE if is_active else AMBER if station_on or heartbeat_on else "#4b4f54"
            dot_fill = plc["colour"] if (is_active or station_on or heartbeat_on) else OFF
            state_text = "RUNNING" if is_active else "OUTPUT" if station_on else "HEARTBEAT" if heartbeat_on else "READY"
            state_fill = BLUE if is_active else AMBER if station_on or heartbeat_on else MUTED

            for item in items.get("body", []):
                try:
                    c.itemconfig(item, outline=outline)
                except tk.TclError:
                    pass
            c.itemconfig(items["dot"], fill=dot_fill)
            c.itemconfig(items["state"], text=state_text, fill=state_fill)

            fill_def = self.classic_items.get("fills", {}).get(key)
            if fill_def:
                tank = self.gaspot_tank(idx)
                if tank:
                    level = max(0.08, min(0.95, float(tank.get("fill_pct", 0.0)) / 100.0))
                else:
                    base = 0.68 if is_active else 0.45 if station_on else 0.28
                    level = max(0.18, min(0.88, base + outputs.count(True) * 0.04))
                h = fill_def["max_h"] * level
                c.coords(
                    fill_def["item"],
                    fill_def["x1"] * scale,
                    (fill_def["bottom"] - h) * scale,
                    fill_def["x2"] * scale,
                    fill_def["bottom"] * scale,
                )
                c.itemconfig(fill_def["item"], stipple="gray25" if is_active or station_on else "gray50")

        # Pipe highlighting, matching active transfer progress.
        for idx, key in enumerate(["mlgx30", "mlgx31", "s7", "phx"]):
            pipe_def = self.classic_items.get("pipes", {}).get(key)
            if not pipe_def:
                continue
            is_active_pipe = self.process_active and active_station is not None and active_station >= idx
            c.itemconfig(
                pipe_def["item"],
                fill=BLUE if is_active_pipe else pipe_def["base"],
                dash=(12, 6) if is_active_pipe else "",
                width=pipe_def["width"] + (1 if is_active_pipe else 0),
            )

        valve_states = {
            "feed": self.output_states.get((PLCS[0]["ip"], 0), False),
            "machine": self.output_states.get((PLCS[1]["ip"], 0), False),
            "drain": self.output_states.get((PLCS[1]["ip"], 3), False),
            "assembly": self.output_states.get((PLCS[2]["ip"], 2), False),
            "cool_a": self.output_states.get((PLCS[3]["ip"], 0), False),
            "cool_b": self.output_states.get((PLCS[3]["ip"], 1), False),
            "return": self.process_active and active_station is not None and active_station >= 2,
        }
        for name, state in valve_states.items():
            item = self.classic_items.get("valves", {}).get(name)
            if item:
                c.itemconfig(item, fill="#ff1c1c" if state else "#c80000", outline="#300000" if state else "#5a0000")

        # Live-looking readouts tied to process and output activity.
        def set_readout(name, value):
            item = self.classic_items.get("readouts", {}).get(name)
            if item:
                c.itemconfig(item, text=value)

        def output_count(plc_idx):
            plc = PLCS[plc_idx]
            return sum(1 for dev in plc["devices"] if self.output_states.get((plc["ip"], dev["idx"]), False))

        set_readout("load_pct", "{} %".format(int(68 + progress * 0.22)))
        set_readout("load_temp", "{} F".format(198 + (active_station + 1 if active_station is not None else 0)))
        set_readout("load_flow", "{} gpm".format(220 + int(progress * 1.4) if self.process_active else 0))
        set_readout("load_pressure", "{:.1f} dPsi".format(1.4 + progress / 120.0))
        set_readout("mach_vac", "{:.1f} inHg".format(14.8 + output_count(1) * 0.8))
        set_readout("mach_temp", "{} F".format(176 + ((active_station or 0) * 2) if self.process_active else 175))
        set_readout("mach_flow", "{} gpm".format(140 + int(progress * 0.9) if self.process_active else 15))
        set_readout("asm_vac", "{:.1f} inHg".format(22.5 + output_count(2) * 0.5))
        set_readout("asm_temp", "{} F".format(131 + (active_station or 0) if self.process_active else 133))
        set_readout("asm_level", "{} %".format(min(80, 42 + int(progress * 0.35)) if self.process_active else 40))
        set_readout("paint_flow", "{} gpm".format(185 + int(progress * 0.7) if self.process_active else 74))
        set_readout("sec_temp", "{} F".format(100 + ((active_station or 0) * 2) if self.process_active else 95))
        set_readout("sec_vac", "{} inHg".format("-24.1" if self.process_active else "-20.5"))
        set_readout("qc_temp", "{} F".format(103 if self.process_active else 82))
        set_readout("qc_out", "{} F".format(106 if self.process_active else 82))
        set_readout("cool_temp", "{} F".format(82 if self.process_active else 74))
        set_readout("cool_pct", "{}%".format(90 if self.process_active else 30))
        set_readout("mid_rate", "{} gpm".format(160 + int(progress * 0.8) if self.process_active else 0))
        set_readout("mid_temp", "{} F".format(171 + ((active_station or 0) * 4) if self.process_active else 185))
        set_readout("prod_rate", "{} gpm".format(74 + int(progress * 0.2) if self.process_active else 0))
        set_readout("prod_vac", "{} inHg".format("-20.5" if self.process_active else "0.0"))

        # If GasPot is available, use its real/simulated tank inventory for the tank readouts.
        tank0 = self.gaspot_tank(0)
        tank1 = self.gaspot_tank(1)
        tank2 = self.gaspot_tank(2)
        tank3 = self.gaspot_tank(3)
        if tank0:
            set_readout("load_pct", "{:.1f} %".format(tank0["fill_pct"]))
            set_readout("load_temp", "{:.1f} F".format(tank0["temperature"]))
            set_readout("load_flow", "T{} {}".format(tank0["tank_id"], tank0["product"]))
            set_readout("load_pressure", "H2O {:.2f} IN".format(tank0["water"]))
        if tank1:
            set_readout("wip_m10", "T{} {} {:.1f}%".format(tank1["tank_id"], tank1["product"], tank1["fill_pct"]))
        if tank2:
            set_readout("wip_r2w", "T{} {} {:.1f}%".format(tank2["tank_id"], tank2["product"], tank2["fill_pct"]))
        if tank3:
            set_readout("prod_rate", "T{} {}".format(tank3["tank_id"], tank3["product"]))
            set_readout("prod_vac", "H2O {:.2f} IN".format(tank3["water"]))

        if active_station is not None:
            coords = self.classic_items["part_positions"][active_station]
            scaled_coords = tuple(v * scale for v in coords)
            c.coords(self.classic_items["part"], *scaled_coords)
            c.itemconfig(self.classic_items["part"], state="normal")
            label = "Widget {} - {}".format(getattr(self, "widget_serial", 0), phase_name)
        else:
            c.itemconfig(self.classic_items["part"], state="hidden")
            label = "No part in process"

        c.itemconfig(self.classic_items["part_label"], text=label, fill=TEXT if self.process_active else MUTED)
        c.itemconfig(self.classic_items["summary"], text="Built: {}".format(getattr(self, "widget_count", 0)))
        note = "QC recheck path active for this widget" if self.process_active and getattr(self, "_widget_defect", False) else ""
        c.itemconfig(self.classic_items["reject_note"], text=note)

    def connect_all(self):
        self.status_var.set("NO PLCs ONLINE - reconnecting in background")
        self.status_label.configure(fg=AMBER)
        if self.comm_thread is None or not self.comm_thread.is_alive():
            self.comm_stop_event.clear()
            self.comm_thread = threading.Thread(target=self.communication_loop, name="plc-communications", daemon=True)
            self.comm_thread.start()

    def update_plc_indicator(self, ip, connected):
        if ip in self.plc_status_widgets:
            sv, lbl, health_var = self.plc_status_widgets[ip]
            sv.set("READY" if connected else "NOT READY")
            lbl.configure(fg=GREEN if connected else RED)
            last_seen = self.last_seen.get(ip, 0.0)
            if connected and last_seen > 0:
                health_var.set("Verified scan {:>4} ms old".format(int((time.monotonic() - last_seen) * 1000)))
            else:
                health_var.set(self.plc_errors.get(ip, "No verified scan")[:34])

    def communication_loop(self):
        """Perform all PLC network work away from the Tkinter UI thread."""
        while not self.comm_stop_event.is_set():
            cycle_started = time.monotonic()
            for plc in PLCS:
                if self.comm_stop_event.is_set():
                    break
                ip = plc["ip"]
                conn = self.conns[ip]
                if not conn.connected:
                    now = time.monotonic()
                    if now < self.next_retry[ip]:
                        continue
                    try:
                        conn.connect()
                        # A reconnected PLC is not ready until commands are
                        # cleared and a subsequent feedback read succeeds.
                        for dev in plc["devices"]:
                            if plc["proto"] in ("enip", "s7"):
                                conn.write_bit(dev["command"], False)
                            else:
                                conn.write_coil(dev["idx"], False)
                        self.retry_delay[ip] = PLC_RETRY_INITIAL_SECONDS
                        self.next_retry[ip] = 0.0
                        self.comm_results.put(("connected", ip, None))
                    except Exception as exc:
                        conn.close()
                        delay = self.retry_delay[ip]
                        self.next_retry[ip] = time.monotonic() + delay
                        self.retry_delay[ip] = min(PLC_RETRY_MAX_SECONDS, delay * 2.0)
                        self.comm_results.put(("offline", ip, str(exc), delay))
                    continue

                try:
                    if plc["proto"] in ("enip", "s7"):
                        outputs = [conn.read_bit(dev["output"]) for dev in plc["devices"]]
                    else:
                        outputs = conn.read_coils(0, len(plc["devices"]))
                    self.comm_results.put(("scan", ip, outputs))
                except Exception as exc:
                    conn.close()
                    delay = self.retry_delay[ip]
                    self.next_retry[ip] = time.monotonic() + delay
                    self.retry_delay[ip] = min(PLC_RETRY_MAX_SECONDS, delay * 2.0)
                    self.comm_results.put(("offline", ip, str(exc), delay))

            elapsed = time.monotonic() - cycle_started
            self.comm_stop_event.wait(max(0.05, (self.poll_ms / 1000.0) - elapsed))

    def apply_communication_results(self):
        """Apply worker results and update widgets only from the UI thread."""
        while True:
            try:
                result = self.comm_results.get_nowait()
            except queue.Empty:
                break
            kind, ip = result[0], result[1]
            plc = next(p for p in PLCS if p["ip"] == ip)
            if kind == "connected":
                for dev in plc["devices"]:
                    self.command_states[(ip, dev["idx"])] = False
                self.last_seen[ip] = 0.0
                self.plc_errors[ip] = "Connected; waiting for fresh read"
                self.conn_status[ip] = "Waiting for read"
                self.plc_ready_state[ip] = False
                self.update_plc_indicator(ip, False)
            elif kind == "scan":
                was_ready = self.plc_ready_state.get(ip, False)
                for dev, output in zip(plc["devices"], result[2]):
                    self.output_states[(ip, dev["idx"])] = bool(output)
                self.last_seen[ip] = time.monotonic()
                self.plc_errors[ip] = ""
                self.conn_status[ip] = "Connected"
                self.plc_ready_state[ip] = True
                self.update_plc_indicator(ip, True)
                if not was_ready:
                    self.add_event("INFO", plc["name"], "Verified communications restored")
            elif kind == "offline":
                was_ready = self.plc_ready_state.get(ip, False)
                self.last_seen[ip] = 0.0
                self.plc_errors[ip] = "Offline; retry in {:.0f}s".format(result[3])
                self.conn_status[ip] = "Offline"
                self.plc_ready_state[ip] = False
                self.update_plc_indicator(ip, False)
                if was_ready:
                    self.add_event("WARNING", plc["name"], "Communications lost: {}".format(result[2]))
                    if self.process_active:
                        self.latch_process_fault("Lost communication with {}".format(plc["name"]))

    def unavailable_plcs(self):
        now = time.monotonic()
        return [
            plc["name"]
            for plc in PLCS
            if self.last_seen.get(plc["ip"], 0.0) <= 0.0
            or now - self.last_seen[plc["ip"]] > self.plc_fresh_seconds
        ]

    def update_connection_summary(self):
        missing = self.unavailable_plcs()
        ready = len(PLCS) - len(missing)
        self.mode_status_var.set("MODE {}".format(self.operating_mode))
        self.ready_status_var.set("NO PLCs ONLINE" if ready == 0 else "{}/{} READY".format(ready, len(PLCS)))
        self.header_status_labels["mode"].configure(bg=BLUE if self.operating_mode == "AUTO" else HEADER)
        self.header_status_labels["ready"].configure(bg=GREEN if ready == len(PLCS) else AMBER if ready else ERR)
        if self.process_fault_latched:
            self.status_var.set("FAULT LATCHED - restore PLCs, inspect equipment, then Reset Fault")
            self.status_label.configure(fg=ERR)
            self.run_status_var.set("FAULT")
            self.header_status_labels["run"].configure(bg=ERR)
            self.interlock_var.set("START INHIBITED: reset the latched fault")
            self.interlock_label.configure(bg="#f2cccc", fg="#681515")
        elif self.process_active:
            self.status_var.set("Automatic sequence running / {}/{} PLCs verified".format(ready, len(PLCS)))
            self.status_label.configure(fg=GREEN)
            self.run_status_var.set("RUNNING")
            self.header_status_labels["run"].configure(bg=GREEN)
            self.interlock_var.set("AUTO SEQUENCE OWNS PROCESS COMMANDS")
            self.interlock_label.configure(bg="#d5e6d8", fg="#214728")
        else:
            self.status_var.set(
                "NO PLCs ONLINE - reconnecting in background"
                if ready == 0 else "{}/{} PLCs ready (verified reads)".format(ready, len(PLCS))
            )
            self.status_label.configure(fg=GREEN if ready == len(PLCS) else AMBER if ready else ERR)
            self.run_status_var.set("STOPPED")
            self.header_status_labels["run"].configure(bg=AMBER)
            reasons = []
            if self.operating_mode != "AUTO":
                reasons.append("select AUTO mode")
            if missing:
                reasons.append("restore verified communications: {}".format(", ".join(missing)))
            if reasons:
                self.interlock_var.set("START INHIBITED: " + " / ".join(reasons))
                self.interlock_label.configure(bg="#f2dfad", fg="#3f3213")
            else:
                self.interlock_var.set("START PERMISSIVES SATISFIED")
                self.interlock_label.configure(bg="#d5e6d8", fg="#214728")
        self.update_mode_controls()

    def poll(self):
        self.apply_communication_results()
        self.poll_gaspot()
        self.update_connection_summary()
        self.render_lamps()
        self.root.after(self.poll_ms, self.poll)

    def render_lamps(self):
        unavailable_ips = {
            plc["ip"]
            for plc in PLCS
            if plc["name"] in self.unavailable_plcs()
        }
        for (ip, idx), (canvas, lamp) in self.output_lamps.items():
            status_on = self.status_lamp_states.get((ip, idx), False) and ip not in unavailable_ips
            feedback_on = self.output_states.get((ip, idx), False)
            command_on = self.command_states.get((ip, idx), False)
            canvas.itemconfig(lamp, fill=GREEN if feedback_on else OFF)
            command = self.command_lamps.get((ip, idx))
            if command is not None:
                command.configure(
                    text="ON" if command_on else "OFF",
                    bg=AMBER if status_on else BLUE if command_on else "#d5d8da",
                    fg="#ffffff" if command_on else TEXT,
                )
                manual_enabled = (
                    self.operating_mode == "MANUAL"
                    and not self.process_active
                    and ip not in unavailable_ips
                )
                command.configure(state=tk.NORMAL if manual_enabled else tk.DISABLED)
        self.update_classic_view()

    def update_mode_controls(self):
        auto_selected = self.operating_mode == "AUTO"
        self.auto_btn.configure(
            bg=HEADER if auto_selected else "#d5d8da",
            fg="#ffffff" if auto_selected else TEXT,
            state=tk.DISABLED if self.process_active else tk.NORMAL,
        )
        self.manual_btn.configure(
            bg=HEADER if not auto_selected else "#d5d8da",
            fg="#ffffff" if not auto_selected else TEXT,
            state=tk.DISABLED if self.process_active else tk.NORMAL,
        )
        if self.process_fault_latched:
            self.process_btn.configure(state=tk.DISABLED, bg=ERR)
            self.process_btn_var.set("Fault Latched")
        elif self.process_active:
            self.process_btn.configure(state=tk.NORMAL, bg=PROCESS_STOP)
            self.process_btn_var.set("Stop Auto")
        else:
            ready = not self.unavailable_plcs()
            can_start = auto_selected and ready
            self.process_btn.configure(state=tk.NORMAL if can_start else tk.DISABLED, bg=PROCESS_RUN)
            self.process_btn_var.set("Start Auto" if auto_selected else "AUTO Required")

    def set_mode(self, mode):
        requested = str(mode).upper()
        if requested not in ("AUTO", "MANUAL") or requested == self.operating_mode:
            return False
        if self.process_active:
            messagebox.showwarning("Mode change blocked", "Stop the automatic sequence before changing mode.")
            return False
        failures = self.clear_all_commands()
        if failures:
            self.latch_process_fault("Mode-change OFF command failed for {}".format(", ".join(failures)))
            return False
        self.operating_mode = requested
        self.stage_var.set("{} mode - stopped".format(requested.title()))
        self.add_event("ACTION", "OPERATOR", "Operating mode changed to {}".format(requested))
        self.update_connection_summary()
        self.render_lamps()
        return True

    def manual_command(self, plc, dev_idx):
        key = (plc["ip"], dev_idx)
        target = not self.command_states.get(key, False)
        if self.process_active:
            messagebox.showwarning("Command blocked", "Manual commands are disabled while AUTO is running.")
            return False
        if target and self.operating_mode != "MANUAL":
            messagebox.showwarning("Command blocked", "Select MANUAL mode before issuing an ON command.")
            return False
        if target and self.process_fault_latched:
            messagebox.showwarning("Command blocked", "Reset the latched fault before issuing an ON command.")
            return False
        if target and plc["name"] in self.unavailable_plcs():
            messagebox.showwarning("Command blocked", "{} does not have fresh verified feedback.".format(plc["name"]))
            return False
        if not self.write_command(plc, dev_idx, target):
            messagebox.showerror("Command failed", "Unable to write {}.".format(plc["devices"][dev_idx].get("label", "point")))
            return False
        point = plc["devices"][dev_idx].get("label", "Point {}".format(dev_idx))
        self.add_event("ACTION", plc["name"], "Manual command {} -> {}".format(point, "ON" if target else "OFF"))
        self.render_lamps()
        return True

    # ---------- Write helpers ----------
    def write_command(self, plc, dev_idx, state, fault_on_failure=False):
        ip = plc["ip"]
        conn = self.conns[ip]
        if not conn.connected:
            if fault_on_failure and self.process_active:
                self.latch_process_fault("Command write failed for {} (offline)".format(plc["name"]))
            return False
        dev = plc["devices"][dev_idx]
        try:
            if plc["proto"] in ("enip", "s7"):
                conn.write_bit(dev["command"], state)
            elif plc["proto"] == "modbus":
                conn.write_coil(dev["idx"], state)
            self.command_states[(ip, dev_idx)] = state
            return True
        except Exception as exc:
            self.last_seen[ip] = 0.0
            self.plc_errors[ip] = str(exc)
            conn.close()
            self.update_plc_indicator(ip, False)
            if fault_on_failure and self.process_active:
                self.latch_process_fault("Command write failed for {}".format(plc["name"]))
            return False

    def clear_all_commands(self):
        failures = []
        for plc in PLCS:
            for dev in plc["devices"]:
                if not self.write_command(plc, dev["idx"], False):
                    failures.append(plc["name"])
                    break
        return sorted(set(failures))

    def cancel_process_job(self):
        if self.process_job is not None:
            try:
                self.root.after_cancel(self.process_job)
            except Exception:
                pass
            self.process_job = None

    def latch_process_fault(self, message):
        if self.process_fault_latched:
            return
        self.process_active = False
        self.cancel_process_job()
        self.process_fault_latched = True
        self.process_fault = message
        self.process_step = 0
        self._widget_plan = []
        self._idle_lamps = {}
        self._idle_next_blink = {}
        for key in self.status_lamp_states:
            self.status_lamp_states[key] = False
        self.clear_all_commands()
        self.add_event("ALARM", "PROCESS", message)
        self.process_btn_var.set("Fault Latched")
        self.process_btn.configure(bg=ERR, state=tk.DISABLED)
        self.reset_btn.configure(state=tk.NORMAL)
        self.stage_var.set("FAULT - {}".format(message))
        self.stage_label.configure(fg=ERR)
        self.update_process_bar(0)
        self.update_connection_summary()
        self.update_classic_view()

    def reset_fault(self):
        if self.process_active:
            messagebox.showwarning("Process running", "Stop the process before resetting a fault.")
            return False
        missing = self.unavailable_plcs()
        if missing:
            messagebox.showerror("Reset blocked", "PLCs not ready:\n" + "\n".join(missing))
            return False
        failures = self.clear_all_commands()
        if failures:
            messagebox.showerror("Reset blocked", "OFF command failed for:\n" + "\n".join(failures))
            return False
        self.process_fault_latched = False
        self.process_fault = ""
        self.process_btn_var.set("Start Auto")
        self.process_btn.configure(bg=PROCESS_RUN, state=tk.NORMAL)
        self.reset_btn.configure(state=tk.DISABLED)
        self.stage_var.set("{} mode - fault reset".format(self.operating_mode.title()))
        self.stage_label.configure(fg=MUTED)
        self.add_event("ACTION", "OPERATOR", "Latched process fault reset")
        self.update_connection_summary()
        self.update_classic_view()
        return True

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
        if self.operating_mode != "AUTO":
            messagebox.showerror("Start blocked", "Select AUTO mode before starting the sequence.")
            return False
        if self.process_fault_latched:
            messagebox.showerror("Start blocked", "Reset the latched fault before starting.")
            return False
        missing = self.unavailable_plcs()
        if missing:
            messagebox.showerror("Start blocked", "PLCs not ready:\n" + "\n".join(missing))
            return False
        failures = self.clear_all_commands()
        if failures:
            self.latch_process_fault("Unable to verify OFF commands for {}".format(", ".join(failures)))
            return False
        self.process_active = True
        self.process_step = 0
        self.widget_count = 0
        self.widget_serial = random.randint(1000, 8999)
        self._widget_plan = []
        self._widget_phase = 0
        self._phase_tick = 0
        self._idle_lamps = {}
        self._idle_next_blink = {plc["ip"]: random.randint(2, 6) for plc in PLCS}
        self.process_btn_var.set("Stop Auto")
        self.process_btn.configure(bg=PROCESS_STOP)
        self.stage_var.set("Starting widget line...")
        self.stage_label.configure(fg=GREEN)
        self.add_event("ACTION", "OPERATOR", "Start command accepted")
        self.update_classic_view()
        self.process_tick()
        return True

    def stop_process(self):
        self.process_active = False
        self.cancel_process_job()
        self.process_btn_var.set("Start Auto")
        self.process_btn.configure(bg=PROCESS_RUN)
        self.stage_var.set("Auto sequence stopped")
        self.stage_label.configure(fg=MUTED)
        self.update_process_bar(0)
        self._widget_plan = []
        self._idle_lamps = {}
        self._idle_next_blink = {}
        for key in self.status_lamp_states:
            self.status_lamp_states[key] = False
        failures = self.clear_all_commands()
        if failures:
            self.latch_process_fault("Stop command failed for {}".format(", ".join(failures)))
            return False
        self.add_event("ACTION", "OPERATOR", "Stop command accepted; all commands cleared")
        self.update_connection_summary()
        self.update_classic_view()
        return True

    def build_widget_plan(self):
        self.widget_serial += 1
        self._widget_defect = random.random() < 0.12
        self._widget_plan = [
            {"name": "Load blank", "station": 0, "ticks": random.randint(9, 14)},
            {"name": "Cut and face", "station": 1, "ticks": random.randint(13, 20)},
            {"name": "Install insert", "station": 2, "ticks": random.randint(12, 18)},
            {"name": "Paint and inspect", "station": 3, "ticks": random.randint(10, 16)},
        ]
        if self._widget_defect:
            self._widget_plan.append({"name": "QC recheck", "station": 3, "ticks": random.randint(5, 8)})
        self._widget_phase = 0
        self._phase_tick = 0
        self._idle_lamps = {}
        self._idle_next_blink = {plc["ip"]: random.randint(2, 6) for plc in PLCS}

    def write_station_states(self, plc, states):
        if not self.process_active:
            return False
        for dev in plc["devices"]:
            idx = dev["idx"]
            state = bool(states[idx]) if idx < len(states) else False
            key = (plc["ip"], idx)
            if self.command_states.get(key) != state:
                if not self.write_command(plc, idx, state, fault_on_failure=True):
                    return False
        return True

    def noisy_states(self, states, on_drop=0.04, off_blip=0.015):
        noisy = []
        for state in states:
            if state:
                noisy.append(random.random() > on_drop)
            else:
                noisy.append(random.random() < off_blip)
        return noisy

    def idle_station_states(self, plc):
        """Advance one visible physical light heartbeat for an inactive PLC."""
        states = [False] * len(plc["devices"])
        ip = plc["ip"]
        active = False
        for dev in plc["devices"]:
            key = (ip, dev["idx"])
            ticks_left = self._idle_lamps.get(key, 0)
            if ticks_left > 0:
                states[dev["idx"]] = True
                self._idle_lamps[key] = ticks_left - 1
                active = True

        if not active:
            ticks_until_blink = self._idle_next_blink.get(ip, random.randint(2, 6))
            if ticks_until_blink <= 0:
                dev = random.choice(plc["devices"])
                idx = dev["idx"]
                key = (ip, idx)
                step_ms = self.get_step_ms()
                # Keep each pulse visible through at least two display polls.
                pulse_ticks = max(2, ((self.poll_ms * 2) + step_ms - 1) // step_ms)
                states[idx] = True
                self._idle_lamps[key] = pulse_ticks - 1
                self._idle_next_blink[ip] = random.randint(4, 10)
            else:
                self._idle_next_blink[ip] = ticks_until_blink - 1

        for dev in plc["devices"]:
            key = (ip, dev["idx"])
            self.status_lamp_states[key] = states[dev["idx"]]
        return states

    def active_station_states(self, station_idx, tick, total):
        phase = tick / max(1, total - 1)
        pulse2 = (tick % 2) == 0
        pulse3 = (tick % 3) == 0

        if station_idx == 0:
            # conveyor/load: motor, entry eye, pusher, transfer gate, part present
            states = [
                phase < 0.88,
                0.10 < phase < 0.42 or (pulse3 and phase < 0.70),
                0.42 < phase < 0.62 and pulse2,
                phase > 0.68,
                0.25 < phase < 0.92,
            ]
            return self.noisy_states(states, on_drop=0.03, off_blip=0.02)

        if station_idx == 1:
            # machining: spindle, clamp, feed pulse, coolant, cycle complete
            states = [
                0.15 < phase < 0.88,
                0.08 < phase < 0.94,
                0.28 < phase < 0.72 and (pulse2 or random.random() < 0.35),
                0.35 < phase < 0.82 and random.random() > 0.25,
                phase > 0.86,
            ]
            return self.noisy_states(states, on_drop=0.05, off_blip=0.01)

        if station_idx == 2:
            # assembly: arm extend, gripper, press/weld, rotate, done
            cycle = tick % 6
            states = [
                cycle in (0, 1, 2),
                cycle in (1, 2, 3),
                0.35 < phase < 0.72 and pulse3,
                0.52 < phase < 0.88 and cycle in (3, 4),
                phase > 0.84,
            ]
            return self.noisy_states(states, on_drop=0.04, off_blip=0.012)

        # paint/qc: spray, pass/recheck lamp
        if getattr(self, "_widget_defect", False) and self._widget_phase >= 4:
            states = [pulse2, phase > 0.70 and pulse2]
        else:
            states = [phase < 0.72 and random.random() > 0.18, phase > 0.78]
        return self.noisy_states(states, on_drop=0.04, off_blip=0.015)

    def drive_process_outputs(self, active_station, tick, total):
        for idx, plc in enumerate(PLCS):
            if idx == active_station:
                states = self.active_station_states(idx, tick, total)
                for dev in plc["devices"]:
                    self.status_lamp_states[(plc["ip"], dev["idx"])] = False
                if not self.write_station_states(plc, states):
                    return False
            else:
                # Keep the other PLCs visibly alive with speed-based random
                # light commands, matching the original trainer behavior.
                states = self.idle_station_states(plc)
                if not self.write_station_states(plc, states):
                    return False
        return True

    def update_process_bar(self, progress):
        progress = max(0, min(100, progress))
        self.process_step = progress
        bar_width = int(696 * (progress / 100.0))
        self.progress_canvas.coords(self.progress_bar, 2, 2, 2 + bar_width, 18)

    def process_tick(self):
        if not self.process_active:
            return

        missing = self.unavailable_plcs()
        if missing:
            self.latch_process_fault("PLCs not ready during process: {}".format(", ".join(missing)))
            return

        if not self._widget_plan:
            self.build_widget_plan()

        if self._widget_phase >= len(self._widget_plan):
            self.widget_count += 1
            self.stage_var.set("Widget {} complete - total built {}".format(self.widget_serial, self.widget_count))
            self.stage_label.configure(fg=TEAL if not self._widget_defect else AMBER)
            self.update_process_bar(100)
            failures = self.clear_all_commands()
            if failures:
                self.latch_process_fault("Changeover OFF command failed for {}".format(", ".join(failures)))
                return
            self._widget_plan = []
            self.update_classic_view()
            step_ms = self.get_step_ms()
            self.process_job = self.root.after(random.randint(step_ms * 3, step_ms * 6), self.process_tick)
            return

        phase = self._widget_plan[self._widget_phase]
        total = phase["ticks"]
        active_station = phase["station"]

        if not self.drive_process_outputs(active_station, self._phase_tick, total):
            return
        self.update_classic_view()

        phase_progress = self._phase_tick / max(1, total)
        progress = int(((self._widget_phase + phase_progress) / len(self._widget_plan)) * 100)
        self.update_process_bar(progress)

        station_name = PLCS[active_station]["name"]
        self.stage_var.set("Widget {} - {} ({})".format(self.widget_serial, phase["name"], station_name))
        self.stage_label.configure(fg=AMBER if self._widget_defect and phase["name"].startswith("QC") else GREEN)

        self._phase_tick += 1
        if self._phase_tick >= total:
            if not self.write_station_states(PLCS[active_station], [False] * len(PLCS[active_station]["devices"])):
                return
            self._widget_phase += 1
            self._phase_tick = 0

        step_ms = self.get_step_ms()
        jitter = int(step_ms * random.uniform(0.25, 0.60))
        delay = random.randint(max(50, step_ms - jitter), step_ms + jitter)
        if random.random() < 0.08:
            delay += random.randint(100, 450)
        self.process_job = self.root.after(delay, self.process_tick)

    # ---------- Cleanup ----------
    def close(self):
        self.process_active = False
        self.cancel_process_job()
        self.comm_stop_event.set()
        self.clear_all_commands()
        for conn in self.conns.values():
            conn.close()
        if self.comm_thread is not None and self.comm_thread.is_alive():
            self.comm_thread.join(timeout=1.0)
        self.root.destroy()


def main():
    args = parse_args()
    configure_process_branding()
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

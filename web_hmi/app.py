#!/usr/bin/env python3
"""
web_hmi/app.py

Web-based Manufacturing Process HMI.
Flask backend that talks to all 4 PLCs and serves a browser dashboard
with animated SVG conveyor belts, drill press, assembly, paint, and QC stations.

Run:
    cd trainer-hmi/web_hmi
    python app.py
    # Open http://localhost:5000

Requires:
    pip install flask pycomm3 python-snap7
"""

import argparse
import json
import random
import socket
import struct
import sys
import threading
import time

from flask import Flask, jsonify, render_template, request

# ---------- Optional PLC imports ----------
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
    get_bool = set_bool = Areas = None


# ======================================================================
# PLC Connection Classes
# ======================================================================

def response_failed(r):
    return (not r) or getattr(r, "error", None)


class EnipConn:
    def __init__(self, host):
        self.host = host
        self.plc = None
        self.lock = threading.Lock()

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
        if self.plc:
            try:
                self.plc.close()
            except Exception:
                pass
        self.plc = None

    def read_bit(self, addr):
        with self.lock:
            r = self.plc.read(addr)
            if response_failed(r):
                raise RuntimeError("read {} failed".format(addr))
            return bool(r.value)

    def write_bit(self, addr, state):
        with self.lock:
            r = self.plc.write((addr, 1 if state else 0))
            if response_failed(r):
                raise RuntimeError("write {} failed".format(addr))


def parse_s7_addr(address):
    address = address.strip().upper()
    prefix = address[0]
    byte_t, bit_t = address[1:].split(".", 1)
    byte_idx, bit_idx = int(byte_t), int(bit_t)
    if prefix in ("I", "E"):
        area = Areas.PE
    elif prefix in ("Q", "A"):
        area = Areas.PA
    else:
        area = Areas.MK
    return area, byte_idx, bit_idx


class S7Conn:
    def __init__(self, host, rack=0, slot=1, port=102):
        self.host = host
        self.rack = rack
        self.slot = slot
        self.port = port
        self.client = None
        self.lock = threading.Lock()

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
        if self.client:
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
        area, by, bi = parse_s7_addr(addr)
        with self.lock:
            data = self.client.read_area(area, 0, by, 1)
            return bool(get_bool(data, 0, bi))

    def write_bit(self, addr, state):
        area, by, bi = parse_s7_addr(addr)
        with self.lock:
            data = self.client.read_area(area, 0, by, 1)
            set_bool(data, 0, bi, bool(state))
            self.client.write_area(area, 0, by, data)


class ModbusConn:
    def __init__(self, host, port=502, source_ip=None, timeout=3.0):
        self.host = host
        self.port = port
        self.source_ip = source_ip
        self.timeout = timeout
        self.sock = None
        self._tx = 0
        self.lock = threading.Lock()

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
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
        self.sock = None

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
                raise ValueError("Modbus exception")
            bc = resp[8]
            bits = []
            for b in resp[9:9 + bc]:
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
                raise ValueError("Modbus write exception")


# ======================================================================
# PLC Definitions
# ======================================================================
PLCS = [
    {
        "id": "mlgx30", "name": "MicroLogix .30", "ip": "192.168.0.30",
        "proto": "enip", "station": "Conveyor In",
        "devices": [
            {"idx": i, "kind": k, "input": "I:0/{}".format(i), "command": "B3:1/{}".format(i), "output": "O:0/{}".format(i)}
            for i, k in [(0, "toggle"), (1, "toggle"), (2, "push"), (3, "push"), (4, "toggle")]
        ],
    },
    {
        "id": "mlgx31", "name": "MicroLogix .31", "ip": "192.168.0.31",
        "proto": "enip", "station": "Drill Press",
        "devices": [
            {"idx": i, "kind": k, "input": "I:0/{}".format(i), "command": "B3:0/{}".format(i), "output": "O:0/{}".format(i)}
            for i, k in [(0, "toggle"), (1, "toggle"), (2, "push"), (3, "push"), (4, "toggle")]
        ],
    },
    {
        "id": "s7", "name": "Siemens S7-1200", "ip": "192.168.0.2",
        "proto": "s7", "station": "Assembly",
        "devices": [
            {"idx": i, "kind": k, "input": "I0.{}".format(i), "command": "M0.{}".format(i), "output": "Q0.{}".format(i)}
            for i, k in [(0, "toggle"), (1, "toggle"), (2, "push"), (3, "push"), (4, "toggle")]
        ],
    },
    {
        "id": "phx", "name": "Phoenix Contact", "ip": "192.168.0.3",
        "proto": "modbus", "station": "Paint & QC",
        "devices": [
            {"idx": 0, "kind": "toggle", "label": "Coil 0"},
            {"idx": 1, "kind": "toggle", "label": "Coil 1"},
        ],
    },
]


# ======================================================================
# Global state
# ======================================================================
conns = {}
state_lock = threading.Lock()
plc_states = {}  # {plc_id: {"connected": bool, "outputs": [bool,...], "commands": [bool,...]}}
process_running = False
process_stage = "Idle"
process_step = 0


def init_connections():
    for plc in PLCS:
        pid = plc["id"]
        ip = plc["ip"]
        n = len(plc["devices"])
        plc_states[pid] = {"connected": False, "outputs": [False] * n, "commands": [False] * n}
        if plc["proto"] == "enip":
            conns[pid] = EnipConn(ip)
        elif plc["proto"] == "s7":
            conns[pid] = S7Conn(ip)
        elif plc["proto"] == "modbus":
            conns[pid] = ModbusConn(ip)


def try_connect(pid):
    conn = conns[pid]
    try:
        conn.connect()
        with state_lock:
            plc_states[pid]["connected"] = True
        return True
    except Exception:
        conn.close()
        with state_lock:
            plc_states[pid]["connected"] = False
        return False


def poll_loop():
    """Background thread that polls all PLCs."""
    while True:
        for plc in PLCS:
            pid = plc["id"]
            conn = conns[pid]
            if not conn.connected:
                try_connect(pid)
                continue
            try:
                outputs = []
                if plc["proto"] in ("enip", "s7"):
                    for dev in plc["devices"]:
                        outputs.append(conn.read_bit(dev["output"]))
                elif plc["proto"] == "modbus":
                    outputs = conn.read_coils(0, len(plc["devices"]))
                with state_lock:
                    plc_states[pid]["outputs"] = outputs
                    plc_states[pid]["connected"] = True
            except Exception:
                conn.close()
                with state_lock:
                    plc_states[pid]["connected"] = False
        time.sleep(0.5)


def write_command(pid, dev_idx, state):
    plc = next(p for p in PLCS if p["id"] == pid)
    conn = conns[pid]
    if not conn.connected:
        return False
    dev = plc["devices"][dev_idx]
    try:
        if plc["proto"] in ("enip", "s7"):
            conn.write_bit(dev["command"], state)
        elif plc["proto"] == "modbus":
            conn.write_coil(dev["idx"], state)
        with state_lock:
            plc_states[pid]["commands"][dev_idx] = state
        return True
    except Exception:
        conn.close()
        with state_lock:
            plc_states[pid]["connected"] = False
        return False


def clear_all():
    for plc in PLCS:
        for dev in plc["devices"]:
            write_command(plc["id"], dev["idx"], False)


def process_loop():
    """Background thread — realistic manufacturing process simulation.

    Runs through production stages where each station activates in sequence
    with realistic timing:
      1. Conveyor loads material (mlgx30 lights cascade)
      2. Drill press operates (mlgx31 lights pulse)
      3. Robot assembles (s7 lights animate)
      4. Paint & QC (phx coils cycle)
    Between full cycles there's a brief changeover pause.
    Within each stage, individual outputs fire with random organic timing
    to look like real equipment operating.
    """
    global process_running, process_stage, process_step

    # Station definitions
    stations = [
        {"pid": "mlgx30", "name": "Conveyor In",       "devs": 5},
        {"pid": "mlgx31", "name": "Drill Press",       "devs": 5},
        {"pid": "s7",     "name": "Assembly Robot",     "devs": 5},
        {"pid": "phx",    "name": "Paint & QC",        "devs": 2},
    ]

    production_runs = [
        "Batch #A-{:04d} — Aluminum Brackets",
        "Batch #B-{:04d} — Steel Housings",
        "Batch #C-{:04d} — Copper Connectors",
        "Batch #D-{:04d} — Titanium Shafts",
    ]

    batch_num = random.randint(1000, 9999)
    run_count = 0

    while process_running:
        batch_label = production_runs[run_count % len(production_runs)].format(batch_num)
        run_count += 1
        batch_num += 1

        # --- Stage 1: Conveyor loads raw material ---
        process_stage = "📦 {} — Loading raw material".format(batch_label)
        for tick in range(12):
            if not process_running: break
            process_step = int(tick / 12 * 25)
            # Cascade conveyor lights one by one
            for i in range(5):
                write_command("mlgx30", i, i <= tick % 5)
            time.sleep(random.uniform(0.2, 0.4))
        # Conveyor running steady
        for i in range(5):
            write_command("mlgx30", i, True)
        time.sleep(0.5)
        # Turn off conveyor
        for i in range(5):
            write_command("mlgx30", i, False)

        if not process_running: break

        # --- Stage 2: Drill Press machining ---
        process_stage = "🔩 {} — Drilling holes".format(batch_label)
        for tick in range(15):
            if not process_running: break
            process_step = 25 + int(tick / 15 * 25)
            # Simulate drill: motor on, clamp, drill down, retract
            write_command("mlgx31", 0, True)  # Motor
            write_command("mlgx31", 1, tick % 4 < 2)  # Clamp cycling
            write_command("mlgx31", 2, tick % 3 == 0)  # Drill pulse
            write_command("mlgx31", 3, tick % 5 < 3)  # Coolant
            write_command("mlgx31", 4, random.random() < 0.4)  # Status
            time.sleep(random.uniform(0.2, 0.35))
        for i in range(5):
            write_command("mlgx31", i, False)

        if not process_running: break

        # --- Stage 3: Robot Assembly ---
        process_stage = "🤖 {} — Assembling components".format(batch_label)
        for tick in range(18):
            if not process_running: break
            process_step = 50 + int(tick / 18 * 25)
            # Simulate: pick, move, place, weld, inspect
            cycle_pos = tick % 6
            write_command("s7", 0, cycle_pos < 3)     # Arm extend/retract
            write_command("s7", 1, cycle_pos in [1,2]) # Gripper
            write_command("s7", 2, cycle_pos == 3)     # Weld
            write_command("s7", 3, cycle_pos == 4)     # Rotate
            write_command("s7", 4, cycle_pos == 5)     # Done signal
            time.sleep(random.uniform(0.15, 0.35))
        for i in range(5):
            write_command("s7", i, False)

        if not process_running: break

        # --- Stage 4: Paint & Quality Check ---
        process_stage = "🎨 {} — Paint & quality check".format(batch_label)
        for tick in range(10):
            if not process_running: break
            process_step = 75 + int(tick / 10 * 20)
            write_command("phx", 0, tick % 3 != 2)  # Spray on/off
            write_command("phx", 1, tick > 6)        # QC pass light
            time.sleep(random.uniform(0.25, 0.5))
        write_command("phx", 0, False)
        write_command("phx", 1, True)  # QC pass
        time.sleep(0.6)
        write_command("phx", 1, False)

        if not process_running: break

        # --- Changeover ---
        process_stage = "✅ {} — Complete! Changeover...".format(batch_label)
        process_step = 100
        time.sleep(1.5)
        process_step = 0

    process_stage = "Stopped"
    process_step = 0
    clear_all()


# ======================================================================
# Flask App
# ======================================================================
app = Flask(__name__)


@app.route("/")
def index():
    return render_template("index.html", plcs=PLCS)


@app.route("/api/state")
def api_state():
    with state_lock:
        data = {
            "plcs": {},
            "process_running": process_running,
            "process_stage": process_stage,
            "process_step": process_step,
        }
        for plc in PLCS:
            pid = plc["id"]
            data["plcs"][pid] = {
                "name": plc["name"],
                "ip": plc["ip"],
                "station": plc["station"],
                "connected": plc_states[pid]["connected"],
                "outputs": plc_states[pid]["outputs"],
                "commands": plc_states[pid]["commands"],
                "devices": len(plc["devices"]),
            }
    return jsonify(data)


@app.route("/api/command", methods=["POST"])
def api_command():
    body = request.json
    pid = body.get("plc_id")
    dev_idx = body.get("dev_idx")
    state = body.get("state", False)
    ok = write_command(pid, int(dev_idx), bool(state))
    return jsonify({"ok": ok})


@app.route("/api/process", methods=["POST"])
def api_process():
    global process_running
    body = request.json
    action = body.get("action", "toggle")

    if action == "start" and not process_running:
        process_running = True
        t = threading.Thread(target=process_loop, daemon=True)
        t.start()
        return jsonify({"running": True})
    elif action == "stop":
        process_running = False
        return jsonify({"running": False})
    else:
        return jsonify({"running": process_running})


def main():
    parser = argparse.ArgumentParser(description="Web-based Manufacturing Process HMI")
    parser.add_argument("--port", type=int, default=5000, help="Web server port (default: 5000)")
    parser.add_argument("--host", default="0.0.0.0", help="Web server bind address (default: 0.0.0.0)")
    args = parser.parse_args()

    init_connections()

    # Start background poll thread
    poll_thread = threading.Thread(target=poll_loop, daemon=True)
    poll_thread.start()

    print("=" * 60)
    print("  Manufacturing Process HMI")
    print("  Open http://localhost:{} in your browser".format(args.port))
    print("=" * 60)

    app.run(host=args.host, port=args.port, debug=False)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# web_hmi/app.py - Flask backend for the web-based HMI
# pip install flask pycomm3 python-snap7

import argparse
import json
import os
import random
import re
import socket
import struct
import sys
import threading
import time
from collections import deque
from datetime import datetime

from flask import Flask, jsonify, render_template, request

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
    get_bool = set_bool = Areas = None
    Parameter = None

PLC_CONNECT_TIMEOUT = 0.75
PLC_RETRY_INITIAL_SECONDS = 2.0
PLC_RETRY_MAX_SECONDS = 30.0


# PLC connection classes

def response_failed(r):
    return (not r) or getattr(r, "error", None)


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
            if plc:
                try:
                    plc.close()
                except Exception:
                    pass

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
            if client:
                try:
                    client.disconnect()
                except Exception:
                    pass
                try:
                    client.destroy()
                except Exception:
                    pass

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
            if sock:
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


# PLC defs
PLCS = [
    {
        "id": "mlgx30", "name": "MicroLogix .30", "ip": "192.168.0.30",
        "proto": "enip", "station": "Conveyor In",
        "devices": [
            {"idx": i, "kind": k, "label": label, "input": "I:0/{}".format(i), "command": "B3:1/{}".format(i), "output": "O:0/{}".format(i)}
            for i, k, label in [
                (0, "toggle", "Conveyor"), (1, "toggle", "Entry Eye"),
                (2, "push", "Pusher"), (3, "push", "Transfer Gate"),
                (4, "toggle", "Part Present"),
            ]
        ],
    },
    {
        "id": "mlgx31", "name": "MicroLogix .31", "ip": "192.168.0.31",
        "proto": "enip", "station": "Drill Press",
        "devices": [
            {"idx": i, "kind": k, "label": label, "input": "I:0/{}".format(i), "command": "B3:0/{}".format(i), "output": "O:0/{}".format(i)}
            for i, k, label in [
                (0, "toggle", "Spindle"), (1, "toggle", "Clamp"),
                (2, "push", "Feed"), (3, "push", "Coolant"),
                (4, "toggle", "Cycle Complete"),
            ]
        ],
    },
    {
        "id": "s7", "name": "Siemens S7-1200", "ip": "192.168.0.2",
        "proto": "s7", "station": "Assembly",
        "devices": [
            {"idx": i, "kind": k, "label": label, "input": "I0.{}".format(i), "command": "M0.{}".format(i), "output": "Q0.{}".format(i)}
            for i, k, label in [
                (0, "toggle", "Arm Extend"), (1, "toggle", "Gripper"),
                (2, "push", "Press"), (3, "push", "Rotate"),
                (4, "toggle", "Assembly Complete"),
            ]
        ],
    },
    {
        "id": "phx", "name": "Phoenix Contact", "ip": "192.168.0.3",
        "proto": "modbus", "station": "Paint & QC",
        "devices": [
            {"idx": 0, "kind": "toggle", "label": "Spray"},
            {"idx": 1, "kind": "toggle", "label": "QC Pass"},
        ],
    },
]


# global state
conns = {}
state_lock = threading.Lock()
process_control_lock = threading.Lock()
event_lock = threading.Lock()
plc_states = {}  # {plc_id: {"connected": bool, "outputs": [bool,...], "commands": [bool,...]}}
process_running = False
process_stage = "Idle"
process_step = 0
process_speed_ms = 300
process_fault_latched = False
process_fault = ""
operating_mode = "AUTO"
process_stop_event = threading.Event()
PLC_FRESH_SECONDS = 2.0
gaspot_client = None
gaspot_poll_seconds = 10.0
gaspot_next_poll = 0.0
gaspot_connected = False
gaspot_error = "Not polled yet"
tank_readings = []
event_log = deque(maxlen=200)
event_sequence = 0


def add_event(severity, source, message):
    """Append a concise operator event without blocking PLC state access."""
    global event_sequence
    with event_lock:
        event_sequence += 1
        event_log.appendleft({
            "id": event_sequence,
            "timestamp": datetime.now().astimezone().isoformat(timespec="seconds"),
            "severity": str(severity).upper(),
            "source": source,
            "message": message,
        })


def recent_events(limit=40):
    with event_lock:
        return list(event_log)[:limit]


def get_process_speed_ms():
    with state_lock:
        return max(50, int(process_speed_ms))


def scaled_sleep(min_factor=1.0, max_factor=None):
    if max_factor is None:
        max_factor = min_factor
    delay = random.uniform(min_factor, max_factor) * (get_process_speed_ms() / 1000.0)
    process_stop_event.wait(max(0.05, delay))


def init_connections(source_ip=None):
    for plc in PLCS:
        pid = plc["id"]
        ip = plc["ip"]
        n = len(plc["devices"])
        plc_states[pid] = {
            "connected": False,
            "outputs": [False] * n,
            "commands": [False] * n,
            "status_lights": [False] * n,
            "last_seen": 0.0,
            "error": "Not connected",
            "event_status": "UNKNOWN",
            "next_retry": 0.0,
            "retry_delay": PLC_RETRY_INITIAL_SECONDS,
        }
        if plc["proto"] == "enip":
            conns[pid] = EnipConn(ip)
        elif plc["proto"] == "s7":
            conns[pid] = S7Conn(ip)
        elif plc["proto"] == "modbus":
            conns[pid] = ModbusConn(ip, source_ip=source_ip)


def try_connect(pid):
    plc = next(p for p in PLCS if p["id"] == pid)
    conn = conns[pid]
    try:
        conn.connect()
        # Reconnect into a known command state before declaring the PLC ready.
        for dev in plc["devices"]:
            if plc["proto"] in ("enip", "s7"):
                conn.write_bit(dev["command"], False)
            else:
                conn.write_coil(dev["idx"], False)
        with state_lock:
            plc_states[pid]["commands"] = [False] * len(plc["devices"])
            plc_states[pid]["connected"] = False
            plc_states[pid]["last_seen"] = 0.0
            plc_states[pid]["error"] = "Connected; waiting for fresh read"
            plc_states[pid]["next_retry"] = 0.0
            plc_states[pid]["retry_delay"] = PLC_RETRY_INITIAL_SECONDS
        return True
    except Exception as exc:
        conn.close()
        with state_lock:
            previous_status = plc_states[pid]["event_status"]
            delay = plc_states[pid]["retry_delay"]
            plc_states[pid]["connected"] = False
            plc_states[pid]["last_seen"] = 0.0
            plc_states[pid]["error"] = "Offline; retry in {:.0f}s".format(delay)
            plc_states[pid]["event_status"] = "OFFLINE"
            plc_states[pid]["next_retry"] = time.monotonic() + delay
            plc_states[pid]["retry_delay"] = min(PLC_RETRY_MAX_SECONDS, delay * 2.0)
        if previous_status != "OFFLINE":
            add_event("WARNING", plc["name"], "Connection failed: {}".format(exc))
        return False


def unavailable_plcs():
    """Return PLC names that do not have a recent successful read."""
    now = time.monotonic()
    with state_lock:
        return [
            plc["name"]
            for plc in PLCS
            if not plc_states[plc["id"]]["connected"]
            or now - plc_states[plc["id"]]["last_seen"] > PLC_FRESH_SECONDS
        ]


def latch_process_fault(message):
    """Stop process sequencing and latch a fault until an operator reset."""
    global process_running, process_stage, process_step, process_fault_latched, process_fault
    with state_lock:
        if process_fault_latched:
            return
        process_running = False
        process_step = 0
        process_fault_latched = True
        if not process_fault:
            process_fault = message
        process_stage = "FAULT - {}".format(process_fault)
    process_stop_event.set()
    add_event("ALARM", "PROCESS", message)


def poll_gaspot(force=False):
    global gaspot_next_poll, gaspot_connected, gaspot_error, tank_readings
    if gaspot_client is None:
        gaspot_connected = False
        gaspot_error = "Disabled"
        tank_readings = []
        return
    now = time.time()
    if not force and now < gaspot_next_poll:
        return
    gaspot_next_poll = now + max(1.0, gaspot_poll_seconds)
    try:
        tanks = gaspot_client.get_inventory()
        with state_lock:
            tank_readings = tanks
            gaspot_connected = bool(tanks)
            gaspot_error = "OK" if tanks else "No tank rows returned"
    except Exception as exc:
        with state_lock:
            gaspot_connected = False
            gaspot_error = str(exc)


def poll_loop():
    while True:
        poll_gaspot()
        retry_pid = None
        for plc in PLCS:
            pid = plc["id"]
            conn = conns[pid]
            if not conn.connected:
                with state_lock:
                    retry_due = time.monotonic() >= plc_states[pid]["next_retry"]
                if retry_pid is None and retry_due:
                    retry_pid = pid
                continue
            try:
                outputs = []
                if plc["proto"] in ("enip", "s7"):
                    for dev in plc["devices"]:
                        outputs.append(conn.read_bit(dev["output"]))
                elif plc["proto"] == "modbus":
                    outputs = conn.read_coils(0, len(plc["devices"]))
                with state_lock:
                    previous_status = plc_states[pid]["event_status"]
                    plc_states[pid]["outputs"] = outputs
                    plc_states[pid]["connected"] = True
                    plc_states[pid]["last_seen"] = time.monotonic()
                    plc_states[pid]["error"] = ""
                    plc_states[pid]["event_status"] = "READY"
                    plc_states[pid]["retry_delay"] = PLC_RETRY_INITIAL_SECONDS
                if previous_status != "READY":
                    add_event("INFO", plc["name"], "Verified communications restored")
            except Exception as exc:
                conn.close()
                with state_lock:
                    previous_status = plc_states[pid]["event_status"]
                    delay = plc_states[pid]["retry_delay"]
                    plc_states[pid]["connected"] = False
                    plc_states[pid]["last_seen"] = 0.0
                    plc_states[pid]["error"] = "Offline; retry in {:.0f}s".format(delay)
                    plc_states[pid]["event_status"] = "OFFLINE"
                    plc_states[pid]["next_retry"] = time.monotonic() + delay
                    plc_states[pid]["retry_delay"] = min(PLC_RETRY_MAX_SECONDS, delay * 2.0)
                if previous_status != "OFFLINE":
                    add_event("WARNING", plc["name"], "Communications lost: {}".format(exc))
                if process_running:
                    latch_process_fault("Lost communication with {}".format(plc["name"]))
                    threading.Thread(target=clear_all, daemon=True).start()
        # Limit reconnect work to one offline PLC per cycle. This keeps scans
        # of healthy PLCs responsive even when several addresses are absent.
        if retry_pid is not None:
            try_connect(retry_pid)
        # Communications polling is safety-related and must not slow down with process speed.
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
    except Exception as exc:
        conn.close()
        with state_lock:
            plc_states[pid]["connected"] = False
            plc_states[pid]["last_seen"] = 0.0
            plc_states[pid]["error"] = str(exc)
        return False


def clear_all():
    failures = []
    for plc in PLCS:
        for dev in plc["devices"]:
            if not write_command(plc["id"], dev["idx"], False):
                failures.append(plc["name"])
                break
    return sorted(set(failures))


def process_write(pid, dev_idx, state):
    """Write a process command; any failed write immediately faults the cycle."""
    if not process_running:
        return False
    if write_command(pid, dev_idx, state):
        return True
    plc = next(p for p in PLCS if p["id"] == pid)
    latch_process_fault("Command write failed for {}".format(plc["name"]))
    threading.Thread(target=clear_all, daemon=True).start()
    return False


# idle blink for inactive stations
_idle_stop = threading.Event()
_idle_threads = []


def _idle_blink_worker(pid, num_devs):
    """Pulse random physical light commands on an inactive station."""
    speed_s = get_process_speed_ms() / 1000.0
    _idle_stop.wait(random.uniform(speed_s * 1.0, speed_s * 3.0))
    while not _idle_stop.is_set():
        speed_s = get_process_speed_ms() / 1000.0
        dev = random.randint(0, num_devs - 1)
        if not process_write(pid, dev, True):
            break
        with state_lock:
            plc_states[pid]["status_lights"][dev] = True
        _idle_stop.wait(random.uniform(speed_s * 0.5, speed_s * 1.25))
        if not write_command(pid, dev, False) and process_running:
            plc = next(p for p in PLCS if p["id"] == pid)
            latch_process_fault("Status-light OFF write failed for {}".format(plc["name"]))
            threading.Thread(target=clear_all, daemon=True).start()
            break
        with state_lock:
            plc_states[pid]["status_lights"][dev] = False
        _idle_stop.wait(random.uniform(speed_s * 3.5, speed_s * 10.0))
    with state_lock:
        plc_states[pid]["status_lights"] = [False] * num_devs


def start_idle_blink(exclude_pid):
    stop_idle_blink()  # clean up any old threads
    _idle_stop.clear()
    stations_map = {p["id"]: len(p["devices"]) for p in PLCS}
    for pid, ndev in stations_map.items():
        if pid == exclude_pid:
            continue
        t = threading.Thread(target=_idle_blink_worker, args=(pid, ndev), daemon=True)
        t.start()
        _idle_threads.append(t)


def stop_idle_blink():
    _idle_stop.set()
    for t in _idle_threads:
        t.join(timeout=2)
    _idle_threads.clear()


# Legacy stopped-state worker controls retained for clean shutdown compatibility.
# A stopped cell is intentionally de-energized; no workers are started.
_global_idle_stop = threading.Event()
_global_idle_threads = []


def _global_idle_blink_worker(pid, num_devs):
    """Keep compatibility with old callers without energizing a stopped cell."""
    _global_idle_stop.wait()
    with state_lock:
        plc_states[pid]["status_lights"] = [False] * num_devs


def start_global_idle_blink():
    """Stopped means de-energized; clear heartbeat state and start no workers."""
    stop_global_idle_blink()
    with state_lock:
        for plc in PLCS:
            plc_states[plc["id"]]["status_lights"] = [False] * len(plc["devices"])


def stop_global_idle_blink():
    """Stop all global idle blink threads."""
    _global_idle_stop.set()
    for t in _global_idle_threads:
        t.join(timeout=2)
    _global_idle_threads.clear()


def process_loop():
    global process_running, process_stage, process_step

    # Station definitions
    stations = [
        {"pid": "mlgx30", "name": "Conveyor In",       "devs": 5},
        {"pid": "mlgx31", "name": "Drill Press",       "devs": 5},
        {"pid": "s7",     "name": "Assembly Robot",     "devs": 5},
        {"pid": "phx",    "name": "Paint & QC",        "devs": 2},
    ]

    production_runs = [
        "Batch A-{:04d}",
        "Batch B-{:04d}",
        "Batch C-{:04d}",
        "Batch D-{:04d}",
    ]

    batch_num = random.randint(1000, 9999)
    run_count = 0
    add_event("INFO", "PROCESS", "Automatic production sequence started")

    while process_running:
        batch_label = production_runs[run_count % len(production_runs)].format(batch_num)
        run_count += 1
        batch_num += 1

        # --- Stage 1: Conveyor loads raw material ---
        start_idle_blink("mlgx30")
        process_stage = "{} - Loading raw material".format(batch_label)
        for tick in range(12):
            if not process_running: break
            process_step = int(tick / 12 * 25)
            # Cascade conveyor lights one by one
            for i in range(5):
                if not process_write("mlgx30", i, i <= tick % 5): break
            scaled_sleep(0.7, 1.3)
        # Conveyor running steady
        if process_running:
            for i in range(5):
                if not process_write("mlgx30", i, True): break
            scaled_sleep(1.5)
        # Turn off conveyor
        for i in range(5):
            write_command("mlgx30", i, False)

        if not process_running: break

        # --- Stage 2: Drill Press machining ---
        start_idle_blink("mlgx31")
        process_stage = "{} - Drilling holes".format(batch_label)
        for tick in range(15):
            if not process_running: break
            process_step = 25 + int(tick / 15 * 25)
            # Simulate drill: motor on, clamp, drill down, retract
            if not process_write("mlgx31", 0, True): break  # Motor
            if not process_write("mlgx31", 1, tick % 4 < 2): break  # Clamp cycling
            if not process_write("mlgx31", 2, tick % 3 == 0): break  # Drill pulse
            if not process_write("mlgx31", 3, tick % 5 < 3): break  # Coolant
            if not process_write("mlgx31", 4, random.random() < 0.4): break  # Status
            scaled_sleep(0.7, 1.2)
        for i in range(5):
            write_command("mlgx31", i, False)

        if not process_running: break

        # --- Stage 3: Robot Assembly ---
        start_idle_blink("s7")
        process_stage = "{} - Assembling components".format(batch_label)
        for tick in range(18):
            if not process_running: break
            process_step = 50 + int(tick / 18 * 25)
            # Simulate: pick, move, place, weld, inspect
            cycle_pos = tick % 6
            if not process_write("s7", 0, cycle_pos < 3): break     # Arm extend/retract
            if not process_write("s7", 1, cycle_pos in [1,2]): break # Gripper
            if not process_write("s7", 2, cycle_pos == 3): break     # Weld
            if not process_write("s7", 3, cycle_pos == 4): break     # Rotate
            if not process_write("s7", 4, cycle_pos == 5): break     # Done signal
            scaled_sleep(0.5, 1.2)
        for i in range(5):
            write_command("s7", i, False)

        if not process_running: break

        # --- Stage 4: Paint & Quality Check ---
        start_idle_blink("phx")
        process_stage = "{} - Paint & quality check".format(batch_label)
        for tick in range(10):
            if not process_running: break
            process_step = 75 + int(tick / 10 * 20)
            if not process_write("phx", 0, tick % 3 != 2): break  # Spray on/off
            if not process_write("phx", 1, tick > 6): break        # QC pass light
            scaled_sleep(0.8, 1.6)
        write_command("phx", 0, False)
        if process_running:
            process_write("phx", 1, True)  # QC pass
            scaled_sleep(2.0)
            write_command("phx", 1, False)

        if not process_running: break

        # --- Changeover ---
        stop_idle_blink()
        process_stage = "{} - Complete! Changeover...".format(batch_label)
        process_step = 100
        scaled_sleep(5.0)
        process_step = 0

    stop_idle_blink()
    if not process_fault_latched:
        process_stage = "Stopped"
        process_step = 0
    clear_all()
    if not process_fault_latched:
        add_event("INFO", "PROCESS", "Automatic sequence stopped; all commands cleared")


# flask app - use an explicit template path so launch CWD/import style cannot
# change where the operator interface is loaded from.
app = Flask(
    __name__,
    template_folder=os.path.join(os.path.dirname(os.path.abspath(__file__)), "templates"),
    static_folder=os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "assets"),
    static_url_path="/assets",
)


@app.after_request
def set_operator_response_headers(response):
    """Prevent stale operator state and add basic browser hardening headers."""
    response.headers["Cache-Control"] = "no-store, max-age=0"
    response.headers["Pragma"] = "no-cache"
    response.headers["X-Content-Type-Options"] = "nosniff"
    response.headers["X-Frame-Options"] = "SAMEORIGIN"
    response.headers["Referrer-Policy"] = "no-referrer"
    return response


@app.route("/")
def index():
    return render_template("index.html", plcs=PLCS)


@app.route("/api/state")
def api_state():
    now = time.monotonic()
    missing = unavailable_plcs()
    with state_lock:
        data = {
            "plcs": {},
            "operating_mode": operating_mode,
            "process_running": process_running,
            "process_stage": process_stage,
            "process_step": process_step,
            "process_speed_ms": process_speed_ms,
            "process_fault_latched": process_fault_latched,
            "process_fault": process_fault,
            "ready": not missing and not process_fault_latched,
            "ready_count": len(PLCS) - len(missing),
            "connection_summary": (
                "NO PLCs ONLINE"
                if len(missing) == len(PLCS)
                else "{}/{} PLCs READY".format(len(PLCS) - len(missing), len(PLCS))
            ),
            "unavailable_plcs": missing,
            "interlocks": [
                reason for reason in (
                    "Select AUTO mode" if operating_mode != "AUTO" else "",
                    "Reset the latched fault" if process_fault_latched else "",
                    "No PLCs online; background reconnect is active"
                    if len(missing) == len(PLCS)
                    else "Restore verified PLC communications: {}".format(", ".join(missing)) if missing else "",
                ) if reason
            ],
            "gaspot_connected": gaspot_connected,
            "gaspot_error": gaspot_error,
            "tanks": list(tank_readings),
            "events": recent_events(),
            "server_time": datetime.now().astimezone().isoformat(timespec="seconds"),
        }
        for plc in PLCS:
            pid = plc["id"]
            last_seen = plc_states[pid]["last_seen"]
            data["plcs"][pid] = {
                "name": plc["name"],
                "ip": plc["ip"],
                "protocol": plc["proto"].upper(),
                "station": plc["station"],
                "connected": plc_states[pid]["connected"],
                "fresh": plc_states[pid]["connected"] and now - last_seen <= PLC_FRESH_SECONDS,
                "age_ms": round((now - last_seen) * 1000) if last_seen > 0 else None,
                "outputs": plc_states[pid]["outputs"],
                "commands": plc_states[pid]["commands"],
                "status_lights": plc_states[pid]["status_lights"],
                "error": plc_states[pid]["error"],
                "devices": len(plc["devices"]),
                "points": [
                    {
                        "idx": dev["idx"],
                        "label": dev.get("label", "Point {}".format(dev["idx"])),
                        "command_address": dev.get("command", "Coil {}".format(dev["idx"])),
                        "feedback_address": dev.get("output", "Coil {}".format(dev["idx"])),
                    }
                    for dev in plc["devices"]
                ],
            }
    return jsonify(data)


@app.route("/api/command", methods=["POST"])
def api_command():
    with process_control_lock:
        return _api_command_locked()


def _api_command_locked():
    body = request.json or {}
    pid = body.get("plc_id")
    state = body.get("state", False)
    if process_running:
        return jsonify({"ok": False, "error": "Manual commands are disabled while the process is running"}), 409
    if bool(state) and operating_mode != "MANUAL":
        return jsonify({"ok": False, "error": "Select MANUAL mode before issuing an ON command"}), 409
    if process_fault_latched and bool(state):
        return jsonify({"ok": False, "error": "Manual ON commands are disabled while a fault is latched"}), 409
    if pid not in conns:
        return jsonify({"ok": False, "error": "Unknown PLC"}), 400
    try:
        dev_idx = int(body.get("dev_idx"))
        if dev_idx < 0 or dev_idx >= len(next(p for p in PLCS if p["id"] == pid)["devices"]):
            raise ValueError
    except (TypeError, ValueError):
        return jsonify({"ok": False, "error": "Invalid device index"}), 400
    if bool(state):
        with state_lock:
            ready = (
                plc_states[pid]["connected"]
                and time.monotonic() - plc_states[pid]["last_seen"] <= PLC_FRESH_SECONDS
            )
        if not ready:
            return jsonify({"ok": False, "error": "PLC does not have a fresh connection"}), 409
    ok = write_command(pid, dev_idx, bool(state))
    if not ok:
        return jsonify({"ok": False, "error": "PLC command failed"}), 503
    plc = next(p for p in PLCS if p["id"] == pid)
    point = plc["devices"][dev_idx].get("label", "Point {}".format(dev_idx))
    add_event("ACTION", plc["name"], "Manual command {} -> {}".format(point, "ON" if state else "OFF"))
    return jsonify({"ok": True})


@app.route("/api/mode", methods=["POST"])
def api_mode():
    global operating_mode, process_stage
    body = request.json or {}
    requested = str(body.get("mode", "")).upper()
    if requested not in ("AUTO", "MANUAL"):
        return jsonify({"ok": False, "error": "Mode must be AUTO or MANUAL"}), 400
    with process_control_lock:
        if process_running:
            return jsonify({"ok": False, "error": "Stop the automatic sequence before changing mode"}), 409
        failures = clear_all()
        if failures:
            latch_process_fault("Mode-change OFF command failed for {}".format(", ".join(failures)))
            return jsonify({"ok": False, "error": process_fault}), 503
        with state_lock:
            operating_mode = requested
            process_stage = "{} mode - stopped".format(requested.title())
        add_event("ACTION", "OPERATOR", "Operating mode changed to {}".format(requested))
    return jsonify({"ok": True, "mode": requested})


@app.route("/api/process", methods=["POST"])
def api_process():
    with process_control_lock:
        return _api_process_locked()


def _api_process_locked():
    global process_running, process_stage, process_step, process_fault_latched, process_fault
    body = request.json or {}
    action = body.get("action", "toggle")

    if action == "start" and not process_running:
        if operating_mode != "AUTO":
            return jsonify({"ok": False, "running": False, "error": "Select AUTO mode before starting"}), 409
        if process_fault_latched:
            return jsonify({"ok": False, "running": False, "error": "Reset the latched fault before starting"}), 409
        missing = unavailable_plcs()
        if missing:
            return jsonify({
                "ok": False,
                "running": False,
                "error": "Cannot start; PLCs not ready: {}".format(", ".join(missing)),
            }), 409
        stop_global_idle_blink()
        failures = clear_all()
        if failures:
            latch_process_fault("Unable to verify OFF commands for {}".format(", ".join(failures)))
            return jsonify({"ok": False, "running": False, "error": process_fault}), 503
        process_stop_event.clear()
        process_running = True
        add_event("ACTION", "OPERATOR", "Start command accepted")
        t = threading.Thread(target=process_loop, daemon=True)
        t.start()
        return jsonify({"ok": True, "running": True})
    elif action == "stop":
        process_running = False
        process_stop_event.set()
        add_event("ACTION", "OPERATOR", "Stop command accepted")
        failures = clear_all()
        if failures:
            latch_process_fault("Stop command failed for {}".format(", ".join(failures)))
            return jsonify({"ok": False, "running": False, "error": process_fault}), 503
        return jsonify({"ok": True, "running": False})
    elif action == "reset":
        if process_running:
            return jsonify({"ok": False, "running": True, "error": "Stop the process before reset"}), 409
        missing = unavailable_plcs()
        if missing:
            return jsonify({
                "ok": False,
                "running": False,
                "error": "Cannot reset; PLCs not ready: {}".format(", ".join(missing)),
            }), 409
        failures = clear_all()
        if failures:
            return jsonify({
                "ok": False,
                "running": False,
                "error": "Cannot reset; OFF command failed for {}".format(", ".join(failures)),
            }), 503
        with state_lock:
            process_fault_latched = False
            process_fault = ""
            process_stage = "{} mode - fault reset".format(operating_mode.title())
            process_step = 0
        process_stop_event.set()
        add_event("ACTION", "OPERATOR", "Latched process fault reset")
        return jsonify({"ok": True, "running": False})
    else:
        return jsonify({"ok": True, "running": process_running})


@app.route("/api/config", methods=["GET", "POST"])
def api_config():
    global process_speed_ms
    if request.method == "POST":
        body = request.json or {}
        try:
            speed = int(body.get("speed_ms", process_speed_ms))
        except (TypeError, ValueError):
            return jsonify({"ok": False, "error": "speed_ms must be an integer"}), 400
        with state_lock:
            process_speed_ms = max(50, min(10000, speed))
        add_event("ACTION", "OPERATOR", "Process step time set to {} ms".format(process_speed_ms))
    return jsonify({"ok": True, "speed_ms": get_process_speed_ms()})


def main():
    parser = argparse.ArgumentParser(description="Web-based Manufacturing Process HMI")
    parser.add_argument("--port", type=int, default=5000, help="Web server port (default: 5000)")
    parser.add_argument("--host", default="127.0.0.1", help="Web server bind address (default: 127.0.0.1; use 0.0.0.0 for explicit LAN access)")
    parser.add_argument("--process-ms", type=int, default=300, help="Process speed/step interval ms (default: 300)")
    parser.add_argument("--source-ip", default=None, help="Local IP to bind the Modbus connection to")
    parser.add_argument("--gaspot-host", default="127.0.0.1", help="GasPot host for embedded tank monitoring (default: 127.0.0.1)")
    parser.add_argument("--gaspot-port", type=int, default=10001, help="GasPot TLS port (default: 10001)")
    parser.add_argument("--gaspot-poll-seconds", type=float, default=10.0, help="GasPot tank polling interval seconds (default: 10)")
    parser.add_argument("--disable-gaspot", action="store_true", help="Disable embedded GasPot tank monitoring")
    args = parser.parse_args()

    global process_speed_ms, gaspot_client, gaspot_poll_seconds
    process_speed_ms = max(50, args.process_ms)
    gaspot_poll_seconds = max(1.0, args.gaspot_poll_seconds)
    gaspot_client = None if args.disable_gaspot else GasPotTankClient(args.gaspot_host, args.gaspot_port)

    init_connections(source_ip=args.source_ip)

    # Start background poll thread
    poll_thread = threading.Thread(target=poll_loop, daemon=True)
    poll_thread.start()

    # No synchronous PLC I/O occurs during startup. Each successful reconnect
    # clears commands before the PLC can become ready.
    add_event("INFO", "SYSTEM", "HMI service started; PLC reconnect active in background")

    print("=" * 60)
    print("  Manufacturing Process HMI")
    print("  Open http://localhost:{} in your browser".format(args.port))
    print("=" * 60)

    app.run(host=args.host, port=args.port, debug=False)


if __name__ == "__main__":
    main()

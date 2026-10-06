#!/usr/bin/env python3
# Simple HMI – shows two lights (coils 0 & 1) read from a Modbus/TCP PLC.
# Uses tkinter for the GUI and raw Modbus/TCP packets (same approach as
# py3_modturnt.py) for maximum compatibility.
# Click on a light to toggle its coil state.
#
# Usage:
#   python hmi_lights.py 192.168.0.3
#   python hmi_lights.py 192.168.0.3 --source-ip 192.168.0.22
#   python hmi_lights.py 192.168.0.3 --poll-ms 250
#
# Author: Stephen J. Hilt
###########################################################

import argparse
import random
import socket
import struct
import sys
import tkinter as tk

from hmi_branding import apply_window_branding, configure_process_branding


# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------

def parse_args():
    parser = argparse.ArgumentParser(
        description="Simple HMI that shows two PLC coil lights via Modbus/TCP.",
        epilog=(
            "Examples:\n"
            "  python hmi_lights.py 192.168.0.3\n"
            "  python hmi_lights.py 192.168.0.3 --source-ip 192.168.0.22\n"
            "  python hmi_lights.py 192.168.0.3 --poll-ms 200\n"
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("host", help="Target Modbus/TCP host (PLC IP)")
    parser.add_argument(
        "-p", "--port", type=int, default=502,
        help="Modbus/TCP port (default: 502)",
    )
    parser.add_argument(
        "-s", "--source-ip",
        help="Local IP to bind to (selects the outgoing NIC)",
    )
    parser.add_argument(
        "--poll-ms", type=int, default=500,
        help="Polling interval in milliseconds (default: 500)",
    )
    parser.add_argument(
        "--timeout", type=float, default=3.0,
        help="Modbus TCP timeout in seconds (default: 3)",
    )
    return parser.parse_args()


# ---------------------------------------------------------------------------
# Raw Modbus/TCP helpers (matches py3_modturnt.py approach)
# ---------------------------------------------------------------------------

class ModbusConnection:
    """Thin wrapper around a raw TCP socket speaking Modbus/TCP."""

    def __init__(self, host, port, source_ip=None, timeout=3.0):
        self.host = host
        self.port = port
        self.source_ip = source_ip
        self.timeout = timeout
        self.sock = None
        self._tx_id = 0

    @property
    def connected(self):
        return self.sock is not None

    def _next_tx_id(self):
        self._tx_id = (self._tx_id + 1) & 0xFFFF
        return self._tx_id

    def connect(self):
        """Open a TCP connection to the PLC."""
        self.close()
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(self.timeout)
            if self.source_ip:
                s.bind((self.source_ip, 0))
            s.connect((self.host, self.port))
            self.sock = s
            return True
        except OSError:
            self.sock = None
            return False

    def close(self):
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None

    def _send_recv(self, packet):
        """Send a Modbus/TCP frame and receive the response."""
        self.sock.sendall(packet)
        return self.sock.recv(4096)

    def read_coils(self, start_addr=0, count=2, unit_id=1):
        """Send Read Coils (FC 01) and return a list of bools."""
        tx_id = self._next_tx_id()
        # MBAP header + FC 01 + start address (2 bytes) + quantity (2 bytes)
        packet = struct.pack(
            ">HHHBBHH",
            tx_id,       # Transaction ID
            0,           # Protocol ID
            6,           # Length (unit id + fc + 4 data bytes)
            unit_id,     # Unit ID
            0x01,        # Function Code: Read Coils
            start_addr,  # Starting address
            count,       # Quantity of coils
        )
        resp = self._send_recv(packet)
        # Response: MBAP (7 bytes) + FC (1) + byte count (1) + data bytes
        if len(resp) < 10:
            raise ValueError("Short response ({} bytes)".format(len(resp)))
        fc = resp[7]
        if fc & 0x80:
            raise ValueError("Modbus exception: FC=0x{:02x} code=0x{:02x}".format(fc, resp[8]))
        byte_count = resp[8]
        data_bytes = resp[9 : 9 + byte_count]
        # Unpack bits
        bits = []
        for b in data_bytes:
            for i in range(8):
                bits.append(bool(b & (1 << i)))
        return bits[:count]

    def write_coil(self, coil, enabled, unit_id=1):
        """Send Write Single Coil (FC 05)."""
        tx_id = self._next_tx_id()
        value = 0xFF00 if enabled else 0x0000
        packet = struct.pack(
            ">HHHBBHH",
            tx_id,     # Transaction ID
            0,         # Protocol ID
            6,         # Length
            unit_id,   # Unit ID
            0x05,      # Function Code: Write Single Coil
            coil,      # Coil address
            value,     # Value
        )
        resp = self._send_recv(packet)
        fc = resp[7]
        if fc & 0x80:
            raise ValueError("Modbus write exception: FC=0x{:02x} code=0x{:02x}".format(fc, resp[8]))


# ---------------------------------------------------------------------------
# GUI
# ---------------------------------------------------------------------------

COLOR_ON_GREEN = "#00ff00"  # bright green when left coil is ON
COLOR_ON_RED   = "#ff2222"  # bright red when right coil is ON
COLOR_OFF      = "#333333"  # dark grey when coil is OFF
COLOR_ERR      = "#ff4444"  # red tint when connection lost
BG_COLOR  = "#1a1a2e"   # dark background
LABEL_FG  = "#e0e0e0"   # light text


class HMIApp:
    def __init__(self, root, conn, poll_ms):
        self.root = root
        self.conn = conn
        self.poll_ms = poll_ms
        self.coil_states = [False, False]
        self.random_active = False
        self._random_job = None

        self.root.title("PLC HMI \u2013 {} : {}".format(conn.host, conn.port))
        apply_window_branding(self.root)
        self.root.configure(bg=BG_COLOR)
        self.root.resizable(False, False)

        # Title
        tk.Label(
            root, text="PLC Coil Status",
            font=("Segoe UI", 20, "bold"), fg=LABEL_FG, bg=BG_COLOR,
        ).pack(pady=(18, 4))

        # Connection status
        self.status_var = tk.StringVar(value="Connecting\u2026")
        self.status_label = tk.Label(
            root, textvariable=self.status_var,
            font=("Segoe UI", 10), fg="#aaaaaa", bg=BG_COLOR,
        )
        self.status_label.pack(pady=(0, 10))

        # Frame for the two lights
        frame = tk.Frame(root, bg=BG_COLOR)
        frame.pack(padx=40, pady=10)

        # --- Light 0 ---
        col0 = tk.Frame(frame, bg=BG_COLOR)
        col0.pack(side=tk.LEFT, padx=30)
        tk.Label(col0, text="Button 0", font=("Segoe UI", 14),
                 fg=LABEL_FG, bg=BG_COLOR).pack(pady=(0, 6))
        self.canvas0 = tk.Canvas(col0, width=120, height=120,
                                 bg=BG_COLOR, highlightthickness=0, cursor="hand2")
        self.canvas0.pack()
        self.light0 = self.canvas0.create_oval(
            10, 10, 110, 110, fill=COLOR_OFF, outline="#555555", width=3)
        self.canvas0.bind("<Button-1>", lambda e: self.toggle_coil(0))
        self.label0_var = tk.StringVar(value="OFF")
        tk.Label(col0, textvariable=self.label0_var, font=("Segoe UI", 12, "bold"),
                 fg=LABEL_FG, bg=BG_COLOR).pack(pady=(6, 0))

        # --- Light 1 ---
        col1 = tk.Frame(frame, bg=BG_COLOR)
        col1.pack(side=tk.LEFT, padx=30)
        tk.Label(col1, text="Button 1", font=("Segoe UI", 14),
                 fg=LABEL_FG, bg=BG_COLOR).pack(pady=(0, 6))
        self.canvas1 = tk.Canvas(col1, width=120, height=120,
                                 bg=BG_COLOR, highlightthickness=0, cursor="hand2")
        self.canvas1.pack()
        self.light1 = self.canvas1.create_oval(
            10, 10, 110, 110, fill=COLOR_OFF, outline="#555555", width=3)
        self.canvas1.bind("<Button-1>", lambda e: self.toggle_coil(1))
        self.label1_var = tk.StringVar(value="OFF")
        tk.Label(col1, textvariable=self.label1_var, font=("Segoe UI", 12, "bold"),
                 fg=LABEL_FG, bg=BG_COLOR).pack(pady=(6, 0))

        # --- Random button ---
        self.random_btn_var = tk.StringVar(value="\u25b6 Random")
        self.random_btn = tk.Button(
            root,
            textvariable=self.random_btn_var,
            font=("Segoe UI", 12, "bold"),
            fg="#ffffff", bg="#4a4a8a",
            activebackground="#6a6aaa", activeforeground="#ffffff",
            relief=tk.FLAT, padx=20, pady=6,
            cursor="hand2",
            command=self.toggle_random,
        )
        self.random_btn.pack(pady=(10, 4))

        # Info bar
        tk.Label(
            root,
            text="Click a light to toggle  |  Polling every {} ms".format(poll_ms),
            font=("Segoe UI", 9), fg="#777777", bg=BG_COLOR,
        ).pack(pady=(10, 14))

        # Start
        self._try_connect()
        self.poll()

    # ---- connection -------------------------------------------------------

    def _try_connect(self):
        ok = self.conn.connect()
        if ok:
            src = ""
            if self.conn.source_ip:
                src = " from {}".format(self.conn.source_ip)
            self.status_var.set("Connected to {}:{}{}".format(
                self.conn.host, self.conn.port, src))
            self.status_label.configure(fg="#66cc66")
        else:
            self.status_var.set("Connection failed \u2013 retrying\u2026")
            self.status_label.configure(fg=COLOR_ERR)

    # ---- polling ----------------------------------------------------------

    def poll(self):
        if not self.conn.connected:
            self._try_connect()

        if self.conn.connected:
            try:
                self.coil_states = self.conn.read_coils(0, 2)
            except Exception:
                self.status_var.set("Connection lost \u2013 reconnecting\u2026")
                self.status_label.configure(fg=COLOR_ERR)
                self.conn.close()

        # Update visuals
        on_colors = [COLOR_ON_GREEN, COLOR_ON_RED]
        for idx, (canvas, light, lbl) in enumerate([
            (self.canvas0, self.light0, self.label0_var),
            (self.canvas1, self.light1, self.label1_var),
        ]):
            on = self.coil_states[idx] if idx < len(self.coil_states) else False
            canvas.itemconfig(light, fill=on_colors[idx] if on else COLOR_OFF)
            lbl.set("ON" if on else "OFF")

        self.root.after(self.poll_ms, self.poll)

    # ---- random mode ------------------------------------------------------

    def toggle_random(self):
        """Start or stop random toggling of coils."""
        if self.random_active:
            self.random_active = False
            self.random_btn_var.set("\u25b6 Random")
            self.random_btn.configure(bg="#4a4a8a")
            if self._random_job is not None:
                self.root.after_cancel(self._random_job)
                self._random_job = None
            self.status_var.set("Random mode stopped")
            self.status_label.configure(fg="#66cc66")
        else:
            self.random_active = True
            self.random_btn_var.set("\u25a0 Stop Random")
            self.random_btn.configure(bg="#aa3333")
            self.status_var.set("Random mode active")
            self.status_label.configure(fg="#ffaa00")
            self._random_tick()

    def _random_tick(self):
        """Toggle a random coil, then schedule the next tick."""
        if not self.random_active or not self.conn.connected:
            if not self.conn.connected:
                self.status_var.set("Random paused \u2013 not connected")
                self.status_label.configure(fg=COLOR_ERR)
            if self.random_active:
                self._random_job = self.root.after(1000, self._random_tick)
            return

        coil_num = random.randint(0, 1)
        new_state = random.choice([True, False])
        try:
            self.conn.write_coil(coil_num, new_state)
            self.coil_states[coil_num] = new_state
        except Exception:
            pass  # poll() will handle reconnection

        # Random delay between 100ms and 800ms
        delay = random.randint(100, 800)
        self._random_job = self.root.after(delay, self._random_tick)

    # ---- toggle -----------------------------------------------------------

    def toggle_coil(self, coil_num):
        if not self.conn.connected:
            return
        new_state = not self.coil_states[coil_num]
        try:
            self.conn.write_coil(coil_num, new_state)
            self.coil_states[coil_num] = new_state
            # Immediate visual feedback
            canvas = self.canvas0 if coil_num == 0 else self.canvas1
            light = self.light0 if coil_num == 0 else self.light1
            lbl = self.label0_var if coil_num == 0 else self.label1_var
            on_color = COLOR_ON_GREEN if coil_num == 0 else COLOR_ON_RED
            canvas.itemconfig(light, fill=on_color if new_state else COLOR_OFF)
            lbl.set("ON" if new_state else "OFF")
            self.status_var.set("Toggled coil {} \u2192 {}".format(
                coil_num, "ON" if new_state else "OFF"))
            self.status_label.configure(fg="#66cc66")
        except Exception as exc:
            self.status_var.set("Write failed: {}".format(exc))
            self.status_label.configure(fg=COLOR_ERR)
            self.conn.close()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    args = parse_args()
    configure_process_branding()

    conn = ModbusConnection(
        host=args.host,
        port=args.port,
        source_ip=args.source_ip,
        timeout=args.timeout,
    )

    root = tk.Tk()
    app = HMIApp(root, conn, args.poll_ms)

    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        conn.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())

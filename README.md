# trainer-hmi

HMI (Human-Machine Interface) scripts for a multi-PLC training lab.

## License

This project is source-available for noncommercial use. Software is licensed
under the **PolyForm Noncommercial License 1.0.0**; project documentation and
artwork are licensed under **CC BY-NC-SA 4.0**. Commercial use is not
permitted under either license. See [`LICENSING.md`](LICENSING.md) for the
exact scope, attribution requirements, complete terms, and treatment of
earlier MIT-licensed copies.

## PLCs

| PLC | IP | Protocol | I/O |
|---|---|---|---|
| MicroLogix 1100 | 192.168.0.30 | EtherNet/IP (SLC) | B3:1/0-4, I:0/0-4, O:0/0-4 |
| MicroLogix 1100 | 192.168.0.31 | EtherNet/IP (SLC) | B3:0/0-4, I:0/0-4, O:0/0-4 |
| Siemens S7-1200 | 192.168.0.2 | S7comm (snap7) | M0.0-4, I0.0-4, Q0.0-4 |
| Phoenix Contact | 192.168.0.3 | Modbus/TCP | Coils 0-1 |

## Scripts

### Tkinter-based (standalone)

- **`enip_hmi.py`** — Single PLC HMI for MicroLogix .31
- **`enip_hmi_2.py`** — Single PLC HMI for MicroLogix .30
- **`s7_hmi.py`** — Single PLC HMI for Siemens S7-1200
- **`hmi_lights.py`** — Modbus HMI for Phoenix Contact
- **`mfg_hmi.py`** — All-in-one manufacturing process HMI (all 4 PLCs)
- **`enipturnt.py`** — ENIP utility script

### Web-based

- **`web_hmi/app.py`** — Flask web server + animated factory dashboard
  - High-performance process overview, compact line, PLC I/O diagnostics, and event/alarm journal
  - Separate command and measured-feedback indications with enforced AUTO/MANUAL ownership

## Install

```bash
pip install -r requirements.txt
```

### GasPot (local dependency)

GasPot provides the ATG (Automatic Tank Gauge) simulator backend.
It is **not** included in this repo — clone it locally into a `GasPot/`
subdirectory (which is listed in `.gitignore`):

```bash
git clone https://github.com/sjhilt/GasPot.git GasPot
```

The Widget Factory configuration is stored in
**`gaspot-widget-factory.ini`** (tracked in this repo).  `run.py`
automatically passes `--config gaspot-widget-factory.ini` to GasPot so
you never need to edit GasPot's own `config.ini`.

## Run

The desktop title bar/taskbar and Web browser tab use the included Trainer Cell control-system icon in `assets/`, rather than the default Python/Tk identity. Running from source still uses `python.exe` as the underlying process; creating a fully branded executable would require an optional packaging step.

### Combined launcher (recommended)

```bash
python run.py --web            # Web HMI + GasPot
python run.py                  # Desktop HMI + GasPot
python run.py --web --port 8080
python run.py --web --host 0.0.0.0  # Explicitly allow LAN access
python run.py --no-gaspot      # HMI only, no GasPot
python run.py --gaspot-only    # GasPot only, no HMI
```

### Tkinter HMIs (standalone)
```bash
python enip_hmi.py          # MicroLogix .31
python enip_hmi_2.py        # MicroLogix .30
python s7_hmi.py            # Siemens S7-1200
python hmi_lights.py        # Phoenix Contact (Modbus)
python mfg_hmi.py           # All 4 PLCs combined
```

### Web HMI (standalone, without GasPot)
```bash
cd web_hmi
python app.py
# Open http://localhost:5000
```

The Web HMI binds to `127.0.0.1` by default. Use `--host 0.0.0.0` only on a trusted, firewalled control/training network; the application does not provide user authentication or replace network segmentation.

## Desktop and Web HMI fail-safe behavior

- **Applies to both combined HMIs:** `python run.py` (`mfg_hmi.py`) and `python run.py --web` (`web_hmi/app.py`).
- **AUTO/MANUAL ownership is enforced:** AUTO owns sequence and heartbeat commands; manual ON commands require MANUAL mode and fresh PLC feedback. Changing mode first clears all commands.
- **Run is interlocked:** all four PLCs must have a recent successful read before a cycle can start.
- **Offline startup is non-blocking:** both HMIs load immediately with **NO PLCs ONLINE** when equipment is unavailable. Reconnect runs in the background with short connection timeouts and exponential retry backoff, so offline PLCs do not freeze the interface or generate continuous network traffic.
- **Communication/write faults stop sequencing:** the process is aborted, OFF commands are attempted on every reachable PLC, and the fault is latched.
- **Manual reset is required:** restore communications, verify the machine is physically safe, then use **Reset Fault** in the desktop or Web HMI. Reset is rejected until all PLCs are online and OFF commands succeed.
- **Reconnects start safe:** command bits/coils are written OFF before a reconnected PLC is marked ready.
- **STOPPED is de-energized:** random physical heartbeat commands run only on the inactive/other PLCs during an AUTO production cycle. They follow the configured process speed, are written back OFF, and a failed write latches a process fault.
- **Command and feedback are distinct:** the I/O detail screens separately show requested commands and PLC output feedback, including heartbeat ownership and scan freshness.
- **Events are journaled:** mode changes, operator commands, starts/stops, communication transitions, and latched faults appear in an in-memory event/alarm history.
- **Derived values are identified:** animated process readouts are training/demo values unless the screen identifies them as GasPot tank data.

> **Safety limitation:** This training HMI is not a safety-rated control system. If a PLC or network link is unreachable, software cannot guarantee that the remote machine is de-energized. Use a hardwired emergency stop, safety relay/safety PLC, guarded equipment, and contactor/drive interlocks designed and validated for the machine. Do not rely on the HMI, Ethernet, or ordinary PLC logic as the sole personnel or equipment protection layer.

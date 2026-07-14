# trainer-hmi

HMI (Human-Machine Interface) scripts for a multi-PLC training lab.

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
  - Conveyor belt, drill press, robot arm, paint spray animations
  - Real-time polling, click-to-toggle lights, run/stop process button

## Install

```bash
pip install -r requirements.txt
```

## Run

### Tkinter HMIs
```bash
python enip_hmi.py          # MicroLogix .31
python enip_hmi_2.py        # MicroLogix .30
python s7_hmi.py            # Siemens S7-1200
python hmi_lights.py        # Phoenix Contact (Modbus)
python mfg_hmi.py           # All 4 PLCs combined
```

### Web HMI
```bash
cd web_hmi
python app.py
# Open http://localhost:5000
```

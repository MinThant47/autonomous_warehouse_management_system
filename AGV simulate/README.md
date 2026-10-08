# Two-robot AGV simulator

This folder contains a small MQTT client that simulates R1 and R2 on the same
warehouse map used by the backend. It does not call backend Python functions or
the HTTP API. Robot reports and commands use MQTT, like the physical AGVs.

The simulator subscribes to `agv/+/command` and publishes:

- `agv/R1/node` and `agv/R2/node` with `{"node_id":"<map node>"}` at startup
  and whenever a simulated robot reaches an RFID node.
- `agv/R1/telemetry` and `agv/R2/telemetry` with cumulative integer
  `ticks_L` and `ticks_R` readings while moving and while idle.

The simulator uses the backend's `backend/new_warehouse_map.py` graph and
accepts only commands to adjacent RFID nodes (or a same-node target, which the
backend's encoder replanner can use to return to the last RFID tag). The
simulator moves at `speed_mps` and converts simulated travel into cumulative
encoder ticks using each robot's configurable ticks-per-centimeter factors.
Forklift commands are logged; no physical load or forklift actuator is modeled.

## Start the application

Run the local Mosquitto broker first. The backend's existing run guide uses
`localhost:1883`.

**Disconnect the physical AGVs from MQTT before starting this simulator.** It
uses the same `R1`/`R2` topics, so a real robot subscribed to the broker would
also receive the simulator-driven backend commands.

In **Terminal 1**, start the backend from the `backend` folder:

```powershell
cd "C:\Users\USER\Desktop\AGV\autonomous_warehouse_management_system-main\backend"
$env:MQTT_ENABLED = "true"
$env:MQTT_HOST = "localhost"
$env:MQTT_PORT = "1883"
python app.py
```

In **Terminal 2**, install the simulator dependencies once and start both virtual
AGVs:

```powershell
cd "C:\Users\USER\Desktop\AGV\autonomous_warehouse_management_system-main\AGV simulate"
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
python simulator.py
```

If PowerShell blocks activation, run the simulator's virtual-environment Python
directly instead:

```powershell
.\.venv\Scripts\python.exe simulator.py
```

In **Terminal 3**, start the frontend:

```powershell
cd "C:\Users\USER\Desktop\AGV\autonomous_warehouse_management_system-main\frontend"
npm run dev
```

Assign missions to R1 or R2 in the UI. The simulator will report
the starting RFID nodes `Parking_1` and `Parking_2`; then it will obey the
backend's commands and report each arrival. Stop the simulator with `Ctrl+C`.

## Configuration

Edit `config.json` to change the broker, simulated speed, telemetry interval,
initial parking nodes, map scale, or per-wheel encoder tick factors. The initial
speed is `0.25333 m/s`; tick factors are `40 ticks/cm` for each wheel, matching
the backend's current simulation-only placeholder calibration. They are not
measured values for the real AGVs.

The `MQTT_HOST` and `MQTT_PORT` environment variables override the broker values
in this file. If the broker requires authentication, set `MQTT_USERNAME` and
`MQTT_PASSWORD` in the simulator's terminal; those credentials are not stored
in the config. MQTT publishes use QoS 1.

## Restarting

This first version starts both simulated robots at their configured parking
nodes and begins encoder counters at zero. For a clean reset, stop and restart
both the backend and simulator before assigning new missions. The simulator
does not persist robot position, tick totals, or active missions between runs.

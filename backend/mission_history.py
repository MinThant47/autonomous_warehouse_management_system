"""Append-only CSV history for AGV mission encoder and RFID events."""

import csv
import logging
import os
from datetime import datetime, timedelta, timezone
from pathlib import Path
from threading import RLock
from uuid import uuid4


_DEFAULT_HISTORY_PATH = Path(__file__).with_name("data") / "mission_history.csv"
_LOCK = RLock()
_LOG = logging.getLogger(__name__)
_FIELDS = [
    "mission_id", "robot_id", "task_id", "serial_number", "task_type",
    "start_node", "pickup_node", "destination_node", "event", "event_at",
    "task_status", "node", "source", "ticks_L", "ticks_R",
    "distance_since_last_node_cm", "encoder_distance_calibrated",
]


def history_path():
    configured = os.getenv("MISSION_HISTORY_CSV")
    return Path(configured) if configured else _DEFAULT_HISTORY_PATH


def begin_mission(robot_id, task, start_node):
    """Assign a stable trip ID and write its starting event."""
    task.setdefault("mission_id", str(uuid4()))
    task.setdefault("mission_started_at", datetime.now(timezone.utc).isoformat())
    task.setdefault("mission_start_node", start_node)
    record_mission_event(robot_id, task, "mission_started", node=start_node)
    return task["mission_id"]


def record_mission_event(
    robot_id,
    task,
    event,
    *,
    node=None,
    source=None,
    ticks_left=None,
    ticks_right=None,
    distance_cm=None,
    calibrated=None,
):
    """Append one event row. Missing fields are left blank in the CSV."""
    if not task or not task.get("mission_id"):
        return
    row = {
        "mission_id": task["mission_id"],
        "robot_id": robot_id,
        "task_id": task.get("id"),
        "serial_number": task.get("serial_number"),
        "task_type": task.get("type"),
        "start_node": task.get("mission_start_node"),
        "pickup_node": task.get("PL"),
        "destination_node": task.get("DL"),
        "event": event,
        "event_at": datetime.now(timezone.utc).isoformat(),
        "task_status": task.get("status"),
        "node": node,
        "source": source,
        "ticks_L": ticks_left,
        "ticks_R": ticks_right,
        "distance_since_last_node_cm": distance_cm,
        "encoder_distance_calibrated": calibrated,
    }
    path = history_path()
    try:
        with _LOCK:
            path.parent.mkdir(parents=True, exist_ok=True)
            needs_header = not path.exists() or path.stat().st_size == 0
            with path.open("a", newline="", encoding="utf-8-sig") as output:
                writer = csv.DictWriter(output, fieldnames=_FIELDS, extrasaction="ignore")
                if needs_header:
                    writer.writeheader()
                writer.writerow(row)
    except OSError:
        # History storage must not interrupt mission control or MQTT telemetry.
        _LOG.exception("Could not append AGV mission history to %s", path)


def read_mission_history(robot_id, mission_id=None):
    """Read persisted event rows for a robot, optionally filtering one trip."""
    path = history_path()
    if not path.exists():
        return []
    with _LOCK, path.open("r", newline="", encoding="utf-8-sig") as source:
        reader = csv.DictReader(source)
        return [
            row for row in reader
            if row.get("robot_id") == robot_id
            and (mission_id is None or row.get("mission_id") == mission_id)
        ]


def read_completed_missions(robot_id):
    """Return completed mission summaries for the robot from persisted events."""
    missions = {}
    for row in read_mission_history(robot_id):
        mission_id = row.get("mission_id")
        if not mission_id:
            continue
        mission = missions.setdefault(mission_id, {
            "mission_id": mission_id,
            "robot_id": robot_id,
            "task_id": row.get("task_id"),
            "serial_number": row.get("serial_number"),
            "task_type": row.get("task_type"),
            "start_node": row.get("start_node"),
            "pickup_node": row.get("pickup_node"),
            "destination_node": row.get("destination_node"),
            "started_at": None,
            "completed_at": None,
        })
        if row.get("event") == "mission_started" and mission["started_at"] is None:
            mission["started_at"] = row.get("event_at")
        if row.get("event") == "mission_completed":
            mission["completed_at"] = row.get("event_at")

    completed = [mission for mission in missions.values() if mission["completed_at"]]
    return sorted(completed, key=lambda mission: mission["completed_at"], reverse=True)


def export_mission_history_csv(robot_id, mission_id=None):
    """Render persisted rows as a downloadable CSV with separate date/time columns."""
    import io

    output = io.StringIO(newline="")
    export_fields = [
        field
        for field in _FIELDS
        if field != "event_at"
    ]
    event_index = export_fields.index("event") + 1
    export_fields[event_index:event_index] = ["event_date", "event_time"]
    writer = csv.DictWriter(output, fieldnames=export_fields, extrasaction="ignore")
    writer.writeheader()
    for row in read_mission_history(robot_id, mission_id):
        export_row = {key: value for key, value in row.items() if key != "event_at"}
        event_at = row.get("event_at") or ""
        try:
            event_datetime = datetime.fromisoformat(event_at)
        except ValueError:
            export_row["event_date"] = ""
            export_row["event_time"] = event_at
        else:
            myanmar_datetime = event_datetime.astimezone(timezone(timedelta(hours=6, minutes=30)))
            export_row["event_date"] = myanmar_datetime.date().isoformat()
            export_row["event_time"] = myanmar_datetime.strftime("%H:%M:%S.%f")
        writer.writerow(export_row)
    return output.getvalue()

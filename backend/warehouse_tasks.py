"""Shared warehouse task workflows used by the HTTP and MQTT adapters."""

from database.database import WarehouseDatabaseError, receive_inbound_item
from new_warehouse_map import nodes
from scheduler.scheduler import dispatch_task


tasks = []


def dispatch_warehouse_task(serial_number, task_type, pickup_location, dropoff_location):
    """Create, schedule, and retain a warehouse task."""
    task = {
        "id": len(tasks) + 1,
        "type": task_type,
        "PL": pickup_location,
        "DL": dropoff_location,
        "pickup_time": 3,
        "dropoff_time": 3,
        "deadline": 300,
        "serial_number": serial_number,
    }
    result = dispatch_task(task)
    tasks.append(task)
    return task, result


def create_inbound_warehouse_task(serial_code, pickup_location):
    """Store an arriving item and create its route to a matching empty shelf."""
    if not isinstance(serial_code, str) or not serial_code.strip():
        raise WarehouseDatabaseError("serial_code is required")
    if not isinstance(pickup_location, str) or not pickup_location.strip():
        raise WarehouseDatabaseError("pickup_location is required")

    serial_code = serial_code.strip()
    pickup_location = pickup_location.strip()
    if pickup_location not in nodes:
        raise WarehouseDatabaseError(f"Unknown pickup location: {pickup_location}")

    storage = receive_inbound_item(serial_code, pickup_location)
    task, result = dispatch_warehouse_task(
        serial_code, 0, pickup_location, storage["dropoff_location"]
    )
    return task, result, storage

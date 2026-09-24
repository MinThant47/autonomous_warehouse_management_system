"""Small in-process event bus for live dashboard updates."""
from queue import Queue
from threading import Lock

_subscribers = set()
_lock = Lock()
_camera_url = None


def get_camera_url():
    with _lock:
        return _camera_url


def publish_camera_url(url):
    """Store and broadcast the latest URL reported by the QR camera."""
    global _camera_url
    with _lock:
        _camera_url = url
    _publish({"event": "camera-url", "data": {"url": url}})


def subscribe():
    subscriber = Queue(maxsize=20)
    with _lock:
        _subscribers.add(subscriber)
    return subscriber


def unsubscribe(subscriber):
    with _lock:
        _subscribers.discard(subscriber)


def publish_robot_state(state):
    """Notify every connected dashboard without blocking MQTT processing."""
    _publish({"event": "robot-state", "data": state})


def publish_warehouse_alert(message, serial_code=None):
    """Notify dashboards when an inbound QR report cannot create a task."""
    if "already in inventory" in message:
        alert_type = "duplicate-item"
    elif "No empty shelf is available" in message:
        alert_type = "shelf-full"
    elif "does not exist" in message:
        alert_type = "unrecognized-qr"
    else:
        return
    _publish({
        "event": "warehouse-alert",
        "data": {"type": alert_type, "message": message, "serial_code": serial_code},
    })


def _publish(event):
    """Deliver an event to every dashboard without blocking MQTT processing."""
    with _lock:
        subscribers = list(_subscribers)
    for subscriber in subscribers:
        if subscriber.full():
            subscriber.get_nowait()
        subscriber.put_nowait(event)

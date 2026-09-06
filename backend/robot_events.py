"""Small in-process event bus for live robot-monitor updates."""
from queue import Queue
from threading import Lock

_subscribers = set()
_lock = Lock()


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
    with _lock:
        subscribers = list(_subscribers)
    for subscriber in subscribers:
        if subscriber.full():
            subscriber.get_nowait()
        subscriber.put_nowait(state)

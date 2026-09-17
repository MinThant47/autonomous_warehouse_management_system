"""
ESP32-CAM Live Object Detection - Flask (single file)
------------------------------------------------------
Replaces the earlier Django project. Same functionality:
  - Resolves each ESP32-CAM's mDNS hostname to an IP (works across hotspot
    reconnects, since the hostname stays fixed even when the IP changes)
  - Falls back to a STATIC/reserved IP if mDNS resolution fails (needed on
    networks -- like an office router -- where multicast/mDNS traffic is
    blocked or not forwarded, even though normal unicast connectivity is
    fine). This lets the SAME config work automatically on both a phone
    hotspot (mDNS resolves) and a router that blocks mDNS (falls back to
    the static IP), with no manual editing when you switch networks.
  - Pulls MJPEG frames from /stream on each camera, skipping backlog so
    detection never falls behind
  - Runs your custom-trained YOLO model (3 classes only -- no general
    COCO detection)
  - Detection runs every Nth frame (DETECT_EVERY_N_FRAMES) for higher
    displayed FPS; skipped frames reuse the last known boxes
  - Serves each camera as its own MJPEG stream, shared across any number
    of simultaneous viewers (one background thread per camera, not per
    viewer -- this matters a lot for CPU usage)
  - Per-class colored bounding boxes with semi-transparent label
    background drawn INSIDE the box (top-left corner)
  - Any "obstacle" class (traffic_cone, forklift) is drawn in a fixed
    warning color (red) and triggers a JSON API + an on-page warning
    banner
  - FPS shown in a black bar ABOVE the video, not overlapping it
  - A /api/obstacle JSON endpoint so a teammate's robot-navigation code
    can find out which camera currently sees an obstacle, without
    needing to touch any of this Python/YOLO code

No Arduino code changes needed -- this only talks to the existing
/stream endpoint your ESP32-CAM sketches already expose.

IMPORTANT for the static-IP fallback to actually stay valid: reserve a
fixed DHCP lease for each ESP32-CAM's MAC address in your router's admin
panel (usually under "DHCP" -> "Address Reservation" / "Static Leases").
Without a reservation, the router could hand the camera a different IP
next time it reconnects, and the fallback value below would go stale.

Run as part of the main warehouse server (recommended):
    MQTT_ENABLED=true MQTT_HOST=localhost WAITRESS_THREADS=20 python app.py
    # Open http://<computer-ip>:8000/object-detection/

The standalone application below remains available only for isolated camera
testing.  In normal use, do not start a second Waitress process on port 8001.
"""

# ------------------------------ IMPORTS ------------------------------
import os              # reads the optional website URL configuration
import re              # parsing MAC addresses out of the OS's "arp -a" output
import socket          # used to resolve the ESP32's mDNS hostname, and for the subnet-scan fallback
import subprocess      # used to run "arp -a" to read the OS's ARP cache (IP <-> MAC mappings)
import threading        # each camera runs its own background thread so multiple cameras work at once
import time             # used for FPS calculation, sleeping between retries, and timestamps
from concurrent.futures import ThreadPoolExecutor   # scans the local subnet in parallel, not one host at a time

import cv2             # OpenCV: decodes JPEG bytes into images, draws boxes/text, re-encodes to JPEG
import numpy as np     # used to turn raw bytes into an array OpenCV can decode, and to build the FPS bar
import requests        # used to open an HTTP connection to the ESP32-CAM's MJPEG stream
from flask import Blueprint, Flask, Response, jsonify, render_template_string
from ultralytics import YOLO   # the YOLO model class -- loads your .pt or .onnx file and runs detection


# ============================== CONFIG ==============================
# Everything you're likely to want to change lives in this one section.

# One entry per physical ESP32-CAM. "name" is used in the URL
# (e.g. /video_feed/cam1/) and must be unique. Each camera needs its
# own mDNS hostname set in its Arduino sketch (MDNS.begin("esp32cam2")
# for the second board, etc.) so they don't collide on the network.
#
# "mac" is the camera's hardware WiFi MAC address (printed once via
# Serial.println(WiFi.macAddress()) -- see setup()). This is the key
# to network-agnostic discovery: on ANY WiFi network you don't control
# (coffee shop, university, office, hotspot), if mDNS resolution of
# "hostname" fails, the code scans the current subnet and finds
# whichever device has this exact MAC address, regardless of what IP
# that network happened to hand it. Since the MAC is burned into the
# chip, this works unchanged everywhere -- no router access needed.
#
# "static_ip" is optional and only useful on networks you personally
# control and have reserved a DHCP lease on. Leave it as None if you
# don't have one -- the mac-based subnet scan below covers you on
# networks you don't control anyway.
ESP32_CAMERAS = [
    {"name": "cam1", "hostname": "esp32cam.local",
     "mac": "28:05:A5:67:03:04", "static_ip": None, "port": 81},
    {"name": "cam2", "hostname": "esp32cam2.local",
     "mac": "B0:CB:D8:E1:74:8C", "static_ip": None, "port": 81},
]

# Path to your trained model file. .onnx runs noticeably faster on CPU
# than .pt (this is why you exported to ONNX earlier) -- but .pt works
# too if you'd rather skip dealing with fixed input-size ONNX export.
MODULE_DIR = os.path.dirname(os.path.abspath(__file__))
CUSTOM_MODEL_PATH = os.path.join(MODULE_DIR, "models", "warehouse_objects_best.onnx")

# If using a .onnx file, this MUST match the imgsz you used when you
# ran model.export(format="onnx", imgsz=...) -- ONNX models have a
# fixed input size baked in, unlike .pt files which accept any size.
CUSTOM_INPUT_SIZE = 320

# Minimum confidence (0-1) for a detection to be shown at all. Raised
# from the default 0.5 earlier specifically to cut down on false
# positives you were seeing in testing.
CUSTOM_CONFIDENCE = 0.5

# Only run the (expensive) YOLO model every Nth frame. On frames in
# between, we just redraw the last known boxes on the new frame. This
# reduces inference work to one quarter (at N=4), at the cost of
# detections updating slightly less often -- a good trade for
# real-time viewing since the video still looks smooth.
DETECT_EVERY_N_FRAMES = 4

# Which of your model's class names should be treated as an "obstacle"
# for robot navigation. Must exactly match names from your training
# data.yaml (case-sensitive). Every class listed here gets special
# treatment: a fixed red bounding box color, and any of them can set
# obstacle_detected = True in the API / trigger the warning banner.
OBSTACLE_CLASS_NAMES = {"traffic_cone", "forklift"}

# How many mDNS resolution attempts to make (each ~1 second apart)
# before giving up and trying the static_ip fallback, if one is
# configured. Kept short (a few seconds) so that when mDNS genuinely
# doesn't work on this network, you're not stuck waiting through the
# old long retry/backoff loop before the fallback kicks in.
MDNS_QUICK_RETRIES = 4
MDNS_QUICK_RETRY_DELAY = 1  # seconds

# --- Subnet-scan (MAC-based) discovery settings ---
# Used as the fallback whenever mDNS fails AND a camera has "mac" set
# in ESP32_CAMERAS above. Works on any network without needing router
# access, because it identifies the camera by hardware MAC address
# rather than by hostname or a reserved IP.
SUBNET_SCAN_PORT_TIMEOUT = 0.4     # seconds to wait per host when probing the camera's port
SUBNET_SCAN_MAX_WORKERS = 100      # how many hosts to probe in parallel (254 hosts finishes in ~1-2s)
SUBNET_SCAN_RETRIES = 3            # how many full subnet scans to attempt (camera may still be booting)
SUBNET_SCAN_RETRY_DELAY = 2        # seconds to wait between subnet-scan attempts

# =====================================================================

object_detection_bp = Blueprint("object_detection", __name__)

# This stays lazy so starting the warehouse backend does not load the YOLO
# model until somebody opens the object-detection page or calls its API.
_model = None
_model_lock = threading.Lock()
# Ultralytics creates its predictor lazily on the first predict call. Both
# camera threads share one model, so serialize predictions to prevent a race
# that can create multiple ONNX Runtime sessions during that initialization.
_prediction_lock = threading.Lock()


def _get_model():
    """Load and return the shared YOLO model on first use."""
    global _model
    if _model is None:
        with _model_lock:
            if _model is None:
                print(f"Loading YOLO model: {CUSTOM_MODEL_PATH} ...")
                _model = YOLO(CUSTOM_MODEL_PATH)
    return _model

# Colors (in OpenCV's BGR order, NOT RGB!) used for drawing bounding
# boxes. Cycled by class ID so each of your non-obstacle classes gets
# a consistent, distinct color every time.
COLOR_PALETTE = [
    (66, 133, 244),   # blue
    (52, 168, 83),    # green
    (251, 188, 5),    # yellow
    (154, 62, 226),   # purple
]

# Reserved specifically for the obstacle class -- solid red, so it
# reads as a clear visual warning distinct from the "normal" palette
# colors above.
WARNING_COLOR = (0, 0, 255)  # red in BGR order


def _color_for_class(cls_id, names):
    """Returns the BGR color to use for a given detected class.

    Any class in OBSTACLE_CLASS_NAMES always gets WARNING_COLOR (red),
    checked by NAME rather than by class ID number -- this way, if you
    retrain the model later and the class IDs happen to get reassigned
    (e.g. traffic_cone becomes class 2 instead of class 0), the
    warning color still follows the correct classes automatically.

    Every other class just cycles through COLOR_PALETTE based on its
    ID, using modulo so it wraps around safely even if you add more
    classes than there are colors in the palette.
    """
    if names.get(cls_id) in OBSTACLE_CLASS_NAMES:
        return WARNING_COLOR
    return COLOR_PALETTE[cls_id % len(COLOR_PALETTE)]


def _draw_boxes(frame, boxes, confs, clss, names):
    """Draws all detection boxes + labels directly onto `frame` (in place)
    and returns it.

    Parameters:
      frame  - the image (numpy array) to draw on
      boxes  - list of [x1, y1, x2, y2] pixel coordinates, one per detection
      confs  - list of confidence scores (0-1), one per detection
      clss   - list of class ID integers, one per detection
      names  - dict mapping class ID -> class name string (comes from the model)

    All four lists are the same length and correspond index-for-index
    (boxes[i] goes with confs[i], clss[i], etc.) -- this is the raw
    format Ultralytics gives back from a prediction.
    """
    for (x1, y1, x2, y2), conf, cls_id in zip(boxes, confs, clss):
        # Box coordinates come back as floats; pixel drawing needs ints.
        x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
        color = _color_for_class(cls_id, names)
        label = f"{names[cls_id]}: {conf:.2f}"   # e.g. "traffic_cone: 0.91"

        # Draw the box outline itself.
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

        # Figure out how big the label text will be, so we can size a
        # background rectangle for it that comfortably fits the text.
        (text_w, text_h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
        # Clip the label background to the frame's edges, in case the
        # box is right at the edge of the image (avoids drawing off-frame).
        bx2 = min(x1 + text_w + 10, frame.shape[1])
        by2 = min(y1 + text_h + 12, frame.shape[0])

        # Draw the label background as SEMI-TRANSPARENT rather than
        # solid: we draw it on a copy of the frame, then blend that
        # copy back into the real frame at 55% opacity. This keeps the
        # label readable without fully hiding whatever object is
        # sitting underneath the label (e.g. the pallet under its own
        # "loaded_pallet: 0.95" tag).
        overlay = frame.copy()
        cv2.rectangle(overlay, (x1, y1), (bx2, by2), color, -1)  # -1 thickness = filled
        cv2.addWeighted(overlay, 0.55, frame, 0.45, 0, frame)

        # Draw the label text itself, in black, on top of the
        # semi-transparent colored background. Placed just inside the
        # box's top-left corner (x1+5, not x1) so it doesn't sit
        # exactly on the box outline.
        cv2.putText(frame, label, (x1 + 5, y1 + text_h + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
    return frame


def _add_top_bar(frame, text):
    """Adds a solid black strip above the camera frame and writes `text`
    into it (used for the "camX  FPS: NN.N" overlay).

    We do this instead of just writing text directly onto the video
    image because text drawn over the live picture can end up
    overlapping and obscuring part of the actual scene. A dedicated
    bar above the image guarantees the overlay never covers real
    content.

    np.vstack "stacks" two images vertically -- the small black bar on
    top, the actual camera frame below -- producing one taller image.
    """
    bar_height = 30
    h, w = frame.shape[:2]                     # frame.shape is (height, width, channels)
    bar = np.zeros((bar_height, w, 3), dtype=np.uint8)   # a solid black image, same width as the frame
    cv2.putText(bar, text, (10, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    return np.vstack([bar, frame])


def _normalize_mac(mac):
    """Turns any MAC address formatting ("28:05:A5:67:03:04",
    "28-05-a5-67-03-04", etc.) into a single comparable form: lowercase,
    no separators. This is needed because the MAC you copy from Serial
    Monitor uses colons, but Windows' "arp -a" output uses hyphens --
    without normalizing, a perfectly matching MAC would look "different"
    and never match.
    """
    return re.sub(r"[:\-]", "", mac).strip().lower()


def _get_local_subnet_base():
    """Figures out the first three octets of the subnet this PC is
    currently on (e.g. "192.168.1" from IP "192.168.1.42"), so we know
    which /24 range to scan for the camera.

    The trick: opening a UDP "connection" to a public IP doesn't
    actually send any packets (UDP is connectionless) -- it just asks
    the OS to pick which local network interface/IP it WOULD use to
    reach that address. This reliably gives us this PC's real,
    currently-active WiFi IP, regardless of which network we're on or
    whether other network interfaces are also active.

    Assumes a /24 (i.e. a "x.x.x.0/24") subnet, which covers the
    overwhelming majority of home, office, and public WiFi networks.
    """
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(2)
        s.connect(("8.8.8.8", 80))
        local_ip = s.getsockname()[0]
        s.close()
        return ".".join(local_ip.split(".")[:3])
    except OSError:
        return None


def _read_arp_table():
    """Reads the PC's own ARP cache (IP <-> MAC mappings the operating
    system already knows about) and returns it as a dict of
    {ip_address: mac_address}.

    Every time this PC successfully talks to another device on the
    same local network, the OS automatically records that device's
    MAC address here -- this is a normal, built-in part of how IP
    networking works on Ethernet/WiFi, not something we're doing
    specially. We just read it back out after probing the subnet.

    Uses the Windows "arp -a" command specifically (this project runs
    on Windows 11 per your setup). The output looks like:

        Interface: 192.168.1.100 --- 0x10
          Internet Address      Physical Address      Type
          192.168.1.55           a4-cf-12-34-56-78     dynamic
          192.168.1.1            10-20-30-40-50-60     dynamic
    """
    try:
        output = subprocess.check_output(
            ["arp", "-a"], text=True, timeout=5,
            creationflags=subprocess.CREATE_NO_WINDOW if hasattr(subprocess, "CREATE_NO_WINDOW") else 0,
        )
    except (subprocess.SubprocessError, OSError):
        return {}

    entries = {}
    ip_re = re.compile(r"^\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3}$")
    mac_re = re.compile(r"^[0-9a-fA-F]{2}([:-][0-9a-fA-F]{2}){5}$")

    for line in output.splitlines():
        parts = line.split()
        if len(parts) >= 2 and ip_re.match(parts[0]) and mac_re.match(parts[1]):
            entries[parts[0]] = parts[1]

    return entries


def _scan_subnet_for_mac(target_mac, port):
    """Scans every host (1-254) on the current /24 subnet by attempting
    a quick TCP connection to `port` on each one, in parallel. This
    doesn't need the camera's port to actually respond correctly -- the
    mere act of the OS trying to reach that IP on the local network is
    enough to make it resolve (and cache) that device's MAC address via
    ARP, which is what we actually care about.

    After probing, reads back the OS's ARP cache and looks for an entry
    whose MAC matches `target_mac`. Returns that device's current IP if
    found, or None if no match turned up in this pass (the camera might
    not be booted yet, or might not be on this subnet at all).
    """
    base = _get_local_subnet_base()
    if base is None:
        return None

    candidates = [f"{base}.{i}" for i in range(1, 255)]

    def probe(ip):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(SUBNET_SCAN_PORT_TIMEOUT)
        try:
            s.connect_ex((ip, port))
        except OSError:
            pass
        finally:
            s.close()

    with ThreadPoolExecutor(max_workers=SUBNET_SCAN_MAX_WORKERS) as pool:
        list(pool.map(probe, candidates))

    target_norm = _normalize_mac(target_mac)
    for ip, mac in _read_arp_table().items():
        if _normalize_mac(mac) == target_norm:
            return ip

    return None


class CameraEngine:
    """Owns everything related to ONE physical ESP32-CAM:
      - a background thread that continuously pulls frames from its
        MJPEG stream, runs detection, and stores the latest annotated
        JPEG in memory
      - the current obstacle-detection status for this camera
      - thread-safe accessors so Flask's request-handling threads
        (which run separately from this camera's background thread)
        can safely read the latest frame / obstacle status without
        data races

    One CameraEngine instance exists per camera name (see get_engine()
    below), created the first time that camera is accessed and reused
    after that.
    """

    def __init__(self, name, hostname, port, mac=None, static_ip=None):
        self.name = name           # e.g. "cam1" -- used in URLs and printed logs
        self.hostname = hostname   # e.g. "esp32cam.local" -- the mDNS hostname to resolve
        self.port = port           # e.g. 81 -- the port the ESP32's MJPEG stream is served on
        self.mac = mac             # e.g. "28:05:A5:67:03:04" -- used for subnet-scan discovery, or None
        self.static_ip = static_ip # e.g. "192.168.110.55" -- fallback IP if mDNS fails, or None

        # --- Shared state for the video frame itself ---
        # frame_lock protects everything below it from being read and
        # written at the same time by different threads (this camera's
        # own capture thread writes it; Flask's viewer threads read it).
        self.frame_lock = threading.Lock()
        self.latest_jpeg_bytes = None   # the most recent annotated frame, already JPEG-encoded
        self.frame_version = 0          # incremented every time latest_jpeg_bytes changes;
                                         # lets viewers detect "is there a NEW frame yet?"
                                         # without needing to compare the (large) bytes directly
        self.fps = 0.0                  # current measured frames-per-second, recalculated once per second
        self.status = "starting"        # human-readable current state, useful for debugging

        # --- Cached detection results, reused between detection passes ---
        # Since DETECT_EVERY_N_FRAMES skips running the model on most
        # frames, we need to remember the LAST result so we can keep
        # drawing boxes on the frames where we didn't re-run detection.
        self.last_boxes, self.last_confs, self.last_clss, self.last_names = [], [], [], {}
        self.frame_counter = 0   # counts frames seen so far, used to decide when to run detection

        # --- Obstacle status: kept SEPARATE from the frame state above ---
        # This is deliberately its own isolated piece of data with its
        # own lock, rather than being bundled into the frame/JPEG
        # state. That separation is what makes it "modular": a
        # teammate's code (or the /api/obstacle route below) only ever
        # needs to read THIS small dict-like state -- it never needs
        # to know anything about JPEG bytes, OpenCV, or YOLO at all.
        self.obstacle_lock = threading.Lock()
        self.obstacle_detected = False       # True/False: is ANY obstacle class currently visible?
        self.obstacle_object = None          # name of the obstacle class currently driving that flag
        self.obstacle_confidence = 0.0       # confidence score of the most recent obstacle sighting
        self.obstacle_last_updated = None    # unix timestamp of the last time this was checked

        # Start the background thread immediately. daemon=True means
        # this thread will not prevent the whole program from exiting
        # (so Ctrl+C / stopping the server works cleanly).
        self.thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.thread.start()

    def _resolve_ip(self, retries=10, delay=3):
        """Resolves this camera's IP address using up to three stages,
        tried in order, so the SAME config works unmodified on any
        network -- your phone hotspot, an office router, a coffee shop,
        a university, anywhere:

          1. mDNS (fast path). Works on networks that forward multicast
             traffic, like your phone hotspot. Tried first because it's
             quick when it works.

          2. Subnet-scan by MAC address (network-agnostic fallback).
             If mDNS fails and `self.mac` is set, scans every host on
             the current /24 subnet and matches against the camera's
             hardware MAC address (which never changes, no matter what
             network or IP it's on). This needs no router access at
             all -- it works on any network EXCEPT ones with client/AP
             isolation enabled, where devices genuinely cannot reach
             each other regardless of what software does.

          3. Static IP fallback, if configured. Only useful on networks
             you've personally set up a DHCP reservation on.

        If none of these succeed, raises with a clear explanation of
        what was tried.
        """
        # --- Stage 1: quick mDNS attempts ---
        have_fallback = bool(self.mac or self.static_ip)
        quick_attempts = MDNS_QUICK_RETRIES if have_fallback else retries
        quick_delay = MDNS_QUICK_RETRY_DELAY if have_fallback else delay

        for attempt in range(1, quick_attempts + 1):
            try:
                ip = socket.gethostbyname(self.hostname)
                self.status = f"resolved {self.hostname} -> {ip} via mDNS"
                return ip
            except socket.gaierror:
                self.status = (f"waiting for {self.hostname} via mDNS "
                                f"(attempt {attempt}/{quick_attempts})")
                time.sleep(quick_delay)

        # --- Stage 2: subnet scan by MAC address ---
        if self.mac:
            for attempt in range(1, SUBNET_SCAN_RETRIES + 1):
                self.status = (f"mDNS failed -- scanning subnet for MAC "
                                f"{self.mac} (attempt {attempt}/{SUBNET_SCAN_RETRIES})")
                print(f"[{self.name}] {self.status}")
                ip = _scan_subnet_for_mac(self.mac, self.port)
                if ip:
                    self.status = f"found {self.mac} -> {ip} via subnet scan"
                    print(f"[{self.name}] {self.status}")
                    return ip
                time.sleep(SUBNET_SCAN_RETRY_DELAY)

        # --- Stage 3: static IP fallback, if configured ---
        if self.static_ip:
            self.status = (f"mDNS and subnet scan both failed -- falling "
                            f"back to static IP {self.static_ip}")
            print(f"[{self.name}] {self.status}")
            return self.static_ip

        # --- Nothing worked ---
        raise RuntimeError(
            f"Could not resolve camera '{self.name}': mDNS lookup of "
            f"{self.hostname} failed"
            + (f", subnet scan for MAC {self.mac} found no match" if self.mac else "")
            + (", and no static_ip is configured" if not self.static_ip else "")
            + ". If this network has client/AP isolation enabled, no "
              "software fix on this PC can reach the camera -- try a "
              "network you control instead (e.g. your phone hotspot)."
        )

    def _detect_and_draw(self, frame):
        """Given one raw camera frame, either runs YOLO detection on it
        (every DETECT_EVERY_N_FRAMES-th frame) or reuses the last
        detection result (on the frames in between), then draws
        whichever boxes are current onto the frame and returns it.
        """
        self.frame_counter += 1

        # Only actually run the (CPU-expensive) model every Nth frame.
        if self.frame_counter % DETECT_EVERY_N_FRAMES == 0:
            # model.predict() runs the neural network forward pass and
            # returns a list of Results objects -- we only ever pass
            # in one frame at a time, so we always want results[0].
            model = _get_model()
            with _prediction_lock:
                results = model.predict(frame, imgsz=CUSTOM_INPUT_SIZE,
                                        conf=CUSTOM_CONFIDENCE, verbose=False)
            r = results[0]

            # r.boxes can be None if literally nothing was detected in
            # this frame -- guard against that before pulling data out
            # of it. When boxes exist, .xyxy gives pixel coordinates
            # (as opposed to .xywh, a different coordinate format), and
            # everything is converted from PyTorch tensors (.cpu()
            # moves off any GPU, .numpy() converts to a plain array)
            # into plain Python-friendly numpy arrays.
            self.last_boxes = r.boxes.xyxy.cpu().numpy() if r.boxes is not None else []
            self.last_confs = r.boxes.conf.cpu().numpy() if r.boxes is not None else []
            self.last_clss = r.boxes.cls.cpu().numpy().astype(int) if r.boxes is not None else []
            self.last_names = model.names   # dict of {class_id: class_name}, e.g. {0: 'forklift', ...}

            # Update obstacle status right after getting a FRESH
            # detection result (not on skipped frames -- the obstacle
            # flag just keeps its last known value until the next real
            # detection pass updates it, which is fine given detection
            # still runs every 2nd frame).
            self._update_obstacle_status()

        # Whether we just ran detection or are reusing cached results,
        # draw whatever the current boxes are onto this frame.
        return _draw_boxes(frame, self.last_boxes, self.last_confs, self.last_clss, self.last_names)

    def _update_obstacle_status(self):
        """Looks at the most recent set of detections and figures out
        whether any class in OBSTACLE_CLASS_NAMES (traffic_cone,
        forklift, ...) is currently present. Kept as its own separate
        method -- rather than being buried inside _detect_and_draw --
        so this specific piece of logic is easy to find, read, and
        modify independently of the drawing/streaming code around it.
        """
        detected = False
        confidence = 0.0
        detected_object = None

        # Walk through every current detection; if ANY of them is one
        # of the obstacle classes, mark detected=True and remember
        # whichever obstacle sighting has the highest confidence (in
        # case there happen to be multiple -- e.g. a forklift AND a
        # traffic_cone -- in the same frame).
        for conf, cls_id in zip(self.last_confs, self.last_clss):
            name = self.last_names.get(cls_id)
            if name in OBSTACLE_CLASS_NAMES and float(conf) >= confidence:
                detected = True
                confidence = float(conf)
                detected_object = name

        # obstacle_lock protects this write from colliding with a read
        # happening at the same moment from a Flask request thread
        # (e.g. a teammate's code hitting /api/obstacle right now).
        with self.obstacle_lock:
            self.obstacle_detected = detected
            self.obstacle_object = detected_object
            self.obstacle_confidence = confidence
            self.obstacle_last_updated = time.time()

    def get_obstacle_status(self):
        """Returns a plain dict describing this camera's current
        obstacle status, safe to call from ANY thread (e.g. a Flask
        request handler running on a totally different thread than
        this camera's own capture loop).

        This is intentionally the ONE place other code should go to
        find out "is there an obstacle on this camera right now" --
        both the /api/obstacle Flask route below, and (if a teammate's
        code ever runs inside this same Python process) any other
        Python code, should call this method rather than reaching into
        obstacle_detected / obstacle_confidence directly.
        """
        with self.obstacle_lock:
            return {
                "camera": self.name,
                "obstacle_detected": self.obstacle_detected,
                "object": self.obstacle_object,
                "confidence": round(self.obstacle_confidence, 3),
                "last_updated": self.obstacle_last_updated,
            }

    def _capture_loop(self):
        """Runs forever in a background thread (one per camera). This
        is the heart of the whole system: connect to the camera's
        stream, continuously read frames, run detection, and store the
        latest annotated frame for viewers to pick up.

        Wrapped in an outer while-True + try/except so that if
        anything goes wrong (camera disconnects, network hiccup,
        stream ends), we log the error and automatically retry after a
        few seconds, rather than the thread just dying silently.
        """
        frame_count = 0        # frames processed since the last FPS calculation
        fps_timer = time.time()  # when we last recalculated FPS

        while True:
            try:
                # Find the camera's current IP (mDNS, falling back to
                # static_ip if configured -- see _resolve_ip above) and
                # open the MJPEG stream.
                ip = self._resolve_ip()
                stream_url = f"http://{ip}:{self.port}/stream"
                self.status = f"connected to {stream_url}"
                print(f"[{self.name}] Connecting to {stream_url} ...")

                # stream=True means requests won't try to download the
                # whole (infinite) response at once -- it lets us read
                # it incrementally via iter_content below.
                resp = requests.get(stream_url, stream=True, timeout=5)
                byte_buffer = b""   # accumulates raw bytes until we find a complete JPEG image

                # An MJPEG stream is just a continuous sequence of JPEG
                # images sent one after another. We read raw chunks of
                # bytes and look for JPEG "start" (0xFFD8) and "end"
                # (0xFFD9) markers to find where one complete image is.
                for chunk in resp.iter_content(chunk_size=4096):
                    byte_buffer += chunk

                    # IMPORTANT: use rfind (search from the END) for the
                    # start marker. If our processing has fallen behind
                    # the camera's frame rate, multiple complete JPEGs
                    # can pile up in byte_buffer. Using rfind means we
                    # always jump straight to the NEWEST complete frame
                    # and silently discard any older, stale ones --
                    # this is what prevents growing lag over time.
                    last_start = byte_buffer.rfind(b'\xff\xd8')
                    end = byte_buffer.find(b'\xff\xd9', last_start)

                    if last_start == -1 or end == -1:
                        # We don't have a complete JPEG yet -- keep
                        # accumulating bytes. But guard against the
                        # buffer growing forever if something's wrong
                        # (e.g. markers never found) by clearing it
                        # past a safety size limit.
                        if len(byte_buffer) > 500_000:
                            byte_buffer = b""
                        continue

                    # Extract just the one complete JPEG we found, and
                    # keep only whatever bytes came after it (which
                    # belong to the NEXT frame).
                    jpg = byte_buffer[last_start:end + 2]
                    byte_buffer = byte_buffer[end + 2:]

                    # Decode the raw JPEG bytes into an actual image
                    # (a numpy array of pixel values) that OpenCV/YOLO
                    # can work with.
                    frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                    if frame is None:
                        # Decoding can occasionally fail on a corrupted
                        # frame (e.g. if it got cut off mid-transfer);
                        # just skip it and move on to the next one.
                        continue

                    # Run detection (or reuse cached boxes) and draw
                    # them onto the frame.
                    annotated = self._detect_and_draw(frame)

                    # --- FPS calculation ---
                    # Rather than computing FPS every single frame
                    # (which would be noisy), we count frames and
                    # recalculate the rate once per second.
                    frame_count += 1
                    now = time.time()
                    if now - fps_timer >= 1.0:
                        self.fps = frame_count / (now - fps_timer)
                        frame_count = 0
                        fps_timer = now

                    # Add the "camX  FPS: NN.N" bar above the image.
                    annotated = _add_top_bar(annotated, f"{self.name}  FPS: {self.fps:.1f}")

                    # Re-encode the finished, annotated frame back into
                    # JPEG bytes -- this is the format that gets sent
                    # to browsers via the MJPEG stream. We do this
                    # ENCODING ONCE HERE (not once per viewer!) so that
                    # if 10 people are watching this camera at once,
                    # they all share this one encoded frame rather than
                    # each doing their own (expensive) encoding work.
                    ok, jpeg = cv2.imencode('.jpg', annotated)
                    if ok:
                        with self.frame_lock:
                            self.latest_jpeg_bytes = jpeg.tobytes()
                            self.frame_version += 1   # tells viewers "there's a new frame"

            except Exception as e:
                # Something went wrong (camera offline, network drop,
                # etc.) -- log it, wait a few seconds, then the outer
                # while-True loop will try to reconnect from scratch.
                self.status = f"error: {e}"
                print(f"[{self.name}] Stream error: {e}. Retrying in 3 seconds...")
                time.sleep(3)

    def get_frame_if_new(self, last_seen_version):
        """Called by each viewer (once per loop iteration in the
        video_feed route below) to check "is there a frame newer than
        the last one I already sent to this particular viewer?"

        Returns (jpeg_bytes, new_version) if there IS a newer frame, or
        (None, last_seen_version) if not -- unchanged, so the caller
        knows to just wait and check again shortly.

        This version-tracking is important: without it, every viewer
        would busy-loop re-sending the SAME frame over and over as
        fast as possible, wasting CPU and starving the actual capture/
        detection thread of processing time (this caused a real FPS
        collapse when testing with multiple simultaneous viewers).
        """
        with self.frame_lock:
            if self.frame_version != last_seen_version and self.latest_jpeg_bytes is not None:
                return self.latest_jpeg_bytes, self.frame_version
            return None, last_seen_version


# --- Camera engine registry ---
# Maps camera name -> its CameraEngine instance. Engines are created
# lazily (the first time a camera is actually requested/viewed) rather
# than all at startup, via get_engine() below.
_engines = {}
_engines_lock = threading.Lock()   # protects _engines from being modified by two threads at once


def get_engine(camera_name):
    """Returns the CameraEngine for the given camera name, CREATING it
    (and starting its background capture thread) the first time this
    camera is requested. Subsequent calls for the same camera name
    just return the already-running engine -- so opening the page
    twice, or having multiple viewers, does NOT start duplicate
    capture threads or load the model twice.
    """
    if camera_name not in _engines:
        with _engines_lock:
            # Double-checked locking: check again inside the lock,
            # in case another thread created the engine between our
            # first check above and acquiring the lock just now.
            if camera_name not in _engines:
                camera_config = next((c for c in ESP32_CAMERAS if c["name"] == camera_name), None)
                if camera_config is None:
                    raise ValueError(f"Unknown camera: {camera_name}")
                _engines[camera_name] = CameraEngine(
                    name=camera_config["name"],
                    hostname=camera_config["hostname"],
                    port=camera_config["port"],
                    mac=camera_config.get("mac"),
                    static_ip=camera_config.get("static_ip"),
                )
    return _engines[camera_name]


# ============================== ROUTES ==============================
# Everything below this point defines what the web server actually
# responds with for each URL a browser (or other client) might request.

# The entire HTML page is stored as one big string here (rather than a
# separate template file) since this whole project is meant to be a
# single Python file. Flask's render_template_string() fills in the
# {{ ... }} placeholders using the `cameras` list, same as a normal
# Jinja2 template file would.
INDEX_HTML = """
<!DOCTYPE html>
<html>
<head>
    <title>ESP32-CAM Live Object Detection</title>
    <style>
        * { box-sizing: border-box; }
        body {
            margin: 0; min-height: 100vh; padding: 24px clamp(24px, 3vw, 46px);
            background: #f2f5fa; color: #172033;
            font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
        }
        .page-shell { max-width: 1280px; margin: 0 auto; }
        .panel-heading { display: flex; justify-content: space-between; gap: 24px; align-items: center; margin-bottom: 22px; }
        .eyebrow { margin: 0 0 2px; color: #3973df; text-transform: uppercase; letter-spacing: .11em; font-size: 10px; font-weight: 800; }
        h1 { margin: 0; color: #172033; font-size: clamp(26px, 4vw, 30px); letter-spacing: -.04em; font-weight: 500; }
        .header-actions { display: flex; align-items: center; gap: 10px; }
        .live-status { display: inline-flex; align-items: center; gap: 7px; padding: 6px 10px; border: 1px solid #cae8d7; border-radius: 999px; background: #f0fdf4; color: #167344; font-size: 12px; font-weight: 700; white-space: nowrap; }
        .live-status i { width: 7px; height: 7px; border-radius: 50%; background: #25a867; box-shadow: 0 0 0 3px #d9f5e4; }
        .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(300px, 1fr)); gap: 18px; align-items: start; }

        .camera-box {
            min-width: 0; padding: 16px; border: 1px solid #e5eaf2; border-radius: 14px;
            background: #fff; box-shadow: 0 8px 24px #21314b0d;
        }
        .camera-box h2 {
            margin: 0 0 12px; color: #25324a; font-size: 14px; font-weight: 800;
            text-transform: uppercase; letter-spacing: .06em;
        }
        img { width: 100%; height: auto; display: block; border: 1px solid #d7dfeb; border-radius: 9px; background: #172033; }

        .warning-banner {
            margin-top: 12px; padding: 9px 11px; border: 1px solid #ffcaca; border-radius: 7px;
            background: #fff0f0; color: #b42318; font-weight: 800; font-size: 12px;
            display: none; width: 100%;
        }
        .warning-banner.active { display: block; }
        .back-link {
            padding: 7px 10px; border-radius: 6px; background: #172f62; color: #fff;
            font-size: 12px; font-weight: 700; text-decoration: none; white-space: nowrap;
        }
        .back-link:hover { background: #0f2450; }
        @media (max-width: 650px) {
            body { padding: 20px; }
            .panel-heading { align-items: flex-start; flex-direction: column; }
        }
    </style>
</head>
<body>
    <main class="page-shell">
    <header class="panel-heading">
        <div>
            <p class="eyebrow">Live vision monitor</p>
            <h1>Robots' Live View</h1>
        </div>
        <div class="header-actions">
            <span class="live-status"><i></i>Camera service active</span>
            <a class="back-link" href="{{ website_url }}">Back to warehouse</a>
        </div>
    </header>
    <section class="grid">
        {% for camera in cameras %}
        <div class="camera-box">
            <h2>{{ camera.name }}</h2>
            <!-- The page refreshes this image with the newest annotated
                 snapshot, which works reliably with the Waitress server. -->
            <img
                id="feed-{{ camera.name }}"
                src="{{ url_for('object_detection.camera_snapshot', camera_name=camera.name) }}"
                alt="{{ camera.name }} live feed"
            >

            <!-- One warning banner per camera, initially hidden.
                 The JS below shows/hides this based on obstacle status. -->
            <div class="warning-banner" id="warning-{{ camera.name }}">
                Obstacle detected on {{ camera.name|upper }}
            </div>
        </div>
        {% endfor %}
    </section>
    </main>
    <script>
        // Build a plain JS list of camera names from the Jinja2
        // `cameras` list, so the polling loop below knows which
        // cameras to check without hardcoding "cam1"/"cam2" in JS.
        const cameraNames = {{ cameras|map(attribute='name')|list|tojson }};
        const snapshotUrlTemplate = "{{ url_for('object_detection.camera_snapshot', camera_name='CAMERA_NAME') }}";

        // Waitress serves a latest JPEG snapshot much more reliably than a
        // browser-held MJPEG response. Refreshing the already-annotated frame
        // ten times per second keeps the monitor responsive without holding a
        // Waitress worker open for every camera viewer.
        function refreshFrames() {
            const cacheBuster = Date.now();
            for (const name of cameraNames) {
                const image = document.getElementById(`feed-${name}`);
                image.src = `${snapshotUrlTemplate.replace('CAMERA_NAME', name)}?t=${cacheBuster}`;
            }
        }

        // Checks every camera's current obstacle status and shows/
        // hides the corresponding warning banner. This is completely
        // separate from the video <img> tags above -- the video
        // stream updates itself continuously on its own, while this
        // function is the only thing responsible for the text
        // warning banners.
        async function pollObstacles() {
            for (const name of cameraNames) {
                try {
                    const resp = await fetch(`{{ url_for('object_detection.obstacle_status', camera_name='CAMERA_NAME') }}`.replace('CAMERA_NAME', name));
                    const data = await resp.json();
                    const banner = document.getElementById(`warning-${name}`);
                    // !!data.obstacle_detected converts the value to a
                    // strict true/false, in case it ever comes back as
                    // something else (like null).
                    banner.classList.toggle('active', !!data.obstacle_detected);
                } catch (err) {
                    // If the fetch fails (e.g. brief network hiccup),
                    // just ignore it silently -- the next poll (half a
                    // second later) will simply try again.
                }
            }
        }

        // Check obstacle status twice a second, forever, starting
        // immediately (the extra pollObstacles() call outside
        // setInterval means we don't wait 500ms before the FIRST check).
        setInterval(pollObstacles, 500);
        pollObstacles();
        setInterval(refreshFrames, 100);
        refreshFrames();
    </script>
</body>
</html>
"""


@object_detection_bp.route("/")
def index():
    """The main page: shows both camera feeds side by side with their
    warning banners. Just renders the HTML template above, passing in
    the list of configured cameras so the template can loop over them."""
    website_url = os.environ.get("WAREHOUSE_WEBSITE_URL", "http://localhost:5173")
    return render_template_string(
        INDEX_HTML, cameras=ESP32_CAMERAS, website_url=website_url
    )


@object_detection_bp.route("/video_feed/<camera_name>/")
def video_feed(camera_name):
    """The actual MJPEG video stream for one camera. This is what the
    <img> tag in the HTML page points at.

    Returns a StreamingResponse (via Flask's Response with a generator
    function) rather than a normal one-shot response -- the connection
    stays open indefinitely, continuously sending new frame data as it
    becomes available, which is how MJPEG streaming over HTTP works.
    """
    try:
        engine = get_engine(camera_name)
    except ValueError:
        return f"Unknown camera: {camera_name}", 404

    def generate():
        """A generator function: each time Flask asks it for the next
        chunk of the response, it yields one JPEG frame wrapped in the
        multipart boundary format that MJPEG streaming requires.

        last_version starts at -1 (a value frame_version will never
        naturally be) so the very first call always counts as "new".
        """
        last_version = -1
        while True:
            jpeg_bytes, last_version = engine.get_frame_if_new(last_version)
            if jpeg_bytes is None:
                # No new frame yet -- wait briefly before checking
                # again, rather than busy-looping and wasting CPU.
                time.sleep(0.02)
                continue
            # The exact byte format required by the
            # "multipart/x-mixed-replace" MIME type: a boundary marker,
            # a Content-Type header for this part, a blank line, then
            # the raw JPEG bytes themselves.
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + jpeg_bytes + b'\r\n')

    return Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frame")


@object_detection_bp.route("/snapshot/<camera_name>.jpg")
def camera_snapshot(camera_name):
    """Return the newest annotated JPEG without holding a worker open."""
    try:
        engine = get_engine(camera_name)
    except ValueError:
        return jsonify({"error": f"Unknown camera: {camera_name}"}), 404

    jpeg_bytes, _ = engine.get_frame_if_new(-1)
    if jpeg_bytes is None:
        return jsonify({"status": "Waiting for the first camera frame"}), 503

    return Response(
        jpeg_bytes,
        mimetype="image/jpeg",
        headers={"Cache-Control": "no-store, max-age=0"},
    )


@object_detection_bp.route("/api/obstacle/<camera_name>")
def obstacle_status(camera_name):
    """Modular JSON endpoint: returns current obstacle status for ONE camera.

    This is the integration point for a teammate's robot-navigation
    code (or any other system) -- they just need to make an HTTP GET
    request to this URL from wherever their code runs; they never need
    to see or touch any of the Python/OpenCV/YOLO code above this line.

    When mounted by the warehouse server:
        GET http://<this-PC-IP>:8000/object-detection/api/obstacle/cam1

    Example response:
        {
          "camera": "cam1",
          "obstacle_detected": true,
          "object": "traffic_cone",   (or "forklift", whichever was seen)
          "confidence": 0.91,
          "last_updated": 1732400000.123
        }
    """
    try:
        engine = get_engine(camera_name)
    except ValueError:
        return jsonify({"error": f"Unknown camera: {camera_name}"}), 404
    return jsonify(engine.get_obstacle_status())


@object_detection_bp.route("/api/obstacle")
def obstacle_status_all():
    """Modular JSON endpoint: returns current obstacle status for ALL
    configured cameras in a single response -- convenient if one
    consumer needs to check every robot's camera at once instead of
    making a separate request per camera.

    When mounted by the warehouse server:
        GET http://<this-PC-IP>:8000/object-detection/api/obstacle

    Example response:
        {
          "cam1": { "camera": "cam1", "obstacle_detected": false, ... },
          "cam2": { "camera": "cam2", "obstacle_detected": true, ... }
        }
    """
    return jsonify({
        cam["name"]: get_engine(cam["name"]).get_obstacle_status()
        for cam in ESP32_CAMERAS
    })


def create_object_detection_app():
    """Create a standalone app for running this module by itself."""
    standalone_app = Flask(__name__)
    standalone_app.register_blueprint(object_detection_bp)
    return standalone_app


if __name__ == "__main__":
    # This block only runs if you execute "python app.py" directly.
    # It's Flask's built-in development server -- fine for quick
    # testing, but NOT recommended for actually running this, since it
    # doesn't handle multiple simultaneous viewers/connections as
    # robustly as a real production server.
    #
    # For real use, run this file with waitress instead (note: when
    # using waitress, this __main__ block is skipped entirely --
    # waitress imports this file as a module and uses the `app` object
    # directly):
    #   pip install waitress
    #   waitress-serve --host=0.0.0.0 --port=8000 --threads=20 app:app
    port = int(os.environ.get("OBJECT_DETECTION_PORT", "8001"))
    create_object_detection_app().run(host="0.0.0.0", port=port, threaded=True)

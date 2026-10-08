import { useEffect, useState } from "react";
import TaskForm from "./components/TaskForm";
import RemainingRouteDisplay from "./features/remaining-route-display/RemainingRouteDisplay";
import API from "./api";
import amrWarehouseDesign from "./assets/warehouse-map/amr-warehouse.png";
import finalYearThesisDesign from "./assets/warehouse-map/final-year-thesis.png";
import "./App.css";

function App() {
  const [map, setMap] = useState(null);
  const [robots, setRobots] = useState({});
  const [error, setError] = useState("");
  const [warehouse, setWarehouse] = useState(null);
  const [warehouseError, setWarehouseError] = useState("");
  const [activeSidebarTab, setActiveSidebarTab] = useState("monitor");
  const [warehouseAlert, setWarehouseAlert] = useState(null);
  const [cameraUrl, setCameraUrl] = useState("");
  const [trafficControlEnabled, setTrafficControlEnabled] = useState(null);
  const [trafficControlBusy, setTrafficControlBusy] = useState(false);
  const [trafficControlError, setTrafficControlError] = useState("");

  useEffect(() => {
    const load = async () => {
      try {
        const [{ data: mapData }, { data: robotData }, { data: trafficData }] = await Promise.all([
          API.get("/map"),
          API.get("/robots"),
          API.get("/traffic-control"),
        ]);
        setMap(mapData);
        setRobots(robotData);
        setTrafficControlEnabled(trafficData.enabled);
        setError("");
      } catch {
        setError("Waiting for the backend at http://localhost:8000");
      }
    };
    load();
    // const timer = setInterval(load, 1000);
    // return () => clearInterval(timer);
  }, []);

  useEffect(() => {
    let active = true;
    let timer;
    const loadWarehouse = async () => {
      try {
        const { data } = await API.get("/warehouse/monitor");
        if (active) {
          setWarehouse(data);
          setWarehouseError("");
        }
      } catch {
        if (active) setWarehouseError("Warehouse data is unavailable");
      } finally {
        // Start the next refresh only after this request finishes. This keeps
        // a slow or locked SQLite request from piling up in Waitress.
        if (active) timer = setTimeout(loadWarehouse, 2000);
      }
    };
    loadWarehouse();
    return () => {
      active = false;
      clearInterval(timer);
      clearTimeout(timer);
    };
  }, []);

  useEffect(() => {
    const events = new EventSource("http://localhost:8000/robot-events");
    API.get("/camera-url").then(({ data }) => {
      if (typeof data.url === "string") setCameraUrl(data.url);
    }).catch(() => { });
    events.addEventListener("camera-url", (event) => {
      const { url } = JSON.parse(event.data);
      if (typeof url === "string") setCameraUrl(url);
    });
    events.addEventListener("robot-state", (event) => {
      const robot = JSON.parse(event.data);
      setRobots((current) => ({ ...current, [robot.robot_id]: robot }));
      setError("");
    });
    events.addEventListener("traffic-control", (event) => {
      const state = JSON.parse(event.data);
      setTrafficControlEnabled(state.enabled);
      setTrafficControlError("");
    });
    events.addEventListener("warehouse-alert", (event) => {
      const alert = JSON.parse(event.data);
      const alertTitles = {
        "duplicate-item": "Item already stored",
        "shelf-full": "Shelf capacity reached",
        "unrecognized-qr": "Unrecognized QR code",
      };
      setWarehouseAlert({
        title: alertTitles[alert.type] || "Warehouse alert",
        message: alert.message,
      });
    });
    events.onerror = () => setError("Live connection lost — retrying…");
    return () => events.close();
  }, []);

  const toggleTrafficControl = async () => {
    if (trafficControlEnabled === null || trafficControlBusy) return;
    setTrafficControlBusy(true);
    setTrafficControlError("");
    try {
      const { data } = await API.post("/traffic-control", {
        enabled: !trafficControlEnabled,
      });
      setTrafficControlEnabled(data.enabled);
      if (data.errors?.length) {
        setTrafficControlError(data.errors.map((item) => `${item.robot_id}: ${item.error}`).join("; "));
      }
    } catch (requestError) {
      const data = requestError.response?.data;
      if (typeof data?.enabled === "boolean") setTrafficControlEnabled(data.enabled);
      setTrafficControlError(data?.error || "Could not update traffic control");
    } finally {
      setTrafficControlBusy(false);
    }
  };

  return (
    <main className="dashboard">
      <section className="map-panel">
        <div className="panel-heading">
          <div>
            <p className="eyebrow">{activeSidebarTab === "warehouse" ? "Inventory operations" : activeSidebarTab === "task" ? "Task planning" : "Live MQTT monitor"}</p>
            <h1>{activeSidebarTab === "monitor" ? "Warehouse robots" : "Warehouse inventory"}</h1>
          </div>
          <div className="header-meta">
            <span className={error ? "connection offline" : "connection"}><i />{error || "Live MQTT connection"}</span>
            <button
              className={`traffic-control-toggle ${trafficControlEnabled ? "enabled" : "disabled"}`}
              type="button"
              aria-pressed={trafficControlEnabled === true}
              disabled={trafficControlEnabled === null || trafficControlBusy}
              onClick={toggleTrafficControl}
              title="When off, robot positions are still reported and shown, but cross-robot reservations are bypassed."
            >
              Traffic Control: {trafficControlEnabled === null ? "Loading…" : trafficControlEnabled ? "ON" : "OFF"}
            </button>
            <a className="object-detection-link" href={`http://${window.location.hostname || "localhost"}:8000/object-detection/`}>
              Live View
            </a>
            <button
              className="qr-scanner-link"
              type="button"
              disabled={!cameraUrl}
              onClick={() => window.open(cameraUrl, "_blank", "noopener,noreferrer")}
              title={cameraUrl ? "Open QR scanner" : "Waiting for camera IP from MQTT"}
            >
              QR Scanner View
            </button>
          </div>
        </div>
        {trafficControlError && <p className="traffic-control-error" role="alert">{trafficControlError}</p>}
        {activeSidebarTab === "warehouse" ? (
          <WarehouseMonitor warehouse={warehouse} map={map} error={warehouseError} />
        ) : activeSidebarTab === "task" ? (
          <WarehouseLocations warehouse={warehouse} map={map} error={warehouseError} />
        ) : map ? <WarehouseMap map={map} robots={robots} /> : (
          <p className="map-loading" role="status">{error || "Loading warehouse map…"}</p>
        )}
      </section>
      <aside className="task-panel">
        <div className="sidebar-tabs" role="tablist" aria-label="Warehouse controls">
          <button
            type="button"
            role="tab"
            aria-selected={activeSidebarTab === "monitor"}
            className={activeSidebarTab === "monitor" ? "active" : ""}
            onClick={() => setActiveSidebarTab("monitor")}
          >
            Robot Monitor
          </button>
          <button
            type="button"
            role="tab"
            aria-selected={activeSidebarTab === "task"}
            className={activeSidebarTab === "task" ? "active" : ""}
            onClick={() => setActiveSidebarTab("task")}
          >
            Create <br /> New Tasks
          </button>
          <button
            type="button"
            role="tab"
            aria-selected={activeSidebarTab === "warehouse"}
            className={activeSidebarTab === "warehouse" ? "active" : ""}
            onClick={() => setActiveSidebarTab("warehouse")}
          >
            Warehouse Logs
          </button>
        </div>
        {activeSidebarTab === "monitor" ? (
          <section className="side-monitor" aria-label="Robot state monitor">
            <div className="monitor-heading"><span>Robot monitor</span><small>Live state & queue</small></div>
            <div className="robot-list">
              {Object.values(robots).length === 0 ? (
                <p className="empty-queue">Waiting for robot status…</p>
              ) : Object.values(robots).map((robot) => (
                <article className={`robot-card ${robot.robot_id.toLowerCase()}`} key={robot.robot_id}>
                  <header className="robot-card-header">
                    <span className={`robot-dot ${robot.robot_id.toLowerCase()}`} />
                    <div><strong>{robot.robot_id}</strong><small>{robot.waiting_for_traffic ? "Waiting for traffic" : robot.status === "RETURNING_TO_PARKING" ? `Returning to ${robot.idle_goal || "parking"}` : robot.status} · {robot.node}</small></div>
                    <span>{Math.ceil(robot.estimated_seconds_until_free)}s free</span>
                  </header>
                  <dl className="robot-stats">
                    <div><dt>Current task</dt><dd>{robot.current_task_id ?? "None"}</dd></div>
                    <div><dt>Facing</dt><dd>{robot.heading_label ?? "Unknown"}</dd></div>
                    <div><dt>Payload</dt><dd>{robot.has_payload ? "Loaded" : "Empty"}</dd></div>
                    <div><dt>Queue</dt><dd>{robot.queue_length} task{robot.queue_length === 1 ? "" : "s"}</dd></div>
                    <div><dt>Distance since RFID</dt><dd>{robot.distance_since_last_node_cm == null ? "Waiting for encoder" : `${robot.distance_since_last_node_cm.toFixed(1)} cm${robot.encoder_distance_calibrated ? "" : " · estimate"}`}</dd></div>
                    <div><dt>Encoder ticks</dt><dd>{robot.encoder_telemetry ? `L ${robot.encoder_telemetry.ticks_L} · R ${robot.encoder_telemetry.ticks_R}` : "No telemetry"}</dd></div>
                  </dl>
                  <a
                    className="mission-history-download"
                    href={`${API.defaults.baseURL}/robots/${robot.robot_id}/mission-history.csv`}
                    download
                  >Download mission history CSV</a>
                  <RobotQueue
                    tasks={robot.queue}
                    currentTaskId={robot.current_task_id}
                    robotId={robot.robot_id}
                    onRobotUpdate={(updatedRobot) => setRobots((current) => ({
                      ...current,
                      [updatedRobot.robot_id]: updatedRobot,
                    }))}
                  />
                  <CompletedMissionHistory
                    robotId={robot.robot_id}
                    refreshKey={`${robot.current_task_id ?? "none"}:${(robot.queue || []).map((task) => `${task.id}:${task.status}`).join(",")}`}
                  />
                </article>
              ))}
            </div>
          </section>
        ) : activeSidebarTab === "warehouse" ? (
          <WarehouseLogFeed warehouse={warehouse} error={warehouseError} />
        ) : <TaskForm />}
      </aside>
      {warehouseAlert && (
        <div className="alert-backdrop" role="presentation">
          <section className="warehouse-alert-dialog" role="alertdialog" aria-modal="true" aria-labelledby="warehouse-alert-title" aria-describedby="warehouse-alert-message">
            <div className="alert-icon" aria-hidden="true">!</div>
            <div>
              <h2 id="warehouse-alert-title">{warehouseAlert.title}</h2>
              <p id="warehouse-alert-message">{warehouseAlert.message}</p>
            </div>
            <button type="button" autoFocus onClick={() => setWarehouseAlert(null)}>OK</button>
          </section>
        </div>
      )}
    </main>
  );
}

const ZONES = [
  { title: "Yellow", category: "Electronics", color: "#f4d35e" },
  { title: "Green", category: "Mechanical Parts", color: "#8fcb7b" },
  { title: "Purple", category: "Final Products", color: "#a9a0d1" },
  { title: "Orange", category: "Raw Materials", color: "#f2925c" },
];

function WarehouseMonitor({ warehouse, map, error }) {
  if (error) return <p className="warehouse-loading">{error}</p>;
  if (!warehouse) return <p className="warehouse-loading">Loading warehouse data…</p>;
  const { stats, shelves } = warehouse;
  return (
    <section className="warehouse-monitor" aria-label="Live warehouse monitor">
      <WarehouseLocationsContent map={map} shelves={shelves} />
      <div className="warehouse-cards">
        <MetricCard value={stats.total_items} label="Catalog items" />
        <MetricCard value={stats.in_stock} label="Currently in stock" />
        <MetricCard value={`${stats.occupied_shelves}/${stats.total_shelves}`} label="Shelves occupied" />
        <MetricCard value={stats.total_logs} label="Log entries" />
      </div>
    </section>
  );
}

function WarehouseLocations({ warehouse, map, error }) {
  if (error) return <p className="warehouse-loading">{error}</p>;
  if (!warehouse) return <p className="warehouse-loading">Loading warehouse locations…</p>;
  return (
    <section className="warehouse-locations-only" aria-label="Live warehouse inventory locations">
      <WarehouseLocationsContent map={map} shelves={warehouse.shelves} />
    </section>
  );
}

function WarehouseLocationsContent({ map, shelves }) {
  return (
    <>
      <div className="shelf-map-heading">
        <h2 className="shelf-map-title">Inventory locations</h2>
        <div className="shelf-map-legend">
          <span className="occupied-legend"><span className="occupied-swatches" aria-hidden="true">{ZONES.map((zone) => <i key={zone.title} title={zone.title} style={{ background: zone.color, borderColor: zone.color }} />)}</span>Occupied</span>
          <span><i className="vacant" aria-hidden="true" />Empty</span>
        </div>
      </div>
      {map ? <InventoryLocationMap map={map} shelves={shelves} /> : <p className="inventory-map-loading" role="status">Loading warehouse map…</p>}
    </>
  );
}

function InventoryLocationMap({ map, shelves }) {
  const shelfByNode = Object.fromEntries(shelves.map((shelf) => {
    const [category, number] = shelf.shelf_id.split("_");
    return [`${category[0].toUpperCase()}${category.slice(1)}_${number}`, shelf];
  }));
  return (
    <svg className="inventory-location-map" viewBox="0 0 108 112" role="img" aria-label="Warehouse map showing occupied and empty shelf locations">
      {map.edges.map(([from, to]) => {
        const start = map.nodes[from];
        const end = map.nodes[to];
        return <line key={`${from}-${to}`} x1={start[0]} y1={start[1]} x2={end[0]} y2={end[1]} className="map-road" />;
      })}
      {Object.entries(map.nodes).map(([node, [x, y]]) => {
        const isJunction = node.includes("_J");
        const shelf = shelfByNode[node];
        const color = shelf && ZONES.find((zone) => zone.category === shelf.category)?.color;
        if (isJunction) return <circle key={node} cx={x} cy={y} r="0.8" className="junction" />;
        const labelY = nodeLabelY(node, y);
        return (
          <g key={node}>
            {shelf ? (
              <g className={`inventory-shelf-card ${shelf.empty ? "empty" : "occupied"}`}>
                <rect x={x - 5} y={y - 5} width="10" height="10" rx="0.8" style={{ "--shelf-color": color }} />
                <text x={x} y={y - 0.5} textAnchor="middle" className="inventory-shelf-name">{shelf.shelf_id.split("_").at(-1)}</text>
                <text x={x} y={y + 2.2} textAnchor="middle" className="inventory-shelf-item">{shelf.empty ? "empty" : shelf.item_id}</text>
              </g>
            ) : (
              <>
                <circle cx={x} cy={y} r="1.35" className={nodeClass(node, false)} />
                <text x={x} y={labelY} textAnchor={nodeLabelAnchor(x)} className="node-label">{node}</text>
              </>
            )}
            <title>{shelf ? (shelf.empty ? `${shelf.shelf_id}: Empty` : `${shelf.item_name || shelf.item_id} (${shelf.item_id})`) : node}</title>
          </g>
        );
      })}
    </svg>
  );
}

function MetricCard({ value, label }) {
  return <article className="warehouse-card"><strong>{value}</strong><span>{label}</span></article>;
}

function WarehouseLogFeed({ warehouse, error }) {
  return (
    <section className="warehouse-log-feed" aria-label="Recent warehouse logs">
      <div className="monitor-heading"><span>Recent activity</span><small>Auto-refreshing</small></div>
      {error && <p className="empty-queue">{error}</p>}
      {!error && !warehouse && <p className="empty-queue">Loading logs…</p>}
      {warehouse?.logs?.length === 0 && <p className="empty-queue">No warehouse logs yet</p>}
      <div className="warehouse-log-list">
        {warehouse?.logs?.map((log) => (
          <article key={log.id} className="warehouse-log-entry">
            <div><strong>{log.item_id}</strong><span className={`log-status ${log.status.toLowerCase()}`}>{log.status}</span></div>
            <small>{log.pickup_location || "—"} → {log.dropoff_location || "—"}</small>
            <time dateTime={log.time}>{formatWarehouseTimestamp(log.time)}</time>
          </article>
        ))}
      </div>
    </section>
  );
}

function formatWarehouseTimestamp(value) {
  const date = new Date(value);
  if (Number.isNaN(date.getTime())) return value;
  return new Intl.DateTimeFormat(undefined, {
    year: "numeric",
    month: "short",
    day: "numeric",
    hour: "numeric",
    minute: "2-digit",
    second: "2-digit",
  }).format(date);
}

function RobotQueue({ tasks, currentTaskId, robotId, onRobotUpdate }) {
  const [cancelNotice, setCancelNotice] = useState("");
  const [previewLoading, setPreviewLoading] = useState(false);
  const [draggedTaskId, setDraggedTaskId] = useState(null);
  const [reorderLoading, setReorderLoading] = useState(false);
  if (!tasks.length) return <p className="empty-queue">No assigned tasks</p>;
  const previewReplan = async () => {
    setPreviewLoading(true);
    setCancelNotice("");
    try {
      const { data } = await API.post(`/robots/${robotId}/cancel`);
      if (data.robot_state) onRobotUpdate(data.robot_state);
      const plan = data.plan;
      const route = plan?.path?.filter((node) => !node.startsWith("TEMP_ROBOT_")).join(" → ");
      setCancelNotice(route
        ? `${data.message} ${plan.first_action} toward ${plan.first_reentry_node}; route ${route}.`
        : data.message || "Task canceled.");
    } catch (requestError) {
      const updatedRobot = requestError.response?.data?.robot_state;
      if (updatedRobot) onRobotUpdate(updatedRobot);
      const status = requestError.response?.status;
      setCancelNotice(requestError.response?.data?.error || (
        status === 404
          ? "Cancel-and-replan endpoint not found. Restart the backend from this project."
          : `Could not cancel and replan${status ? ` (HTTP ${status})` : ". Check the backend connection."}`
      ));
    } finally {
      setPreviewLoading(false);
    }
  };
  const canReorder = tasks.some((task) => task.id === currentTaskId && task.status === "TO_PICKUP");
  const reorder = async (targetTaskId) => {
    if (!canReorder || draggedTaskId == null || draggedTaskId === targetTaskId || reorderLoading) return;
    const activeTask = tasks.find((task) => task.id === currentTaskId);
    const queued = tasks.filter((task) => task.id !== activeTask?.id);
    const from = queued.findIndex((task) => task.id === draggedTaskId);
    const to = queued.findIndex((task) => task.id === targetTaskId);
    if (from < 0 || to < 0) return;
    const [moved] = queued.splice(from, 1);
    queued.splice(to, 0, moved);
    setReorderLoading(true);
    setCancelNotice("");
    try {
      const { data } = await API.post(`/robots/${robotId}/queue/reorder`, { task_ids: queued.map((task) => task.id), manually_moved_task_id: draggedTaskId });
      if (data.robot_state) onRobotUpdate(data.robot_state);
    } catch (requestError) {
      setCancelNotice(requestError.response?.data?.error || "Could not reorder tasks. Refresh the robot monitor and try again.");
    } finally {
      setReorderLoading(false);
      setDraggedTaskId(null);
    }
  };
  return (
    <ol className="task-queue">
      {tasks.map((task) => {
        const active = task.id === currentTaskId;
        const canCancel = active && task.status === "TO_PICKUP";
        const draggable = canReorder && !active && !reorderLoading;
        return <li className={`task-item ${robotId.toLowerCase()} ${active ? "active-task" : ""} ${task.rank_move && task.rank_move !== "same" ? `rank-${task.rank_move}` : ""} ${draggable ? "draggable-task" : ""}`}
          key={task.id} draggable={draggable}
          onDragStart={(event) => { setDraggedTaskId(task.id); event.dataTransfer.effectAllowed = "move"; }}
          onDragEnd={() => setDraggedTaskId(null)}
          onDragOver={(event) => { if (draggable && draggedTaskId != null) event.preventDefault(); }}
          onDrop={(event) => { event.preventDefault(); reorder(task.id); }}>
          <div className="task-item-heading">
            <strong className="task-identity">
              {!active && <span className={`task-drag-handle ${canReorder ? "available" : "locked"}`} aria-label={canReorder ? "Drag to reorder task" : "Reordering unavailable while robot is carrying a load"} title={canReorder ? "Drag to reorder" : "Reordering unavailable while robot is carrying a load"}><i /><i /><i /><i /><i /><i /></span>}
              <span>#{task.id} · {active ? "Active" : `Queue ${task.pending_rank ?? "–"}`}</span>
            </strong>
            <span>{task.status}</span>
            {active && <button
              className="cancel-task-button"
              type="button"
              disabled={!canCancel || previewLoading}
              title={canCancel ? "Cancel this task and replan for the next queued task" : "Cancel is available only during pickup"}
              onClick={previewReplan}
            >{previewLoading ? "Planning…" : "Cancel"}</button>}
          </div>
          <small>{task.PL} → {task.DL} · deadline {task.deadline}s</small>
        </li>;
      })}
      {canReorder && <li className="queue-reorder-hint" role="note">Drag queued tasks to change their order{reorderLoading ? "…" : ""}</li>}
      {cancelNotice && <li className="cancel-task-notice" role="status">{cancelNotice}</li>}
    </ol>
  );
}

function CompletedMissionHistory({ robotId, refreshKey }) {
  const [missions, setMissions] = useState([]);
  const [error, setError] = useState("");

  useEffect(() => {
    let active = true;
    API.get(`/robots/${robotId}/completed-missions`)
      .then(({ data }) => {
        if (!active) return;
        setMissions(data.missions || []);
        setError("");
      })
      .catch(() => {
        if (active) setError("Mission history is unavailable");
      });
    return () => { active = false; };
  }, [robotId, refreshKey]);

  return (
    <section className="completed-mission-history" aria-label={`${robotId} completed mission history`}>
      <strong>Completed missions</strong>
      {error ? <p className="empty-queue">{error}</p> : missions.length === 0 ? (
        <p className="empty-queue">No completed missions yet</p>
      ) : (
        <ul>
          {missions.map((mission) => (
            <li key={mission.mission_id}>
              <div>
                <strong>#{mission.task_id ?? "—"} · {mission.serial_number || "Mission"}</strong>
                <small>{mission.pickup_node || "—"} → {mission.destination_node || "—"}</small>
                <time dateTime={mission.completed_at}>Completed {formatWarehouseTimestamp(mission.completed_at)}</time>
              </div>
              <a
                className="mission-history-download"
                href={`${API.defaults.baseURL}/robots/${robotId}/mission-history.csv?mission_id=${encodeURIComponent(mission.mission_id)}`}
                download
              >Download CSV</a>
            </li>
          ))}
        </ul>
      )}
    </section>
  );
}

function WarehouseMap({ map, robots }) {
  const point = ([x, y]) => ({ x, y });
  const nodePositions = Object.fromEntries(
    Object.entries(map.nodes).map(([node, position]) => [node, { x: position[0], y: position[1] }]),
  );
  return (
    <svg className="warehouse-map" viewBox="0 0 108 112" role="img" aria-label="Live warehouse robot positions">
      {map.edges.map(([from, to]) => {
        const start = point(map.nodes[from]);
        const end = point(map.nodes[to]);
        return <line key={`${from}-${to}`} x1={start.x} y1={start.y} x2={end.x} y2={end.y} className="map-road" />;
      })}
      <image href={amrWarehouseDesign} x="25" y="48.45" width="22" height="11.1" preserveAspectRatio="xMidYMid meet" aria-label="Autonomous Mobile Robot Warehouse design" />
      <image href={finalYearThesisDesign} x="61" y="48.85" width="22" height="10.3" preserveAspectRatio="xMidYMid meet" aria-label="Final Year Thesis design" />
      {Object.values(robots).map((robot) => {
        const position = robot.map_position || map.nodes[robot.node];
        if (!position || !robot.display_route?.length) return null;
        const remainingRoute = robot.display_route.filter((node) => Boolean(nodePositions[node]));
        return <RemainingRouteDisplay
          key={`route-${robot.robot_id}`}
          agv={{ ...robot, x: position[0], y: position[1], display_route: remainingRoute }}
          nodePositions={nodePositions}
          color={robot.robot_id === "R1" ? "#f97316" : "#dc2626"}
          width={robot.robot_id === "R1" ? 1.2 : 0.9}
        />;
      })}
      {Object.entries(map.nodes).map(([node, position]) => {
        const { x, y } = point(position);
        const isJunction = node.includes("_J");
        return (
          <g key={node}>
            <circle cx={x} cy={y} r={isJunction ? 0.8 : 1.35} className={nodeClass(node, isJunction)} />
            {!isJunction && <text x={x} y={nodeLabelY(node, y)} textAnchor={nodeLabelAnchor(x)} className="node-label">{node}</text>}
            <title>{node}</title>
          </g>
        );
      })}
      {Object.values(robots).map((robot) => {
        const position = robot.map_position || map.nodes[robot.node];
        if (!position) return null;
        const { x, y } = point(position);
        const location = robot.map_edge
          ? `${robot.node} → ${robot.map_edge.to_node} (${robot.distance_since_last_node_cm?.toFixed(1) ?? "?"} cm${robot.encoder_distance_calibrated ? "" : " estimated"})`
          : robot.node;
        return <g key={robot.robot_id} className={`robot-marker ${robot.robot_id.toLowerCase()}`}><circle cx={x} cy={y} r="3.4" /><text x={x} y={y + 0.8}>{robot.robot_id}</text><title>{`${robot.robot_id}: ${location}`}</title></g>;
      })}
    </svg>
  );
}

function nodeLabelAnchor(x) {
  if (x <= 10) return "start";
  if (x >= 94) return "end";
  return "middle";
}

function nodeLabelY(node, y) {
  const labelsAbove = new Set([
    "Yellow_1", "Yellow_2", "Yellow_3",
    "Green_1", "Green_2", "Green_3",
    "Purple_4", "Purple_5",
    "Orange_4", "Orange_5",
    "Parking_1",
  ]);
  return y + (labelsAbove.has(node) ? -2.2 : 3.3);
}

function nodeClass(node, isJunction) {
  if (isJunction) return "junction";
  if (node.startsWith("Yellow")) return "map-node yellow-node";
  if (node.startsWith("Purple")) return "map-node purple-node";
  if (node.startsWith("Green")) return "map-node green-node";
  if (node.startsWith("Orange")) return "map-node orange-node";
  if (node === "Gate_1" || node === "Gate_2") return "map-node pickup-gate-node";
  if (node === "Gate_3" || node === "Gate_4") return "map-node dropoff-gate-node";
  if (node.startsWith("Parking")) return "map-node parking-node";
  return "map-node";
}

export default App;

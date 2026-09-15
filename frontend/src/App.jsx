import { useEffect, useState } from "react";
import TaskForm from "./components/TaskForm";
import API from "./api";
import "./App.css";

function App() {
  const [map, setMap] = useState(null);
  const [robots, setRobots] = useState({});
  const [error, setError] = useState("");
  const [warehouse, setWarehouse] = useState(null);
  const [warehouseError, setWarehouseError] = useState("");
  const [activeSidebarTab, setActiveSidebarTab] = useState("monitor");
  const [warehouseAlert, setWarehouseAlert] = useState(null);

  useEffect(() => {
    const load = async () => {
      try {
        const [{ data: mapData }, { data: robotData }] = await Promise.all([
          API.get("/map"),
          API.get("/robots"),
        ]);
        setMap(mapData);
        setRobots(robotData);
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
    events.addEventListener("robot-state", (event) => {
      const robot = JSON.parse(event.data);
      setRobots((current) => ({ ...current, [robot.robot_id]: robot }));
      setError("");
    });
    events.addEventListener("warehouse-alert", (event) => {
      const alert = JSON.parse(event.data);
      setWarehouseAlert({
        title: alert.type === "duplicate-item" ? "Item already stored" : "Shelf capacity reached",
        message: alert.message,
      });
    });
    events.onerror = () => setError("Live connection lost — retrying…");
    return () => events.close();
  }, []);

  return (
    <main className="dashboard">
      <section className="map-panel">
        <div className="panel-heading">
          <div>
            <p className="eyebrow">Live MQTT monitor</p>
            <h1>Warehouse robots</h1>
          </div>
          <div className="header-meta">
            <span className={error ? "connection offline" : "connection"}><i />{error || "Live MQTT connection"}</span>
            <a className="object-detection-link" href="http://localhost:8001/">
              Live View
            </a>
          </div>
        </div>
        {activeSidebarTab === "warehouse" ? (
          <WarehouseMonitor warehouse={warehouse} error={warehouseError} />
        ) : map && <WarehouseMap map={map} robots={robots} />}
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
            Create Task
          </button>
          <button
            type="button"
            role="tab"
            aria-selected={activeSidebarTab === "warehouse"}
            className={activeSidebarTab === "warehouse" ? "active" : ""}
            onClick={() => setActiveSidebarTab("warehouse")}
          >
            Warehouse
          </button>
        </div>
        {activeSidebarTab === "monitor" ? (
          <section className="side-monitor" aria-label="Robot state monitor">
            <div className="monitor-heading"><span>Robot monitor</span><small>Live state & queue</small></div>
            <div className="robot-list">
              {Object.values(robots).map((robot) => (
                <article className={`robot-card ${robot.robot_id.toLowerCase()}`} key={robot.robot_id}>
                  <header className="robot-card-header">
                    <span className={`robot-dot ${robot.robot_id.toLowerCase()}`} />
                    <div><strong>{robot.robot_id}</strong><small>{robot.status} · {robot.node}</small></div>
                    <span>{Math.ceil(robot.estimated_seconds_until_free)}s free</span>
                  </header>
                  <dl className="robot-stats">
                    <div><dt>Current task</dt><dd>{robot.current_task_id ?? "None"}</dd></div>
                    <div><dt>Payload</dt><dd>{robot.has_payload ? "Loaded" : "Empty"}</dd></div>
                    <div><dt>Queue</dt><dd>{robot.queue_length} task{robot.queue_length === 1 ? "" : "s"}</dd></div>
                  </dl>
                  <RobotQueue tasks={robot.queue} currentTaskId={robot.current_task_id} robotId={robot.robot_id} />
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

function WarehouseMonitor({ warehouse, error }) {
  if (error) return <p className="warehouse-loading">{error}</p>;
  if (!warehouse) return <p className="warehouse-loading">Loading warehouse data…</p>;
  const { stats, shelves } = warehouse;
  return (
    <section className="warehouse-monitor" aria-label="Live warehouse monitor">
      <div className="warehouse-heading">
        <span className="refresh-status"><i />Refreshes every 2 seconds</span>
      </div>
      <div className="warehouse-cards">
        <MetricCard value={stats.total_items} label="Catalog items" />
        <MetricCard value={stats.in_stock} label="Currently in stock" />
        <MetricCard value={`${stats.occupied_shelves}/${stats.total_shelves}`} label="Shelves occupied" />
        <MetricCard value={stats.total_logs} label="Log entries" />
      </div>
      <h2 className="shelf-map-title">Shelf map</h2>
      <div className="warehouse-zones">
        {ZONES.map((zone) => (
          <section className="warehouse-zone" key={zone.category}>
            <h3><span style={{ background: zone.color }} />{zone.title} — {zone.category}</h3>
            <div className="shelf-grid">
              {shelves.filter((shelf) => shelf.category === zone.category).map((shelf) => (
                <article className={`warehouse-shelf ${shelf.empty ? "empty" : ""}`} style={{ background: zone.color }} key={shelf.shelf_id} title={shelf.item_name || "Empty shelf"}>
                  <strong>{shelf.shelf_id}</strong>
                  <span>{shelf.empty ? "empty" : shelf.item_id}</span>
                </article>
              ))}
            </div>
          </section>
        ))}
      </div>
    </section>
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
            <time>{log.time}</time>
          </article>
        ))}
      </div>
    </section>
  );
}

function RobotQueue({ tasks, currentTaskId, robotId }) {
  if (!tasks.length) return <p className="empty-queue">No assigned tasks</p>;
  return (
    <ol className="task-queue">
      {tasks.map((task) => {
        const active = task.id === currentTaskId;
        return <li className={`task-item ${robotId.toLowerCase()} ${active ? "active-task" : ""}`} key={task.id}>
          <div><strong>#{task.id} · {active ? "Active" : `Queue ${task.pending_rank ?? "–"}`}</strong><span>{task.status}</span></div>
          <small>{task.PL} → {task.DL} · deadline {task.deadline}s</small>
        </li>;
      })}
    </ol>
  );
}

function WarehouseMap({ map, robots }) {
  const point = ([x, y]) => ({ x, y });
  return (
    <svg className="warehouse-map" viewBox="0 0 108 112" role="img" aria-label="Live warehouse robot positions">
      {map.edges.map(([from, to]) => {
        const start = point(map.nodes[from]);
        const end = point(map.nodes[to]);
        return <line key={`${from}-${to}`} x1={start.x} y1={start.y} x2={end.x} y2={end.y} className="map-road" />;
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
        const position = map.nodes[robot.node];
        if (!position) return null;
        const { x, y } = point(position);
        return <g key={robot.robot_id} className={`robot-marker ${robot.robot_id.toLowerCase()}`}><circle cx={x} cy={y} r="3.4" /><text x={x} y={y + 0.8}>{robot.robot_id}</text><title>{`${robot.robot_id}: ${robot.node}`}</title></g>;
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

# Virtual Node replanner extraction

This package is a standalone copy of the current Virtual Node method. It does not import Django, the project service layer, MQTT, hardware, or the Static Graph method. `graph.py`, `astar.py`, `directions.py`, and `warehouse_map.json` are copied from the existing project; `replanner.py` contains the existing `AGVState`, backward-penalty calculation, and `VirtualNodeReplanner` behavior.

## Interface

```python
from pathlib import Path
from virtual_node_replanner import AGVState, Graph, VirtualNodeReplanner

graph = Graph(str(Path("warehouse_map.json")))
state = AGVState(
    robot_id="AGV_001",
    previous_node="Node_A",
    next_node="Node_B",
    destination="Node_Z",
    heading=0.0,
    distance_from_previous_cm=5.0,
    distance_to_next_cm=5.0,
    current_position=(10.0, 20.0),
    simulation_time=0.0,
)
result = VirtualNodeReplanner(graph).replan(state, backward_penalty_cm=10.0)
```

The caller must provide an already-resolved current edge and current position. This extraction intentionally does not add RFID/encoder localization, edge discovery, progress calculation, or new validation: those functions are not implemented in the original Virtual Node method. The original method also does not use the two distance fields to determine temporary-edge weights; `Graph.connect_temp_node()` computes weights from graph coordinates.

## Units

The source JSON has numeric coordinates but does not declare feet or another physical unit. A* distances and temporary-edge weights use those raw map coordinate units. The live simulator labels coordinate units as centimetres, and the replanner's output keys use `_cm`; no feet-to-centimetres conversion exists here. Preserve the source map scale when integrating, and resolve this unit contract against the actual map before interpreting output as physical distance.

## Behavior retained

- Each replan deep-copies the graph object, inserts `TEMP_ROBOT_<robot_id>`, and connects it to the supplied previous and next nodes.
- The two temporary edge weights are Manhattan distances from the temporary coordinate to each endpoint.
- Heading can add the configured penalty to the temporary-to-previous edge.
- A* starts at the temporary node and targets `state.destination`.
- The returned `path` includes the temporary node. Existing live simulation code strips that node before executing the route.
- Optional blocked edge/node inputs are retained because they are part of the existing replanner signature. No obstacle detection or Type-1 trigger logic is included.

The extraction tests retain comparison hooks for the original service implementation when that source is available.

## Backend integration

The backend now uses the extracted virtual-node replanner when an
`agv/<robot_id>/node` JSON report includes `previous_node`, `next_node`,
`current_position` (`[x, y]`), and `heading`. Reports containing only an RFID
or node ID continue to use node-by-node replanning. The destination always
comes from the robot's active warehouse task. Coordinates and heading use the
warehouse map convention; the map's coordinate scale is still not calibrated
to centimetres. The optional distance fields are retained as metadata and do
not affect virtual-edge costs.

The backend A* implementation also reports search diagnostics
(`nodes_explored`, iterations, exploration order, neighbor evaluations, and
maximum open-set size). The RFID replanning service returns these in its
`search_metrics` field.

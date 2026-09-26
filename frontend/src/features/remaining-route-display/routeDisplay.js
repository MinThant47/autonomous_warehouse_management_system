/** Return route node IDs that remain ahead of the robot. */
export function getRemainingRoute(agv) {
  if (!agv) return [];
  if (Array.isArray(agv.display_route)) return agv.display_route;

  const route = Array.isArray(agv.route) ? agv.route : [];
  return route.slice(agv.route_index ?? 0);
}

/** Convert the live position and remaining route nodes to SVG polyline points. */
export function makePolylinePoints(position, routeNodes, nodePositions, mapScale = 1) {
  const points = [];
  if (position && position.x != null && position.y != null) points.push(position);

  for (const nodeId of routeNodes || []) {
    const node = nodePositions?.[nodeId];
    if (node) points.push(node);
  }

  return points.map((point) => `${point.x * mapScale},${point.y * mapScale}`).join(" ");
}

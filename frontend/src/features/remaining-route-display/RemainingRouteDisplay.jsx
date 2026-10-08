import { getRemainingRoute, makePolylinePoints } from "./routeDisplay";

/** Draw the live robot position and its remaining route as an SVG polyline. */
export default function RemainingRouteDisplay({
  agv,
  nodePositions,
  mapScale = 1,
  color = "#00897b",
  width = 1.2,
}) {
  if (!agv || agv.x == null || agv.y == null) return null;

  const routeNodes = getRemainingRoute(agv);
  const points = makePolylinePoints(
    { x: agv.x, y: agv.y },
    routeNodes,
    nodePositions,
    mapScale,
  );

  if (points.split(" ").length < 2) return null;
  return (
    <polyline
      points={points}
      fill="none"
      stroke={color}
      strokeWidth={width}
      strokeLinecap="round"
      strokeLinejoin="round"
      opacity="0.9"
      pointerEvents="none"
    />
  );
}

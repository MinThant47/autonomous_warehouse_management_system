"""In-memory reservations for the single-lane warehouse graph."""

from threading import RLock


class TrafficManager:
    """Serialize AGV access to undirected track edges and RFID nodes.

    A robot keeps its current node, traversed edge, and destination node
    reserved until its next RFID report confirms arrival. This is deliberately
    conservative: a missing report leaves the resource occupied.
    """

    def __init__(self, initial_nodes, enabled=True):
        self._lock = RLock()
        self._enabled = bool(enabled)
        self._resource_owner = {}
        self._robot_resources = {}
        self._active_move = {}
        self._waiting_commands = {}
        self._current_node = dict(initial_nodes)
        for robot_id, node in initial_nodes.items():
            resource = ("node", node)
            self._resource_owner[resource] = robot_id
            self._robot_resources[robot_id] = {resource}

    @property
    def enabled(self):
        with self._lock:
            return self._enabled

    def set_enabled(self, enabled):
        """Enable reservations, rejecting activation over conflicting live moves."""
        if not isinstance(enabled, bool):
            raise ValueError("Traffic control enabled state must be a boolean")

        with self._lock:
            if enabled == self._enabled:
                return
            if not enabled:
                self._enabled = False
                self._resource_owner.clear()
                self._robot_resources = {robot_id: set() for robot_id in self._current_node}
                return

            owners = {}
            robot_resources = {}
            for robot_id, current_node in self._current_node.items():
                move = self._active_move.get(robot_id)
                requested = (
                    (
                        ("node", move["from_node"]),
                        self._edge_resource(move["from_node"], move["to_node"]),
                        ("node", move["to_node"]),
                    )
                    if move
                    else (("node", current_node),)
                )
                for resource in requested:
                    owner = owners.get(resource)
                    if owner not in (None, robot_id):
                        raise ValueError(
                            "Traffic control cannot be enabled while the robots' current moves "
                            "claim the same node or edge. Let them clear the conflict first."
                        )
                    owners[resource] = robot_id
                    robot_resources.setdefault(robot_id, set()).add(resource)

            self._resource_owner = owners
            self._robot_resources = robot_resources
            self._enabled = True

    @staticmethod
    def _edge_resource(node_a, node_b):
        return ("edge", *sorted((node_a, node_b)))

    def observe_node(self, robot_id, node):
        """Release the completed move when RFID confirms its destination."""
        with self._lock:
            if not self._enabled:
                self._active_move.pop(robot_id, None)
                self._current_node[robot_id] = node
                self._robot_resources.setdefault(robot_id, set()).clear()
                return True

            active_move = self._active_move.get(robot_id)
            resources = self._robot_resources.setdefault(robot_id, set())
            if active_move and node == active_move["to_node"]:
                for resource in tuple(resources):
                    if resource == ("node", node):
                        continue
                    if self._resource_owner.get(resource) == robot_id:
                        self._resource_owner.pop(resource, None)
                    resources.discard(resource)
                self._active_move.pop(robot_id, None)
                self._current_node[robot_id] = node
            elif active_move and node != active_move["from_node"]:
                # An unexpected RFID report confirms the AGV is no longer on
                # its last edge. Release that move, then reserve the observed
                # node only if no other robot already owns it.
                for resource in tuple(resources):
                    if resource == ("node", node):
                        continue
                    if self._resource_owner.get(resource) == robot_id:
                        self._resource_owner.pop(resource, None)
                    resources.discard(resource)
                self._active_move.pop(robot_id, None)
                self._current_node[robot_id] = node
            elif not active_move and node != self._current_node.get(robot_id):
                for resource in tuple(resources):
                    if resource == ("node", node):
                        continue
                    if self._resource_owner.get(resource) == robot_id:
                        self._resource_owner.pop(resource, None)
                    resources.discard(resource)
                self._current_node[robot_id] = node

            node_resource = ("node", node)
            owner = self._resource_owner.get(node_resource)
            if owner in (None, robot_id):
                self._resource_owner[node_resource] = robot_id
                resources.add(node_resource)
                return True
            return False

    def reserve_command(self, robot_id, command):
        """Reserve the next move; return false and queue it if blocked."""
        source = command.get("current_node")
        target = command.get("next_node")
        if not isinstance(source, str) or not isinstance(target, str):
            with self._lock:
                self._waiting_commands.pop(robot_id, None)
            return True

        edge_resource = self._edge_resource(source, target)
        source_resource = ("node", source)
        target_resource = ("node", target)
        requested = (source_resource, edge_resource, target_resource)
        with self._lock:
            if not self._enabled:
                self._waiting_commands.pop(robot_id, None)
                self._active_move[robot_id] = {"from_node": source, "to_node": target}
                self._robot_resources.setdefault(robot_id, set()).clear()
                return True

            if self._active_move.get(robot_id) == {"from_node": source, "to_node": target}:
                self._waiting_commands.pop(robot_id, None)
                return True

            blocked = any(
                self._resource_owner.get(resource) not in (None, robot_id)
                for resource in requested
            )
            if blocked:
                self._waiting_commands[robot_id] = dict(command)
                return False

            resources = self._robot_resources.setdefault(robot_id, set())
            for resource in requested:
                self._resource_owner[resource] = robot_id
                resources.add(resource)
            self._active_move[robot_id] = {"from_node": source, "to_node": target}
            self._waiting_commands.pop(robot_id, None)
            return True

    def waiting_robot_ids(self):
        with self._lock:
            return list(self._waiting_commands)

    def waiting_commands(self):
        with self._lock:
            return [(robot_id, dict(command)) for robot_id, command in self._waiting_commands.items()]

    def is_waiting(self, robot_id):
        with self._lock:
            return robot_id in self._waiting_commands

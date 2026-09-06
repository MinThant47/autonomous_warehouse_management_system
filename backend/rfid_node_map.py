"""Translate RFID reader values into warehouse map node IDs."""
import json
from pathlib import Path

from new_warehouse_map import nodes


RFID_NODE_MAP_PATH = Path(__file__).with_name("rfid_node_map.json")


def _normalise_rfid_id(rfid_id):
    """Make common UID formats (AA BB, AA:BB, AA-BB) compare equally."""
    return "".join(character for character in rfid_id.upper() if character.isalnum())


def _load_rfid_node_map():
    try:
        mapping = json.loads(RFID_NODE_MAP_PATH.read_text(encoding="utf-8"))
    except FileNotFoundError as error:
        raise ValueError(f"RFID mapping file not found: {RFID_NODE_MAP_PATH}") from error
    except json.JSONDecodeError as error:
        raise ValueError(f"Invalid JSON in RFID mapping file: {error.msg}") from error

    if not isinstance(mapping, dict):
        raise ValueError("RFID mapping file must contain a JSON object")

    normalised_mapping = {}
    for rfid_id, node_id in mapping.items():
        if not isinstance(rfid_id, str) or not isinstance(node_id, str):
            raise ValueError("Every RFID mapping must use string RFID IDs and node IDs")
        if node_id not in nodes:
            raise ValueError(f"RFID '{rfid_id}' maps to unknown map node '{node_id}'")
        normalised_mapping[_normalise_rfid_id(rfid_id)] = node_id
    return normalised_mapping


def resolve_node_id(identifier):
    """Return a map node for either its node name or an RFID reader value."""
    if not isinstance(identifier, str) or not identifier.strip():
        raise ValueError("RFID ID must be a non-empty string")

    identifier = identifier.strip()
    if identifier in nodes:
        return identifier  # Continue accepting existing node-name reports.

    node_id = _load_rfid_node_map().get(_normalise_rfid_id(identifier))
    if node_id:
        return node_id
    raise ValueError(
        f"Unknown RFID ID '{identifier}'. Add it to {RFID_NODE_MAP_PATH.name}."
    )

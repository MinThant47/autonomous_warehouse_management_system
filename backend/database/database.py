"""Independent SQLite operations for warehouse items and storage shelves."""

import sqlite3
from contextlib import contextmanager
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterator

DATABASE_PATH = Path(__file__).with_name("warehouse.db")
SCHEMA_PATH = Path(__file__).with_name("schema.sql")
VALID_LOG_STATUSES = {"IN", "OUT", "RELOCATION"}


class WarehouseDatabaseError(ValueError):
    """Raised when a requested warehouse database operation is invalid."""


def _connect() -> sqlite3.Connection:
    DATABASE_PATH.parent.mkdir(parents=True, exist_ok=True)
    # Allow a concurrent Flask request or another short-lived database writer
    # to finish instead of immediately failing with "database is locked".
    connection = sqlite3.connect(DATABASE_PATH, timeout=30)
    connection.row_factory = sqlite3.Row
    connection.execute("PRAGMA foreign_keys = ON")
    connection.execute("PRAGMA busy_timeout = 30000")
    return connection


@contextmanager
def _transaction() -> Iterator[sqlite3.Connection]:
    connection = _connect()
    try:
        with connection:
            yield connection
    finally:
        connection.close()


def init_database() -> None:
    """Create the warehouse database and all required tables when absent."""
    with _connect() as connection:
        connection.execute("PRAGMA journal_mode = WAL")
        connection.executescript(SCHEMA_PATH.read_text(encoding="utf-8"))


def add_item(item_id: str, item_name: str | None, category: str) -> None:
    """Register an item serial code before placing it into inventory."""
    if not item_id or not category:
        raise WarehouseDatabaseError("item_id and category are required")
    with _transaction() as connection:
        try:
            connection.execute(
                "INSERT INTO ITEM (item_id, item_name, category) VALUES (?, ?, ?)",
                (item_id, item_name, category),
            )
        except sqlite3.IntegrityError as error:
            raise WarehouseDatabaseError(f"Item '{item_id}' already exists") from error


def get_available_shelf(category: str) -> str | None:
    """Return the first unoccupied shelf in *category*, or ``None`` if full."""
    with _connect() as connection:
        row = connection.execute(
            """
            SELECT shelf.shelf_id
            FROM SHELF AS shelf
            WHERE shelf.category = ?
              AND NOT EXISTS (
                  SELECT 1 FROM INVENTORY AS inventory
                  WHERE inventory.shelf_id = shelf.shelf_id
              )
            ORDER BY shelf.shelf_id
            LIMIT 1
            """,
            (category,),
        ).fetchone()
    return row["shelf_id"] if row else None


def get_item_location(item_id: str) -> dict[str, str] | None:
    """Return an item's current shelf, or ``None`` when it is not stored."""
    with _connect() as connection:
        row = connection.execute(
            "SELECT item_id, shelf_id FROM INVENTORY WHERE item_id = ?", (item_id,)
        ).fetchone()
    return dict(row) if row else None


def get_item_at_shelf(shelf_id: str) -> dict[str, str] | None:
    """Return the item currently stored at *shelf_id*, if any."""
    with _connect() as connection:
        row = connection.execute(
            "SELECT item_id, shelf_id FROM INVENTORY WHERE shelf_id = ?", (shelf_id,)
        ).fetchone()
    return dict(row) if row else None


def get_inventory_items() -> list[dict[str, str]]:
    """Return currently stored items for outbound-task selection."""
    with _connect() as connection:
        rows = connection.execute(
            """
            SELECT item.item_id, item.item_name, item.category, inventory.shelf_id
            FROM INVENTORY AS inventory
            JOIN ITEM AS item ON item.item_id = inventory.item_id
            ORDER BY item.item_name, item.item_id
            """
        ).fetchall()
    return [dict(row) for row in rows]


def get_warehouse_monitor() -> dict[str, object]:
    """Return a dashboard-ready snapshot of warehouse stock and recent logs."""
    with _connect() as connection:
        total_items = connection.execute("SELECT COUNT(*) FROM ITEM").fetchone()[0]
        in_stock = connection.execute("SELECT COUNT(*) FROM INVENTORY").fetchone()[0]
        total_shelves = connection.execute("SELECT COUNT(*) FROM SHELF").fetchone()[0]
        total_logs = connection.execute("SELECT COUNT(*) FROM LOG").fetchone()[0]
        shelves = connection.execute(
            """
            SELECT shelf.shelf_id, shelf.category,
                   CASE WHEN inventory.id IS NULL THEN 1 ELSE 0 END AS empty,
                   item.item_id, item.item_name, item.category AS item_category
            FROM SHELF AS shelf
            LEFT JOIN INVENTORY AS inventory ON inventory.shelf_id = shelf.shelf_id
            LEFT JOIN ITEM AS item ON item.item_id = inventory.item_id
            ORDER BY shelf.shelf_id
            """
        ).fetchall()
        logs = connection.execute(
            """
            SELECT id, time, item_id, status, pickup_location, dropoff_location
            FROM LOG
            ORDER BY id DESC
            LIMIT 12
            """
        ).fetchall()

    shelf_data = [dict(row) for row in shelves]
    return {
        "stats": {
            "total_items": total_items,
            "in_stock": in_stock,
            "total_shelves": total_shelves,
            "occupied_shelves": total_shelves - sum(shelf["empty"] for shelf in shelf_data),
            "total_logs": total_logs,
        },
        "shelves": shelf_data,
        "logs": [dict(row) for row in logs],
    }


def get_shelves() -> list[dict[str, str | int]]:
    """Return all shelves with availability calculated from current inventory."""
    with _connect() as connection:
        rows = connection.execute(
            """
            SELECT shelf.shelf_id, shelf.category,
                   CASE WHEN EXISTS (
                       SELECT 1 FROM INVENTORY AS inventory
                       WHERE inventory.shelf_id = shelf.shelf_id
                   ) THEN 0 ELSE 1 END AS empty
            FROM SHELF AS shelf
            ORDER BY shelf.category, shelf.shelf_id
            """
        ).fetchall()
    return [dict(row) for row in rows]


def add_inventory_item(item_id: str, shelf_id: str) -> None:
    """Store an existing item on a compatible, currently free shelf."""
    with _transaction() as connection:
        item = connection.execute(
            "SELECT category FROM ITEM WHERE item_id = ?", (item_id,)
        ).fetchone()
        shelf = connection.execute(
            "SELECT category FROM SHELF WHERE shelf_id = ?", (shelf_id,)
        ).fetchone()
        if not item:
            raise WarehouseDatabaseError(f"Item '{item_id}' does not exist")
        if not shelf:
            raise WarehouseDatabaseError(f"Shelf '{shelf_id}' does not exist")
        if item["category"] != shelf["category"]:
            raise WarehouseDatabaseError("Item and shelf categories must match")
        if connection.execute("SELECT 1 FROM INVENTORY WHERE item_id = ?", (item_id,)).fetchone():
            raise WarehouseDatabaseError(f"Item '{item_id}' is already in inventory")
        if connection.execute("SELECT 1 FROM INVENTORY WHERE shelf_id = ?", (shelf_id,)).fetchone():
            raise WarehouseDatabaseError(f"Shelf '{shelf_id}' is occupied")
        connection.execute("INSERT INTO INVENTORY (item_id, shelf_id) VALUES (?, ?)", (item_id, shelf_id))
        connection.execute("UPDATE SHELF SET empty = 0 WHERE shelf_id = ?", (shelf_id,))


def receive_inbound_item(item_id: str, pickup_location: str) -> dict[str, str]:
    """Store an arriving catalog item in the first free category-matched shelf.

    The item lookup, empty-shelf selection, inventory insert, shelf update, and
    inbound log insert run in one SQLite transaction.
    """
    with _transaction() as connection:
        item = connection.execute(
            "SELECT category FROM ITEM WHERE item_id = ?", (item_id,)
        ).fetchone()
        if not item:
            raise WarehouseDatabaseError(f"Item '{item_id}' does not exist")
        if connection.execute(
            "SELECT 1 FROM INVENTORY WHERE item_id = ?", (item_id,)
        ).fetchone():
            raise WarehouseDatabaseError(f"Item '{item_id}' is already in inventory")

        shelf = connection.execute(
            """
            SELECT shelf.shelf_id
            FROM SHELF AS shelf
            WHERE shelf.category = ?
              AND NOT EXISTS (
                  SELECT 1 FROM INVENTORY AS inventory
                  WHERE inventory.shelf_id = shelf.shelf_id
              )
            ORDER BY shelf.shelf_id
            LIMIT 1
            """,
            (item["category"],),
        ).fetchone()
        if not shelf:
            raise WarehouseDatabaseError(f"No empty shelf is available for {item['category']}")

        shelf_id = shelf["shelf_id"]
        zone, shelf_number = shelf_id.split("_", maxsplit=1)
        dropoff_location = f"{zone.capitalize()}_{shelf_number}"
        connection.execute(
            "INSERT INTO INVENTORY (item_id, shelf_id) VALUES (?, ?)",
            (item_id, shelf_id),
        )
        connection.execute("UPDATE SHELF SET empty = 0 WHERE shelf_id = ?", (shelf_id,))
        connection.execute(
            """
            INSERT INTO LOG (time, item_id, status, pickup_location, dropoff_location)
            VALUES (?, ?, 'IN', ?, ?)
            """,
            (datetime.now(timezone.utc).isoformat(), item_id, pickup_location, dropoff_location),
        )
        return {
            "category": item["category"],
            "shelf_id": shelf_id,
            "dropoff_location": dropoff_location,
        }


def remove_inventory_item(item_id: str) -> None:
    """Remove an item from current inventory and free its former shelf."""
    with _transaction() as connection:
        inventory = connection.execute(
            "SELECT shelf_id FROM INVENTORY WHERE item_id = ?", (item_id,)
        ).fetchone()
        if not inventory:
            raise WarehouseDatabaseError(f"Item '{item_id}' is not in inventory")
        connection.execute("DELETE FROM INVENTORY WHERE item_id = ?", (item_id,))
        connection.execute("UPDATE SHELF SET empty = 1 WHERE shelf_id = ?", (inventory["shelf_id"],))


def move_inventory_item(item_id: str, pickup_shelf: str, destination_shelf: str) -> None:
    """Atomically move an item between compatible warehouse shelves."""
    with _transaction() as connection:
        inventory = connection.execute(
            "SELECT shelf_id FROM INVENTORY WHERE item_id = ?", (item_id,)
        ).fetchone()
        if not inventory or inventory["shelf_id"] != pickup_shelf:
            raise WarehouseDatabaseError(f"Item '{item_id}' is not at shelf '{pickup_shelf}'")
        source = connection.execute("SELECT category FROM SHELF WHERE shelf_id = ?", (pickup_shelf,)).fetchone()
        destination = connection.execute(
            "SELECT category FROM SHELF WHERE shelf_id = ?", (destination_shelf,)
        ).fetchone()
        if not source or not destination:
            raise WarehouseDatabaseError("Source and destination shelves must exist")
        if connection.execute("SELECT 1 FROM INVENTORY WHERE shelf_id = ?", (destination_shelf,)).fetchone():
            raise WarehouseDatabaseError(f"Shelf '{destination_shelf}' is occupied")
        connection.execute("UPDATE INVENTORY SET shelf_id = ? WHERE item_id = ?", (destination_shelf, item_id))
        connection.execute("UPDATE SHELF SET empty = 1 WHERE shelf_id = ?", (pickup_shelf,))
        connection.execute("UPDATE SHELF SET empty = 0 WHERE shelf_id = ?", (destination_shelf,))


def create_log(item_id: str, status: str, pickup_location: str | None, dropoff_location: str | None) -> int:
    """Record a historical IN, OUT, or RELOCATION event and return its log ID."""
    status = status.upper()
    if status not in VALID_LOG_STATUSES:
        raise WarehouseDatabaseError("status must be IN, OUT, or RELOCATION")
    with _transaction() as connection:
        if not connection.execute("SELECT 1 FROM ITEM WHERE item_id = ?", (item_id,)).fetchone():
            raise WarehouseDatabaseError(f"Item '{item_id}' does not exist")
        cursor = connection.execute(
            """
            INSERT INTO LOG (time, item_id, status, pickup_location, dropoff_location)
            VALUES (?, ?, ?, ?, ?)
            """,
            (datetime.now(timezone.utc).isoformat(), item_id, status, pickup_location, dropoff_location),
        )
        return cursor.lastrowid

"""Seed the storage shelves defined in ``new_warehouse_map.py``."""

if __package__:
    from .database import _transaction, init_database
else:  # Supports: python database/seed.py from backend/
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from database.database import _transaction, init_database


SHELVES = {
    "Electronics": [f"yellow_{number}" for number in range(1, 6)],
    "Final Products": [f"purple_{number}" for number in range(1, 6)],
    "Mechanical Parts": [f"green_{number}" for number in range(1, 8)],
    "Raw Materials": [f"orange_{number}" for number in range(1, 8)],
}

ITEMS = [
    ("E0001", "Arduino UNO", "Electronics"),
    ("E0002", "Arduino NANO", "Electronics"),
    ("E0003", "Arduino MEGA", "Electronics"),
    ("E0004", "ESP8266", "Electronics"),
    ("E0005", "ESP32", "Electronics"),
    ("E0006", "ESP32-CAM", "Electronics"),
    ("E0007", "STM32", "Electronics"),
    ("E0008", "Raspberry PI", "Electronics"),
    ("E0009", "Resistors", "Electronics"),
    ("E0010", "Capacitors", "Electronics"),
    ("M0001", "Motors", "Mechanical Parts"),
    ("M0002", "Gears", "Mechanical Parts"),
    ("M0003", "Bearings", "Mechanical Parts"),
    ("M0004", "Shafts", "Mechanical Parts"),
    ("M0005", "Screws", "Mechanical Parts"),
    ("M0006", "Nuts", "Mechanical Parts"),
    ("M0007", "Brackets", "Mechanical Parts"),
    ("M0008", "Pulleys", "Mechanical Parts"),
    ("M0009", "Springs", "Mechanical Parts"),
    ("M0010", "Washers", "Mechanical Parts"),
    ("R0001", "Steel Sheets", "Raw Materials"),
    ("R0002", "Aluminum Sheets", "Raw Materials"),
    ("R0003", "Plastic Sheets", "Raw Materials"),
    ("R0004", "Copper Wire", "Raw Materials"),
    ("R0005", "Rubber Sheets", "Raw Materials"),
    ("R0006", "PVC Pipes", "Raw Materials"),
    ("R0007", "Wood Panels", "Raw Materials"),
    ("R0008", "Acrylic Sheets", "Raw Materials"),
    ("R0009", "Stainless Steel Rods", "Raw Materials"),
    ("R0010", "Glass sheets", "Raw Materials"),
    ("F0001", "Electric Fans", "Final Products"),
    ("F0002", "LED Lamps", "Final Products"),
    ("F0003", "Power Adapters", "Final Products"),
    ("F0004", "Power Supply Units", "Final Products"),
    ("F0005", "Smart Watches", "Final Products"),
    ("F0006", "Digital Clocks", "Final Products"),
    ("F0007", "IoT Devices", "Final Products"),
    ("F0008", "Digital Thermometers", "Final Products"),
    ("F0009", "Bluetooth Speakers", "Final Products"),
    ("F0010", "Wireless Earphones", "Final Products"),
]

# A small initial stock sample. Each item is stored only on a shelf with the
# same business category; rerunning the seed never moves existing inventory.
INITIAL_INVENTORY = [
    ("E0001", "yellow_1", "Gate_1"),
    ("E0002", "yellow_2", "Gate_1"),
    ("F0001", "purple_1", "Gate_2"),
    ("M0001", "green_1", "Gate_1"),
    ("R0001", "orange_1", "Gate_2"),
]


def seed_shelves() -> None:
    """Seed catalog items and map shelves with their matching item categories."""
    init_database()
    with _transaction() as connection:
        connection.executemany(
            """
            INSERT INTO SHELF (shelf_id, category, empty) VALUES (?, ?, 1)
            ON CONFLICT(shelf_id) DO UPDATE SET category = excluded.category
            """,
            [(shelf_id, category) for category, shelf_ids in SHELVES.items() for shelf_id in shelf_ids],
        )
        connection.executemany(
            "INSERT OR IGNORE INTO ITEM (item_id, item_name, category) VALUES (?, ?, ?)",
            ITEMS,
        )
        for item_id, shelf_id, gate in INITIAL_INVENTORY:
            # Insert only if both the item and intended shelf are still free.
            # This makes the seed safe to run at every backend startup.
            connection.execute(
                """
                INSERT INTO INVENTORY (item_id, shelf_id)
                SELECT ?, ?
                WHERE NOT EXISTS (SELECT 1 FROM INVENTORY WHERE item_id = ?)
                  AND NOT EXISTS (SELECT 1 FROM INVENTORY WHERE shelf_id = ?)
                """,
                (item_id, shelf_id, item_id, shelf_id),
            )
            connection.execute(
                """
                INSERT INTO LOG (time, item_id, status, pickup_location, dropoff_location)
                SELECT CURRENT_TIMESTAMP, ?, 'IN', ?, ?
                WHERE EXISTS (
                    SELECT 1 FROM INVENTORY
                    WHERE item_id = ? AND shelf_id = ?
                )
                  AND NOT EXISTS (
                    SELECT 1 FROM LOG
                    WHERE item_id = ? AND status = 'IN' AND dropoff_location = ?
                )
                """,
                (item_id, gate, shelf_id, item_id, shelf_id, item_id, shelf_id),
            )
        # Inventory is the source of truth. Synchronize the cached empty flag
        # in case the database existed before this seed ran.
        connection.execute(
            """
            UPDATE SHELF
            SET empty = CASE WHEN EXISTS (
                SELECT 1 FROM INVENTORY WHERE INVENTORY.shelf_id = SHELF.shelf_id
            ) THEN 0 ELSE 1 END
            """
        )


if __name__ == "__main__":
    seed_shelves()
    print("Warehouse shelves and item catalog seeded.")

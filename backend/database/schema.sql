PRAGMA foreign_keys = ON;

CREATE TABLE IF NOT EXISTS ITEM (
    item_id TEXT PRIMARY KEY,
    item_name TEXT,
    category TEXT NOT NULL
);

CREATE TABLE IF NOT EXISTS SHELF (
    shelf_id TEXT PRIMARY KEY,
    category TEXT NOT NULL,
    empty INTEGER NOT NULL DEFAULT 1 CHECK (empty IN (0, 1))
);

CREATE TABLE IF NOT EXISTS INVENTORY (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    item_id TEXT NOT NULL UNIQUE,
    shelf_id TEXT NOT NULL UNIQUE,
    FOREIGN KEY (item_id) REFERENCES ITEM(item_id),
    FOREIGN KEY (shelf_id) REFERENCES SHELF(shelf_id)
);

CREATE TABLE IF NOT EXISTS LOG (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    time DATETIME NOT NULL,
    item_id TEXT NOT NULL,
    status TEXT NOT NULL CHECK (status IN ('IN', 'OUT', 'RELOCATION')),
    pickup_location TEXT,
    dropoff_location TEXT,
    FOREIGN KEY (item_id) REFERENCES ITEM(item_id)
);

CREATE INDEX IF NOT EXISTS idx_shelf_category ON SHELF(category);

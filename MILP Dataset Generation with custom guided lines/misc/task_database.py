import sqlite3
import time

DB_NAME = "warehouse_tasks.db"

# -----------------------------
# CONNECT
# -----------------------------
def get_connection():

    return sqlite3.connect(DB_NAME)


# -----------------------------
# CREATE TABLE
# -----------------------------
def initialize_database():

    conn = get_connection()
    cursor = conn.cursor()

    cursor.execute(
        """
        CREATE TABLE IF NOT EXISTS tasks (
            task_id INTEGER PRIMARY KEY,
            task_type TEXT,
            PL TEXT,
            DL TEXT,
            assigned_robot TEXT,
            status TEXT,
            original_predicted_duration REAL,
            remaining_time REAL,
            actual_completion_time REAL
        )
        """
    )
    cursor.execute("DELETE FROM tasks")

    conn.commit()
    conn.close()


# -----------------------------
# INSERT TASK
# -----------------------------
def insert_task(
    task_id,
    task_type,
    PL,
    DL,
    assigned_robot,
    predicted_duration
):

    conn = get_connection()
    cursor = conn.cursor()

    cursor.execute(
        """
        INSERT INTO tasks (

            task_id,
            task_type,
            PL,
            DL,
            assigned_robot,
            status,
            original_predicted_duration,
            remaining_time

        )

        VALUES (?, ?, ?, ?, ?, ?, ?, ?)
        """,

        (
            task_id,
            task_type,
            PL,
            DL,
            assigned_robot,
            "QUEUED",
            round(predicted_duration, 2),
            round(predicted_duration, 2),
        )
    )

    conn.commit()
    conn.close()

# -----------------------------
# SQL ROW → DICTIONARY
# -----------------------------
def task_row_to_dict(row):

    if row is None:
        return None

    return {
        "task_id": row[0],
        "task_type": row[1],
        "PL": row[2],
        "DL": row[3],
        "assigned_robot": row[4],
        "status": row[5],
        "original_predicted_duration": row[6],
        "remaining_time": row[7],
    }

# -----------------------------
# GET TASK
# -----------------------------
def get_task(task_id):

    conn = get_connection()
    cursor = conn.cursor()

    cursor.execute(
        """
        SELECT *
        FROM tasks
        WHERE task_id = ?
        """,
        (task_id,)
    )

    row = cursor.fetchone()

    conn.close()
    
    return task_row_to_dict(row)


# -----------------------------
# UPDATE TASK STATUS
# -----------------------------
def update_task_status(task_id, status):

    conn = get_connection()
    cursor = conn.cursor()

    cursor.execute(
        """
        UPDATE tasks
        SET
            status = ?
        WHERE task_id = ?
        """,

        (
            status,
            task_id
        )
    )

    conn.commit()
    conn.close()


# -----------------------------
# UPDATE REMAINING TIME
# -----------------------------
def update_remaining_time(
    task_id,
    remaining_time
):

    conn = get_connection()
    cursor = conn.cursor()

    cursor.execute(
        """
        UPDATE tasks
        SET
            remaining_time = ?
        WHERE task_id = ?
        """,

        (
            round(remaining_time, 2),
            task_id
        )
    )

    conn.commit()
    conn.close()

# -----------------------------
# UPDATE ACTUAL COMPLETION TIME
# -----------------------------
def update_actual_completion_time(
    task_id,
    actual_completion_time
):

    conn = get_connection()
    cursor = conn.cursor()

    cursor.execute(
        """
        UPDATE tasks
        SET
            actual_completion_time = ?
        WHERE task_id = ?
        """,

        (
            round(actual_completion_time, 2),
            task_id
        )
    )

    conn.commit()
    conn.close()


# -----------------------------
# CALCULATE ROBOT RTUF
# -----------------------------
def calculate_robot_rtuf(robot_name):

    conn = get_connection()
    cursor = conn.cursor()

    cursor.execute(
        """
        SELECT SUM(remaining_time)
        FROM tasks
        WHERE
            assigned_robot = ?
            AND status != 'COMPLETED'
        """,
        (robot_name,)
    )

    result = cursor.fetchone()[0]

    conn.close()

    if result is None:
        return 0.0

    return round(result, 2)


# -----------------------------
# GET ROBOT TASKS
# -----------------------------
def get_robot_tasks(robot_name):

    conn = get_connection()
    cursor = conn.cursor()

    cursor.execute(
        """
        SELECT *
        FROM tasks
        WHERE
            assigned_robot = ?
            AND status != 'COMPLETED'
        """,

        (robot_name,)
    )

    rows = cursor.fetchall()
    conn.close()

    return [
        task_row_to_dict(row)
        for row in rows
    ]

# -----------------------------
# DEBUG PRINT TASKS
# -----------------------------
def print_all_tasks():

    conn = get_connection()
    cursor = conn.cursor()

    cursor.execute(
        """
        SELECT
            task_id,
            assigned_robot,
            status,
            original_predicted_duration,
            remaining_time
        FROM tasks
        """
    )

    rows = cursor.fetchall()

    print("\n========== TASK DATABASE ==========")

    for row in rows:

        print(
            f"Task {row[0]} | "
            f"Robot: {row[1]} | "
            f"Status: {row[2]} | "
            f"Predicted: {row[3]} sec | "
            f"Remaining: {row[4]} sec | "        )

    print("===================================\n")

    conn.close()
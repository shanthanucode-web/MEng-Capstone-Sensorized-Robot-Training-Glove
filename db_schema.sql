-- Database schema for glove training data
-- Run once to initialize: sqlite3 glove_dataset.db < db_schema.sql

CREATE TABLE IF NOT EXISTS sessions (
    session_id   INTEGER PRIMARY KEY AUTOINCREMENT,
    task         TEXT NOT NULL,
    subject      TEXT NOT NULL,
    recorded_at  TEXT NOT NULL,
    source_file  TEXT NOT NULL,
    row_count    INTEGER
);

CREATE TABLE IF NOT EXISTS frames (
    frame_id     INTEGER PRIMARY KEY AUTOINCREMENT,
    session_id   INTEGER NOT NULL REFERENCES sessions(session_id),
    timestamp_ms INTEGER NOT NULL,

    -- IMU wrist orientation
    qw REAL, qx REAL, qy REAL, qz REAL,

    -- Flex sensors - raw normalized values and calibrated degrees
    flex_thumb   REAL, flex_thumb_deg   REAL,
    flex_index   REAL, flex_index_deg   REAL,
    flex_middle  REAL, flex_middle_deg  REAL,
    flex_ring    REAL, flex_ring_deg    REAL,
    flex_pinky   REAL, flex_pinky_deg   REAL,

    -- FSRs - raw normalized values and binary contact flags
    fsr_index    REAL, fsr_index_contact  INTEGER,
    fsr_middle   REAL, fsr_middle_contact INTEGER,
    fsr_thumb    REAL, fsr_thumb_contact  INTEGER
);

CREATE INDEX IF NOT EXISTS idx_frames_session ON frames(session_id);
CREATE INDEX IF NOT EXISTS idx_frames_ts      ON frames(timestamp_ms);

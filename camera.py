import cv2
import numpy as np
import time
import math
import threading
from flask import Flask, Response, jsonify

# ==========================
# Configuration
# ==========================

WIDTH = 640
HEIGHT = 480
FPS = 30
HFOV_DEG = 70.0

# Leave as None unless you want distance estimates.
TARGET_DIAMETER_IN = None

MIN_AREA = 450
MIN_CIRCULARITY = 0.42
MAX_ASPECT_ERROR = 0.75
MIN_CONFIDENCE_TO_CREATE_TRACK = 0.55

MIN_HITS_TO_SHOW = 3
MAX_MATCH_DISTANCE_PX = 180
MAX_MISSED_FRAMES_SEEN = 6
MAX_MISSED_FRAMES_PREDICTED = 3

SMOOTHING_ALPHA = 0.72
VELOCITY_ALPHA = 0.50
PREDICTION_GAIN = 1.0
DUPLICATE_IOU_THRESHOLD = 0.35

GROUP_DISTANCE_PX = 130
DRAW_AR_PATH = True

DASHBOARD_PORT = 5800

# Circle splitting settings
ENABLE_CIRCLE_SPLIT = True
MERGED_AREA_MIN = 2500
MERGED_ASPECT_ERROR_MIN = 0.35
MIN_CIRCLE_YELLOW_FILL = 0.45
MAX_CIRCLE_OVERLAP = 0.45
HOUGH_DP = 1.2
HOUGH_MIN_DIST = 35
HOUGH_PARAM1 = 80
HOUGH_PARAM2 = 18
HOUGH_MIN_RADIUS = 12
HOUGH_MAX_RADIUS = 120

# ==========================
# Globals
# ==========================

app = Flask(__name__)

next_track_id = 1
tracks = []

latest_frame_jpeg = None
latest_data = {
    "fps": 0,
    "raw_detections": 0,
    "visible_tracks": [],
    "groups": [],
    "largest_group": None,
    "robot_target": None,
}

data_lock = threading.Lock()
running = True

# ==========================
# Utility
# ==========================

def box_iou(a, b):
    ax1, ay1 = a["x"], a["y"]
    ax2, ay2 = a["x"] + a["w"], a["y"] + a["h"]
    bx1, by1 = b["x"], b["y"]
    bx2, by2 = b["x"] + b["w"], b["y"] + b["h"]

    ix1 = max(ax1, bx1)
    iy1 = max(ay1, by1)
    ix2 = min(ax2, bx2)
    iy2 = min(ay2, by2)

    iw = max(0, ix2 - ix1)
    ih = max(0, iy2 - iy1)
    inter = iw * ih

    area_a = max(1, a["w"] * a["h"])
    area_b = max(1, b["w"] * b["h"])
    union = area_a + area_b - inter

    return inter / union


def circle_overlap(c1, c2):
    dx = c1["center_x"] - c2["center_x"]
    dy = c1["center_y"] - c2["center_y"]
    d = math.sqrt(dx * dx + dy * dy)
    r1 = max(c1["w"], c1["h"]) / 2
    r2 = max(c2["w"], c2["h"]) / 2

    if d >= r1 + r2:
        return 0.0

    smaller = min(r1, r2)
    if d <= abs(r1 - r2):
        return 1.0

    return max(0.0, 1.0 - d / (r1 + r2))


def predicted_center(track):
    return (
        track["center_x"] + PREDICTION_GAIN * track.get("vx", 0.0),
        track["center_y"] + PREDICTION_GAIN * track.get("vy", 0.0),
    )


def distance_to_prediction(track, det):
    px, py = predicted_center(track)
    dx = px - det["center_x"]
    dy = py - det["center_y"]
    return math.sqrt(dx * dx + dy * dy)


# ==========================
# Detection
# ==========================

def make_detection(x, y, w, h, area, circularity, aspect_error, focal_px, source="contour"):
    center_x = x + w / 2
    center_y = y + h / 2
    object_width_px = max(w, h)

    yaw_deg = ((center_x - WIDTH / 2) / (WIDTH / 2)) * (HFOV_DEG / 2)

    if TARGET_DIAMETER_IN is None:
        distance_in = None
    else:
        distance_in = (TARGET_DIAMETER_IN * focal_px) / object_width_px

    circularity_score = min(circularity / 0.85, 1.0)
    area_score = min(area / 2500.0, 1.0)
    aspect_score = max(0.0, 1.0 - min(aspect_error, 1.0))

    confidence = (
        0.50 * circularity_score +
        0.25 * area_score +
        0.25 * aspect_score
    )

    if source == "circle":
        confidence = min(1.0, confidence + 0.08)

    return {
        "x": int(x),
        "y": int(y),
        "w": int(w),
        "h": int(h),
        "center_x": center_x,
        "center_y": center_y,
        "area": area,
        "circularity": circularity,
        "aspect_error": aspect_error,
        "yaw_deg": yaw_deg,
        "distance_in": distance_in,
        "confidence": confidence,
        "source": source,
    }


def yellow_fill_for_circle(mask, cx, cy, r):
    circle_mask = np.zeros(mask.shape, dtype=np.uint8)
    cv2.circle(circle_mask, (int(cx), int(cy)), int(r), 255, -1)

    circle_area = cv2.countNonZero(circle_mask)
    if circle_area <= 0:
        return 0.0

    yellow_inside = cv2.countNonZero(cv2.bitwise_and(mask, circle_mask))
    return yellow_inside / circle_area


def detect_circles_in_blob(frame, mask, x, y, w, h, focal_px):
    roi_frame = frame[y:y+h, x:x+w]
    roi_mask = mask[y:y+h, x:x+w]

    if roi_frame.size == 0 or roi_mask.size == 0:
        return []

    gray = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (7, 7), 1.5)

    min_radius = max(HOUGH_MIN_RADIUS, int(min(w, h) * 0.18))
    max_radius = min(HOUGH_MAX_RADIUS, int(max(w, h) * 0.65))

    if max_radius <= min_radius:
        return []

    circles = cv2.HoughCircles(
        gray,
        cv2.HOUGH_GRADIENT,
        dp=HOUGH_DP,
        minDist=HOUGH_MIN_DIST,
        param1=HOUGH_PARAM1,
        param2=HOUGH_PARAM2,
        minRadius=min_radius,
        maxRadius=max_radius,
    )

    if circles is None:
        return []

    circles = np.round(circles[0, :]).astype("int")
    detections = []

    for cx, cy, r in circles:
        global_cx = x + cx
        global_cy = y + cy

        if global_cx < 0 or global_cx >= WIDTH or global_cy < 0 or global_cy >= HEIGHT:
            continue

        fill = yellow_fill_for_circle(mask, global_cx, global_cy, r)
        if fill < MIN_CIRCLE_YELLOW_FILL:
            continue

        bx = global_cx - r
        by = global_cy - r
        bw = 2 * r
        bh = 2 * r

        bx = max(0, min(bx, WIDTH - bw))
        by = max(0, min(by, HEIGHT - bh))

        area = math.pi * r * r
        det = make_detection(
            bx,
            by,
            bw,
            bh,
            area,
            0.88,
            0.0,
            focal_px,
            source="circle"
        )
        det["yellow_fill"] = fill

        duplicate = False
        for existing in detections:
            if circle_overlap(det, existing) > MAX_CIRCLE_OVERLAP:
                duplicate = True
                break

        if not duplicate:
            detections.append(det)

    detections.sort(key=lambda d: (d.get("yellow_fill", 0), d["confidence"]), reverse=True)
    return detections


def detect_balls(frame, focal_px):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_yellow = np.array([18, 75, 75])
    upper_yellow = np.array([42, 255, 255])

    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    detections = []

    for c in contours:
        area = cv2.contourArea(c)
        if area < MIN_AREA:
            continue

        perimeter = cv2.arcLength(c, True)
        if perimeter <= 0:
            continue

        circularity = 4.0 * math.pi * area / (perimeter * perimeter)

        x, y, w, h = cv2.boundingRect(c)
        if w <= 0 or h <= 0:
            continue

        aspect_ratio = w / h
        aspect_error = abs(1.0 - aspect_ratio)

        likely_merged = (
            ENABLE_CIRCLE_SPLIT and
            area >= MERGED_AREA_MIN and
            (aspect_error >= MERGED_ASPECT_ERROR_MIN or circularity < MIN_CIRCULARITY)
        )

        if likely_merged:
            circle_dets = detect_circles_in_blob(frame, mask, x, y, w, h, focal_px)

            if len(circle_dets) >= 2:
                detections.extend(circle_dets)
                continue

        if circularity < MIN_CIRCULARITY:
            continue

        if aspect_error > MAX_ASPECT_ERROR:
            continue

        det = make_detection(x, y, w, h, area, circularity, aspect_error, focal_px, source="contour")

        if det["confidence"] < MIN_CONFIDENCE_TO_CREATE_TRACK:
            continue

        detections.append(det)

    detections.sort(key=lambda d: d["confidence"], reverse=True)
    return detections, mask


# ==========================
# Tracking
# ==========================

def remove_duplicate_tracks(track_list):
    kept = []

    sorted_tracks = sorted(
        track_list,
        key=lambda t: (t["missed"], -t["hits"], -t["confidence"], -t["area"])
    )

    for t in sorted_tracks:
        duplicate = False

        for k in kept:
            if box_iou(t, k) > DUPLICATE_IOU_THRESHOLD:
                duplicate = True
                break

        if not duplicate:
            kept.append(t)

    return kept


def update_tracks(detections):
    global next_track_id, tracks

    for t in tracks:
        t["matched"] = False
        t["missed"] += 1

    for det in detections:
        best_track = None
        best_score = 999999

        for track in tracks:
            if track["matched"]:
                continue

            d = distance_to_prediction(track, det)
            size_diff = abs(max(track["w"], track["h"]) - max(det["w"], det["h"]))
            score = d + 0.25 * size_diff - 8.0 * min(track.get("hits", 1), 6)

            if d < MAX_MATCH_DISTANCE_PX and score < best_score:
                best_score = score
                best_track = track

        if best_track is None:
            det["id"] = next_track_id
            det["hits"] = 1
            det["missed"] = 0
            det["matched"] = True
            det["vx"] = 0.0
            det["vy"] = 0.0
            tracks.append(det)
            next_track_id += 1
        else:
            old_cx = best_track["center_x"]
            old_cy = best_track["center_y"]

            measured_vx = det["center_x"] - old_cx
            measured_vy = det["center_y"] - old_cy

            best_track["vx"] = VELOCITY_ALPHA * measured_vx + (1 - VELOCITY_ALPHA) * best_track.get("vx", 0.0)
            best_track["vy"] = VELOCITY_ALPHA * measured_vy + (1 - VELOCITY_ALPHA) * best_track.get("vy", 0.0)

            a = SMOOTHING_ALPHA

            best_track["x"] = int(a * det["x"] + (1 - a) * best_track["x"])
            best_track["y"] = int(a * det["y"] + (1 - a) * best_track["y"])
            best_track["w"] = int(a * det["w"] + (1 - a) * best_track["w"])
            best_track["h"] = int(a * det["h"] + (1 - a) * best_track["h"])

            best_track["center_x"] = a * det["center_x"] + (1 - a) * best_track["center_x"]
            best_track["center_y"] = a * det["center_y"] + (1 - a) * best_track["center_y"]
            best_track["yaw_deg"] = a * det["yaw_deg"] + (1 - a) * best_track["yaw_deg"]

            if det["distance_in"] is not None and best_track.get("distance_in") is not None:
                best_track["distance_in"] = a * det["distance_in"] + (1 - a) * best_track["distance_in"]
            else:
                best_track["distance_in"] = det["distance_in"]

            best_track["area"] = det["area"]
            best_track["circularity"] = det["circularity"]
            best_track["aspect_error"] = det["aspect_error"]
            best_track["confidence"] = det["confidence"]
            best_track["source"] = det.get("source", "contour")

            best_track["hits"] += 1
            best_track["missed"] = 0
            best_track["matched"] = True

    for t in tracks:
        if not t["matched"] and t["missed"] > 0:
            t["center_x"] += t.get("vx", 0.0)
            t["center_y"] += t.get("vy", 0.0)
            t["x"] = int(t["center_x"] - t["w"] / 2)
            t["y"] = int(t["center_y"] - t["h"] / 2)
            t["vx"] *= 0.80
            t["vy"] *= 0.80

    filtered = []

    for t in tracks:
        if t["hits"] < MIN_HITS_TO_SHOW and t["missed"] > MAX_MISSED_FRAMES_PREDICTED:
            continue

        if t["hits"] >= MIN_HITS_TO_SHOW and t["missed"] > MAX_MISSED_FRAMES_SEEN:
            continue

        filtered.append(t)

    tracks = remove_duplicate_tracks(filtered)

    visible = [
        t for t in tracks
        if t["hits"] >= MIN_HITS_TO_SHOW
    ]

    visible.sort(key=lambda t: (t["missed"], -t["confidence"], -t["area"]))
    return visible


# ==========================
# Grouping
# ==========================

def group_tracks(visible_tracks):
    groups = []
    used = set()

    for track in visible_tracks:
        if track["id"] in used:
            continue

        group = [track]
        used.add(track["id"])

        changed = True
        while changed:
            changed = False

            for other in visible_tracks:
                if other["id"] in used:
                    continue

                for member in group:
                    dx = other["center_x"] - member["center_x"]
                    dy = other["center_y"] - member["center_y"]
                    dist = math.sqrt(dx * dx + dy * dy)

                    if dist <= GROUP_DISTANCE_PX:
                        group.append(other)
                        used.add(other["id"])
                        changed = True
                        break

        groups.append(group)

    output = []

    for idx, group in enumerate(groups):
        count = len(group)
        center_x = sum(t["center_x"] for t in group) / count
        center_y = sum(t["center_y"] for t in group) / count
        area = sum(t["area"] for t in group)
        yaw_deg = ((center_x - WIDTH / 2) / (WIDTH / 2)) * (HFOV_DEG / 2)

        distances = [t["distance_in"] for t in group if t["distance_in"] is not None]
        avg_distance = sum(distances) / len(distances) if distances else None

        output.append({
            "id": idx + 1,
            "count": count,
            "center_x": center_x,
            "center_y": center_y,
            "yaw_deg": yaw_deg,
            "avg_distance_in": avg_distance,
            "area": area,
            "track_ids": [t["id"] for t in group],
        })

    output.sort(key=lambda g: (-g["count"], -g["area"]))
    return output


# ==========================
# Drawing / JSON
# ==========================

def track_to_json(t):
    return {
        "id": t["id"],
        "x": int(t["x"]),
        "y": int(t["y"]),
        "w": int(t["w"]),
        "h": int(t["h"]),
        "center_x": round(t["center_x"], 1),
        "center_y": round(t["center_y"], 1),
        "yaw_deg": round(t["yaw_deg"], 2),
        "distance_in": None if t["distance_in"] is None else round(t["distance_in"], 1),
        "confidence": round(t["confidence"], 2),
        "missed": t["missed"],
        "hits": t["hits"],
        "status": "SEEN" if t["missed"] == 0 else "PRED",
        "source": t.get("source", "contour"),
    }


def draw_overlay(frame, visible_tracks, groups, robot_target, fps_value, raw_count):
    for group in groups:
        gx = int(group["center_x"])
        gy = int(group["center_y"])

        cv2.circle(frame, (gx, gy), 9, (255, 255, 0), 2)

        cv2.putText(
            frame,
            f"GROUP {group['id']} count={group['count']} yaw={group['yaw_deg']:.1f}",
            (gx + 10, gy),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (255, 255, 0),
            2
        )

    if DRAW_AR_PATH and robot_target is not None:
        start = (WIDTH // 2, HEIGHT - 10)
        end = (int(robot_target["center_x"]), int(robot_target["center_y"]))
        cv2.line(frame, start, end, (255, 0, 0), 3)
        cv2.circle(frame, end, 8, (255, 0, 0), 2)

    for t in visible_tracks:
        x = int(t["x"])
        y = int(t["y"])
        w = int(t["w"])
        h = int(t["h"])

        if robot_target is not None and t["id"] == robot_target["id"]:
            color = (255, 0, 0)
            status = "BEST"
            thickness = 3
        elif t["missed"] == 0:
            color = (0, 255, 0)
            status = "SEEN"
            thickness = 2
        else:
            color = (0, 180, 255)
            status = "PRED"
            thickness = 1

        if t.get("source") == "circle":
            color = (255, 0, 255) if status != "BEST" else color

        cv2.rectangle(frame, (x, y), (x + w, y + h), color, thickness)
        cv2.circle(frame, (int(t["center_x"]), int(t["center_y"])), 4, (0, 0, 255), -1)

        label = (
            f"BALL #{t['id']} {status} "
            f"{t.get('source','contour')} "
            f"yaw={t['yaw_deg']:.1f} "
            f"conf={t['confidence']:.2f}"
        )

        cv2.putText(
            frame,
            label,
            (x, max(25, y - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.43,
            color,
            2
        )

    cv2.putText(frame, f"FPS: {fps_value}", (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    cv2.putText(frame, f"Raw detections: {raw_count}", (10, 55),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    cv2.putText(frame, f"Visible tracks: {len(visible_tracks)}", (10, 80),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    if groups:
        largest = groups[0]
        cv2.putText(frame, f"Largest group: {largest['count']} balls yaw={largest['yaw_deg']:.1f}",
                    (10, 105), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

    if robot_target is not None:
        cv2.putText(frame, f"Robot target yaw: {robot_target['yaw_deg']:.1f}",
                    (10, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)


# ==========================
# Vision thread
# ==========================

def vision_loop():
    global latest_frame_jpeg, latest_data, running

    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, FPS)

    if not cap.isOpened():
        raise RuntimeError("Failed to open camera")

    focal_px = WIDTH / (2.0 * math.tan(math.radians(HFOV_DEG / 2.0)))

    last_time = time.time()
    frame_counter = 0
    fps_value = 0

    while running:
        ok, frame = cap.read()
        if not ok:
            continue

        detections, mask = detect_balls(frame, focal_px)
        visible_tracks = update_tracks(detections)

        groups = group_tracks(visible_tracks)
        largest_group = groups[0] if groups else None

        seen_tracks = [t for t in visible_tracks if t["missed"] == 0]
        robot_target = seen_tracks[0] if seen_tracks else None

        frame_counter += 1
        now = time.time()

        if now - last_time >= 1.0:
            fps_value = frame_counter
            frame_counter = 0
            last_time = now

        annotated = frame.copy()
        draw_overlay(annotated, visible_tracks, groups, robot_target, fps_value, len(detections))

        ok_jpg, buffer = cv2.imencode(".jpg", annotated, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if not ok_jpg:
            continue

        data = {
            "fps": fps_value,
            "raw_detections": len(detections),
            "visible_tracks": [track_to_json(t) for t in visible_tracks],
            "groups": [
                {
                    "id": g["id"],
                    "count": g["count"],
                    "center_x": round(g["center_x"], 1),
                    "center_y": round(g["center_y"], 1),
                    "yaw_deg": round(g["yaw_deg"], 2),
                    "avg_distance_in": None if g["avg_distance_in"] is None else round(g["avg_distance_in"], 1),
                    "area": round(g["area"], 1),
                    "track_ids": g["track_ids"],
                }
                for g in groups
            ],
            "largest_group": None if largest_group is None else {
                "id": largest_group["id"],
                "count": largest_group["count"],
                "yaw_deg": round(largest_group["yaw_deg"], 2),
                "avg_distance_in": None if largest_group["avg_distance_in"] is None else round(largest_group["avg_distance_in"], 1),
                "track_ids": largest_group["track_ids"],
            },
            "robot_target": None if robot_target is None else track_to_json(robot_target),
        }

        with data_lock:
            latest_frame_jpeg = buffer.tobytes()
            latest_data = data

    cap.release()


# ==========================
# Flask dashboard
# ==========================

@app.route("/")
def index():
    return """
<!DOCTYPE html>
<html>
<head>
    <title>8324 Game Piece Vision</title>
    <style>
        body { background: #111; color: #eee; font-family: Arial, sans-serif; margin: 0; padding: 20px; }
        h1 { margin-top: 0; }
        .layout { display: flex; gap: 20px; align-items: flex-start; }
        .video { border: 2px solid #444; max-width: 70vw; }
        .panel { background: #1c1c1c; border: 1px solid #444; border-radius: 8px; padding: 15px; min-width: 320px; }
        .stat { margin-bottom: 8px; }
        .warn { color: #ffcc00; }
        .best { color: #66aaff; }
        table { border-collapse: collapse; width: 100%; font-size: 14px; }
        th, td { border-bottom: 1px solid #333; padding: 5px; text-align: left; }
    </style>
</head>
<body>
    <h1>8324 Game Piece Vision Dashboard</h1>

    <div class="layout">
        <div><img class="video" src="/video"></div>

        <div class="panel">
            <h2>Status</h2>
            <div class="stat">FPS: <span id="fps">--</span></div>
            <div class="stat">Raw detections: <span id="raw">--</span></div>
            <div class="stat">Visible balls: <span id="visible">--</span></div>

            <h2>Largest Group</h2>
            <div id="largest" class="warn">None</div>

            <h2>Robot Target</h2>
            <div id="target" class="best">None</div>

            <h2>Groups</h2>
            <table>
                <thead><tr><th>ID</th><th>Count</th><th>Yaw</th><th>Tracks</th></tr></thead>
                <tbody id="groups"></tbody>
            </table>

            <h2>Balls</h2>
            <table>
                <thead><tr><th>ID</th><th>Status</th><th>Source</th><th>Yaw</th><th>Conf</th></tr></thead>
                <tbody id="balls"></tbody>
            </table>
        </div>
    </div>

<script>
async function updateData() {
    try {
        const res = await fetch('/data');
        const data = await res.json();

        document.getElementById('fps').textContent = data.fps;
        document.getElementById('raw').textContent = data.raw_detections;
        document.getElementById('visible').textContent = data.visible_tracks.length;

        if (data.largest_group) {
            document.getElementById('largest').innerHTML =
                `Group ${data.largest_group.id}: ${data.largest_group.count} balls, yaw ${data.largest_group.yaw_deg}°`;
        } else {
            document.getElementById('largest').textContent = 'None';
        }

        if (data.robot_target) {
            document.getElementById('target').innerHTML =
                `Ball #${data.robot_target.id}: yaw ${data.robot_target.yaw_deg}°, conf ${data.robot_target.confidence}`;
        } else {
            document.getElementById('target').textContent = 'None';
        }

        const groupsBody = document.getElementById('groups');
        groupsBody.innerHTML = '';
        for (const g of data.groups) {
            groupsBody.innerHTML += `
                <tr>
                    <td>${g.id}</td>
                    <td>${g.count}</td>
                    <td>${g.yaw_deg}°</td>
                    <td>${g.track_ids.join(', ')}</td>
                </tr>
            `;
        }

        const ballsBody = document.getElementById('balls');
        ballsBody.innerHTML = '';
        for (const b of data.visible_tracks) {
            ballsBody.innerHTML += `
                <tr>
                    <td>${b.id}</td>
                    <td>${b.status}</td>
                    <td>${b.source}</td>
                    <td>${b.yaw_deg}°</td>
                    <td>${b.confidence}</td>
                </tr>
            `;
        }
    } catch (e) {
        console.log(e);
    }
}

setInterval(updateData, 250);
updateData();
</script>
</body>
</html>
"""


@app.route("/video")
def video():
    def generate():
        while True:
            with data_lock:
                frame = latest_frame_jpeg

            if frame is None:
                time.sleep(0.05)
                continue

            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" +
                frame +
                b"\r\n"
            )

            time.sleep(0.03)

    return Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/data")
def data():
    with data_lock:
        return jsonify(latest_data)


# ==========================
# Main
# ==========================

if __name__ == "__main__":
    thread = threading.Thread(target=vision_loop, daemon=True)
    thread.start()

    print("")
    print("Dashboard running.")
    print("On this Beelink, open: http://127.0.0.1:5800")
    print("From another device, open: http://<BEELINK-IP>:5800")
    print("Find Beelink IP with: hostname -I")
    print("")

    app.run(host="0.0.0.0", port=DASHBOARD_PORT, debug=False, threaded=True)
#!/usr/bin/env python3
"""
auto_zone_cali.py

Auto-calibrate zones for DuckieRace from two PNG snapshots (one per overhead camera),
then overwrite:

  config/zones_cam1.yaml
  config/zones_cam2.yaml

Per image assumptions:
- (Calibration snapshot) You try to keep ONLY the 3 calibration tags visible.
- If extra tags appear, this script will FAIL loudly (by default) so you don't generate wrong zones.

Tag assignment (by center.x):
- Leftmost  -> fuel_top_left_id
- Middle   -> fuel_top_right_id
- Rightmost -> merge_tag_id

Fuel zone:
- Top edge = line from fuel_top_left center (A) to fuel_top_right center (B)
- fuel_short_len = K_cam * abs(B.y - M.y)
- Oriented rectangle (A,B,C,D) -> axis-aligned bbox written to YAML

Merge zone (single rectangle, axis-aligned):
- merge_tag center M is TOP-LEFT corner
- width  = fuel_long_len / 3
- height = 3 * fuel_short_len

Charge gate zone (single rectangle, axis-aligned, left of fuel):
- right side x_max = fuel_x_min - GATE_RIGHT_RATIO * fuel_long_len
- gate is square: side = GATE_SHORT_K * fuel_short_len
- top side y_min = merge_tag center y

Finish line:
- vertical in the frame
- x = midpoint of fuel bbox
- direction = left_to_right

NEW: ignore_zones (mask areas for static calibration tags)
- We compute 3 rectangles around the detected calibration tags (slightly enlarged).
- These rectangles are written to YAML as `ignore_zones` so the runtime manager can black them out.
- In the preview, we also draw these cover rectangles so you can visually verify they are correct.
"""

import platform
import os
import cv2
import numpy as np
import matplotlib.pyplot as plt
import yaml
from matplotlib.patches import Rectangle

# ================= HARD-CODED SETTINGS =================
CAM1_IMAGE_PATH = "test/image1.png"
CAM2_IMAGE_PATH = "test/image2.png"

CONFIG_DIR = "config"
CAM1_YAML_PATH = os.path.join(CONFIG_DIR, "zones_cam1.yaml")
CAM2_YAML_PATH = os.path.join(CONFIG_DIR, "zones_cam2.yaml")

CAM1_CAMERA_NAME = "usb_cam_1"
CAM2_CAMERA_NAME = "usb_cam_2"
CAM1_IMAGE_TOPIC = "/usb_cam_1/image_raw"
CAM2_IMAGE_TOPIC = "/usb_cam_2/image_raw"

TAG_FAMILY = "tag36h11"
DECIMATE = 1.0
BLUR_KERNEL = 0  # 0 = no blur, else odd number

# Per-camera tuned fuel short-side scaling
CAM1_SHORT_K = 6
CAM2_SHORT_K = 3.2

# Charge gate tuning knobs
GATE_RIGHT_RATIO = 0.5   # x_max = fuel_x_min - ratio * fuel_long_len
GATE_SHORT_K = 1.5       # square gate side = gate_short_k * fuel_short_len

# Ignore-zone (cover) tuning knob
COVER_PAD_PX = 20        # enlarge each tag bbox by this many pixels on each side

# Strictness: keep True for calibration snapshots (safer)
REQUIRE_EXACTLY_3_TAGS = True

# Preview plot (set False if running headless)
SHOW_PLOT = True

# Low-resolution snapshots can make tags too small for a single detector pass.
DETECT_SCALES = (1.0, 2.0, 3.0)
# =======================================================

# -------- Detector selection --------
if platform.system() == "Windows":
    from pupil_apriltags import Detector
    # If you need DLL path on your Windows machine, uncomment and adjust:
    # os.add_dll_directory(
    #     "C:/Users/yzeng/AppData/Local/anaconda3/envs/apriltag_env/lib/site-packages/pupil_apriltags.libs"
    # )
    detector = Detector(
        families=TAG_FAMILY,
        nthreads=4,
        quad_decimate=DECIMATE,
        quad_sigma=0.0,
        refine_edges=1,
    )
else:
    from dt_apriltags import Detector
    detector = Detector(
        families=TAG_FAMILY,
        quad_decimate=DECIMATE,
    )


def _unit(v: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n < 1e-9:
        raise ValueError("Zero-length vector")
    return v / n


def _bbox_from_points(pts: np.ndarray):
    xs = pts[:, 0]
    ys = pts[:, 1]
    x_min = int(np.floor(xs.min()))
    x_max = int(np.ceil(xs.max()))
    y_min = int(np.floor(ys.min()))
    y_max = int(np.ceil(ys.max()))
    return x_min, x_max, y_min, y_max


def _tag_cover_rect(corners: np.ndarray, img_w: int, img_h: int, pad: int):
    x_min = int(np.floor(corners[:, 0].min())) - pad
    x_max = int(np.ceil(corners[:, 0].max())) + pad
    y_min = int(np.floor(corners[:, 1].min())) - pad
    y_max = int(np.ceil(corners[:, 1].max())) + pad

    x_min = max(0, x_min)
    y_min = max(0, y_min)
    x_max = min(img_w, x_max)
    y_max = min(img_h, y_max)
    return {"x_min": x_min, "x_max": x_max, "y_min": y_min, "y_max": y_max}


def _detect_scaled(gray: np.ndarray, scale: float):
    if scale == 1.0:
        dets = detector.detect(gray)
    else:
        up = cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
        dets = detector.detect(up)

    out = []
    for d in dets:
        center = np.array(d.center, float) / scale
        corners = np.array(d.corners, float) / scale
        out.append((int(d.tag_id), center, corners))
    return out


def _gray_variants(gray: np.ndarray):
    yield "gray", gray
    yield "equalize", cv2.equalizeHist(gray)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    yield "clahe", clahe.apply(gray)
    blur = cv2.GaussianBlur(gray, (0, 0), 1.0)
    yield "sharp", cv2.addWeighted(gray, 1.8, blur, -0.8, 0)


def _detect_tags_with_covers(bgr: np.ndarray):
    """Detect tags and compute ignore_zones rectangles around EACH detected tag."""
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    if BLUR_KERNEL > 0:
        k = BLUR_KERNEL if (BLUR_KERNEL % 2 == 1) else (BLUR_KERNEL + 1)
        gray = cv2.GaussianBlur(gray, (k, k), 0)

    attempts = []
    dets = []
    for variant_name, variant_gray in _gray_variants(gray):
        for scale in DETECT_SCALES:
            candidate = _detect_scaled(variant_gray, scale)
            attempts.append(f"{variant_name}@{scale:g}x={len(candidate)}")
            if len(candidate) == 3:
                dets = candidate
                break
            if not REQUIRE_EXACTLY_3_TAGS and len(candidate) >= 3:
                dets = candidate
                break
        if dets:
            break

    if REQUIRE_EXACTLY_3_TAGS and (len(dets) != 3):
        raise RuntimeError(f"Expected exactly 3 tags; attempts: {', '.join(attempts)}")

    if len(dets) < 3:
        raise RuntimeError(f"Expected at least 3 tags; attempts: {', '.join(attempts)}")

    out = []
    covers = []
    img_h, img_w = gray.shape[:2]

    for tag_id, center, corners in dets:
        out.append((tag_id, center, corners))
        covers.append(_tag_cover_rect(corners, img_w=img_w, img_h=img_h, pad=COVER_PAD_PX))

    # If more than 3 tags and strictness is off, take 3 by x-center after sorting.
    # (Still deterministic.)
    out.sort(key=lambda t: t[1][0])
    if not REQUIRE_EXACTLY_3_TAGS and len(out) > 3:
        out = out[:3]
        covers = covers[:3]

    return out, covers


def _plot_preview(rgb, camera_name, det_triplet, fuel_bbox, merge_bbox, gate_bbox, finish_x, ignore_zones):
    (fuel_tl_id, A, cornersA), (fuel_tr_id, B, cornersB), (merge_id, M, cornersM) = det_triplet

    fig, ax = plt.subplots(num=f"{camera_name} zones preview")
    ax.imshow(rgb)
    ax.axis("off")

    # tag outlines + IDs
    for tag_id, center, corners in [(fuel_tl_id, A, cornersA), (fuel_tr_id, B, cornersB), (merge_id, M, cornersM)]:
        xs = list(corners[:, 0]) + [corners[0, 0]]
        ys = list(corners[:, 1]) + [corners[0, 1]]
        ax.plot(xs, ys)
        ax.text(center[0], center[1], str(tag_id), color="red", fontsize=14, ha="center", va="center")

    def draw_bbox(b, label):
        x0, x1, y0, y1 = b
        poly = np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1], [x0, y0]], float)
        ax.plot(poly[:, 0], poly[:, 1])
        ax.text(x0, y0 - 12, label, color="yellow", fontsize=12, ha="left", va="bottom")

    draw_bbox(fuel_bbox, "FUEL")
    draw_bbox(merge_bbox, "MERGE")
    draw_bbox(gate_bbox, "CHARGE_GATE")

    # finish line (vertical)
    ax.plot([finish_x, finish_x], [0, rgb.shape[0]])
    ax.text(finish_x + 5, 25, "FINISH", color="yellow", fontsize=12, ha="left", va="bottom")

    # ignore zones as black covers (so you see what will be masked in runtime)
    for z in ignore_zones:
        w = z["x_max"] - z["x_min"]
        h = z["y_max"] - z["y_min"]
        ax.add_patch(Rectangle((z["x_min"], z["y_min"]), w, h, facecolor="black", edgecolor="white", alpha=0.35))
        ax.text(z["x_min"], z["y_min"] - 12, "IGNORE", color="white", fontsize=10, ha="left", va="bottom")

    plt.show()


def build_zone_yaml_from_image(image_path: str, short_k: float, camera_name: str, image_topic: str):
    bgr = cv2.imread(image_path, cv2.IMREAD_COLOR)
    if bgr is None:
        raise FileNotFoundError(f"Could not read image: {image_path}")

    det_triplet, ignore_zones = _detect_tags_with_covers(bgr)
    (fuel_tl_id, A, cornersA), (fuel_tr_id, B, cornersB), (merge_id, M, cornersM) = det_triplet

    # ---- Fuel oriented rectangle -> bbox ----
    u = _unit(B - A)
    perp = np.array([-u[1], u[0]], float)

    # choose perp direction to point toward merge tag
    mid_top = 0.5 * (A + B)
    if np.dot(M - mid_top, perp) < 0:
        perp = -perp

    fuel_long_len = float(np.linalg.norm(B - A))
    fuel_short_len = float(short_k) * abs(B[1] - M[1])

    D = A + perp * fuel_short_len
    C = B + perp * fuel_short_len

    fuel_pts = np.vstack([A, B, C, D])
    fuel_x_min, fuel_x_max, fuel_y_min, fuel_y_max = _bbox_from_points(fuel_pts)

    # ---- Merge zone (axis-aligned) ----
    merge_w = fuel_long_len / 3.0
    merge_h = 4.5 * fuel_short_len  # final rule

    merge_tl = M
    merge_tr = M + np.array([merge_w, 0.0])
    merge_br = M + np.array([merge_w, merge_h])
    merge_bl = M + np.array([0.0, merge_h])

    merge_pts = np.vstack([merge_tl, merge_tr, merge_br, merge_bl])
    merge_x_min, merge_x_max, merge_y_min, merge_y_max = _bbox_from_points(merge_pts)

    # ---- Charge gate zone (axis-aligned, left of fuel) ----
    gate_side = float(GATE_SHORT_K) * fuel_short_len
    gate_x_max = float(fuel_x_min) - float(GATE_RIGHT_RATIO) * fuel_long_len
    gate_x_min = gate_x_max - gate_side

    gate_y_min = float(M[1])  # top side at merge tag y
    gate_y_max = gate_y_min + gate_side

    gate_x_min_i = int(np.floor(min(gate_x_min, gate_x_max)))
    gate_x_max_i = int(np.ceil(max(gate_x_min, gate_x_max)))
    gate_y_min_i = int(np.floor(min(gate_y_min, gate_y_max)))
    gate_y_max_i = int(np.ceil(max(gate_y_min, gate_y_max)))

    # ---- Finish line ----
    finish_x = int(round(0.5 * (fuel_x_min + fuel_x_max)))

    zone_yaml = {
        "camera_name": camera_name,
        "image_topic": image_topic,
        "has_finish_line": True,

        "merge_zones": [
            {
                "x_min": merge_x_min,
                "x_max": merge_x_max,
                "y_min": merge_y_min,
                "y_max": merge_y_max,
            }
        ],

        "fuel_zone": {
            "x_min": fuel_x_min,
            "x_max": fuel_x_max,
            "y_min": fuel_y_min,
            "y_max": fuel_y_max,
        },

        "charge_gate_zone": {
            "x_min": gate_x_min_i,
            "x_max": gate_x_max_i,
            "y_min": gate_y_min_i,
            "y_max": gate_y_max_i,
        },

        # NEW: rectangles that should be masked out in the runtime manager before detection
        "ignore_zones": ignore_zones,

        "finish_line": {
            "x": finish_x,
            "direction": "left_to_right",
        },
    }

    # ---- Preview ----
    if SHOW_PLOT:
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        fuel_bbox = (fuel_x_min, fuel_x_max, fuel_y_min, fuel_y_max)
        merge_bbox = (merge_x_min, merge_x_max, merge_y_min, merge_y_max)
        gate_bbox = (gate_x_min_i, gate_x_max_i, gate_y_min_i, gate_y_max_i)

        print(f"{camera_name}: fuel_top_left_id={fuel_tl_id}, fuel_top_right_id={fuel_tr_id}, merge_tag_id={merge_id}")
        print(f"{camera_name}: fuel_long_len={fuel_long_len:.2f}px, fuel_short_len={fuel_short_len:.2f}px (K={short_k})")
        print(f"{camera_name}: fuel bbox   x[{fuel_x_min},{fuel_x_max}] y[{fuel_y_min},{fuel_y_max}]")
        print(f"{camera_name}: merge bbox  x[{merge_x_min},{merge_x_max}] y[{merge_y_min},{merge_y_max}]")
        print(f"{camera_name}: gate bbox   x[{gate_x_min_i},{gate_x_max_i}] y[{gate_y_min_i},{gate_y_max_i}] "
              f"(ratio={GATE_RIGHT_RATIO}, gate_short_k={GATE_SHORT_K})")
        print(f"{camera_name}: ignore_zones={len(ignore_zones)} rects (pad={COVER_PAD_PX}px)")
        print(f"{camera_name}: finish_line.x={finish_x}, direction=left_to_right")

        _plot_preview(rgb, camera_name, det_triplet, fuel_bbox, merge_bbox, gate_bbox, finish_x, ignore_zones)

    return zone_yaml


def write_yaml(path: str, data: dict):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        yaml.safe_dump(data, f, sort_keys=False)


if __name__ == "__main__":
    zones1 = build_zone_yaml_from_image(
        CAM1_IMAGE_PATH, CAM1_SHORT_K, CAM1_CAMERA_NAME, CAM1_IMAGE_TOPIC
    )
    write_yaml(CAM1_YAML_PATH, zones1)
    print(f"Wrote {CAM1_YAML_PATH}")

    zones2 = build_zone_yaml_from_image(
        CAM2_IMAGE_PATH, CAM2_SHORT_K, CAM2_CAMERA_NAME, CAM2_IMAGE_TOPIC
    )
    write_yaml(CAM2_YAML_PATH, zones2)
    print(f"Wrote {CAM2_YAML_PATH}")

#!/usr/bin/env python3
"""
Manual zone drag editor.

Run on the main PC desktop:
  cd ~/catkin_ws/src/vpa_duckierace
  python3 test/zone_drag_editor.py --camera 1
  python3 test/zone_drag_editor.py --camera 2
"""
import argparse
import copy
import os

import cv2
import yaml


COLORS = {
    "fuel_zone": (0, 165, 255),
    "charge_gate_zone": (255, 0, 255),
    "merge_zones": (0, 255, 255),
    "ignore_zones": (80, 80, 80),
}


def rect_from_zone(name, zone, index=None):
    return {
        "name": name,
        "index": index,
        "x_min": int(zone["x_min"]),
        "x_max": int(zone["x_max"]),
        "y_min": int(zone["y_min"]),
        "y_max": int(zone["y_max"]),
    }


def load_rects(config):
    rects = []
    if config.get("fuel_zone"):
        rects.append(rect_from_zone("fuel_zone", config["fuel_zone"]))
    if config.get("charge_gate_zone"):
        rects.append(rect_from_zone("charge_gate_zone", config["charge_gate_zone"]))
    for idx, zone in enumerate(config.get("merge_zones", [])):
        rects.append(rect_from_zone("merge_zones", zone, idx))
    for idx, zone in enumerate(config.get("ignore_zones", [])):
        rects.append(rect_from_zone("ignore_zones", zone, idx))
    return rects


def write_rects(config, rects):
    updated = copy.deepcopy(config)
    for rect in rects:
        target = {
            "x_min": int(min(rect["x_min"], rect["x_max"])),
            "x_max": int(max(rect["x_min"], rect["x_max"])),
            "y_min": int(min(rect["y_min"], rect["y_max"])),
            "y_max": int(max(rect["y_min"], rect["y_max"])),
        }
        name = rect["name"]
        if rect["index"] is None:
            updated[name] = target
        else:
            updated[name][rect["index"]] = target
    return updated


class ZoneEditor:
    def __init__(self, image_path, yaml_path):
        self.image_path = image_path
        self.yaml_path = yaml_path
        self.image = cv2.imread(image_path, cv2.IMREAD_COLOR)
        if self.image is None:
            raise FileNotFoundError(image_path)
        with open(yaml_path, "r") as f:
            self.config = yaml.safe_load(f)
        self.rects = load_rects(self.config)
        self.selected = 0 if self.rects else None
        self.drag_mode = None
        self.drag_start = None
        self.drag_rect = None
        self.window = f"Zone editor: {os.path.basename(image_path)}"
        self.handle_px = 10

    def current_rect(self):
        if self.selected is None:
            return None
        return self.rects[self.selected]

    def normalize_rect(self, r):
        if r["x_min"] > r["x_max"]:
            r["x_min"], r["x_max"] = r["x_max"], r["x_min"]
        if r["y_min"] > r["y_max"]:
            r["y_min"], r["y_max"] = r["y_max"], r["y_min"]

    def hit_mode(self, r, x, y):
        x0, x1 = sorted((r["x_min"], r["x_max"]))
        y0, y1 = sorted((r["y_min"], r["y_max"]))
        pad = self.handle_px
        near_left = abs(x - x0) <= pad
        near_right = abs(x - x1) <= pad
        near_top = abs(y - y0) <= pad
        near_bottom = abs(y - y1) <= pad
        inside_x = x0 - pad <= x <= x1 + pad
        inside_y = y0 - pad <= y <= y1 + pad

        if near_left and near_top:
            return "resize_tl"
        if near_right and near_top:
            return "resize_tr"
        if near_left and near_bottom:
            return "resize_bl"
        if near_right and near_bottom:
            return "resize_br"
        if near_left and inside_y:
            return "resize_l"
        if near_right and inside_y:
            return "resize_r"
        if near_top and inside_x:
            return "resize_t"
        if near_bottom and inside_x:
            return "resize_b"
        if x0 <= x <= x1 and y0 <= y <= y1:
            return "move"
        return None

    def hit_test(self, x, y):
        for idx in range(len(self.rects) - 1, -1, -1):
            r = self.rects[idx]
            mode = self.hit_mode(r, x, y)
            if mode:
                return idx, mode
        return None, None

    def apply_drag(self, x, y):
        dx = x - self.drag_start[0]
        dy = y - self.drag_start[1]
        r = self.current_rect()
        base = self.drag_rect

        if self.drag_mode == "move":
            r["x_min"] = base["x_min"] + dx
            r["x_max"] = base["x_max"] + dx
            r["y_min"] = base["y_min"] + dy
            r["y_max"] = base["y_max"] + dy
            return

        if "l" in self.drag_mode:
            r["x_min"] = base["x_min"] + dx
        if "r" in self.drag_mode:
            r["x_max"] = base["x_max"] + dx
        if "t" in self.drag_mode:
            r["y_min"] = base["y_min"] + dy
        if "b" in self.drag_mode:
            r["y_max"] = base["y_max"] + dy
        self.normalize_rect(r)

    def mouse(self, event, x, y, flags, _param):
        if event == cv2.EVENT_LBUTTONDOWN:
            hit, mode = self.hit_test(x, y)
            if hit is not None:
                self.selected = hit
                self.drag_mode = mode
                self.drag_start = (x, y)
                self.drag_rect = copy.deepcopy(self.current_rect())
        elif event == cv2.EVENT_MOUSEMOVE and self.drag_mode == "move":
            self.apply_drag(x, y)
        elif event == cv2.EVENT_MOUSEMOVE and self.drag_mode:
            self.apply_drag(x, y)
        elif event == cv2.EVENT_LBUTTONUP:
            self.drag_mode = None
            self.drag_start = None
            self.drag_rect = None

    def draw(self):
        canvas = self.image.copy()
        for idx, r in enumerate(self.rects):
            color = COLORS.get(r["name"], (255, 255, 255))
            thickness = 3 if idx == self.selected else 1
            x0, x1 = sorted((r["x_min"], r["x_max"]))
            y0, y1 = sorted((r["y_min"], r["y_max"]))
            cv2.rectangle(canvas, (x0, y0), (x1, y1), color, thickness)
            for px, py in ((x0, y0), (x1, y0), (x0, y1), (x1, y1)):
                cv2.rectangle(canvas, (px - 4, py - 4), (px + 4, py + 4), color, -1)
            label = r["name"] if r["index"] is None else f"{r['name']}[{r['index']}]"
            cv2.putText(
                canvas,
                label,
                (x0, max(18, y0 - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                color,
                2,
                cv2.LINE_AA,
            )
        help_text = "drag inside: move  drag edge/corner: resize  tab/n: next  s: save  q/esc: quit"
        cv2.putText(canvas, help_text, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        return canvas

    def save(self):
        updated = write_rects(self.config, self.rects)
        with open(self.yaml_path, "w") as f:
            yaml.safe_dump(updated, f, sort_keys=False)
        self.config = updated
        print(f"saved {self.yaml_path}")

    def run(self):
        cv2.namedWindow(self.window, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(self.window, self.mouse)
        while True:
            cv2.imshow(self.window, self.draw())
            key = cv2.waitKey(30) & 0xFF
            if key in (27, ord("q")):
                break
            if key in (ord("n"), 9) and self.rects:
                self.selected = (self.selected + 1) % len(self.rects)
            if key == ord("s"):
                self.save()
        cv2.destroyWindow(self.window)


def default_paths(camera):
    here = os.path.dirname(os.path.abspath(__file__))
    pkg = os.path.dirname(here)
    image = os.path.join(here, f"image{camera}.png")
    zones = os.path.join(pkg, "config", f"zones_cam{camera}.yaml")
    return image, zones


def main():
    parser = argparse.ArgumentParser(description="Drag DuckieRace zone rectangles and save YAML.")
    parser.add_argument("--camera", type=int, choices=(1, 2), default=1)
    parser.add_argument("--image")
    parser.add_argument("--zones")
    args = parser.parse_args()

    image, zones = default_paths(args.camera)
    editor = ZoneEditor(args.image or image, args.zones or zones)
    editor.run()


if __name__ == "__main__":
    main()

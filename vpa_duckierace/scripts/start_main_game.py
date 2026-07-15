#!/usr/bin/env python3
"""
Start the full DuckieRace game workflow in tmux.

Typical use:
  rosrun vpa_duckierace start_main_game.py

The script creates a tmux session, opens each module in its own tmux window,
and waits for Enter in the control window before opening the next one.
"""

import argparse
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import textwrap
import time
from pathlib import Path


PACKAGE_DIR = Path(__file__).resolve().parents[1]
WORKSPACE_DIR = PACKAGE_DIR.parents[1]
TEST_DIR = PACKAGE_DIR / "test"
DEFAULT_ROBOTS = (("daisy", "192.168.1.8"), ("lucas", "192.168.1.10"))
KNOWN_ROBOT_NAMES_BY_IP = {
    "192.168.1.8": "daisy",
    "192.168.1.10": "lucas",
    "192.168.1.13": "daisy",
}
KNOWN_ROBOT_TAGS = {
    "daisy": 5,
    "lucas": 4,
    "henry": 6,
    "vivian": 2,
    "gina": 3,
    "luna": 8,
    "dorie": 7,
}
DISPLAY_COLORS = ("blue", "dark green", "purple", "orange", "red", "cyan", "magenta")
STATE_DIR = Path(tempfile.gettempdir()) / "vpa_duckierace_main_game"


class TmuxLauncher:
    def __init__(self, args):
        self.args = args
        self.tmux = shutil.which("tmux")
        self.session = args.session
        self.socket_name = args.tmux_socket
        self.started_windows = []
        STATE_DIR.mkdir(parents=True, exist_ok=True)
        if not self.tmux:
            raise RuntimeError("tmux is not installed. Install tmux first, then run this script again.")

    @property
    def robots(self):
        return self.args.robots

    def clean_tmux_env(self):
        env = os.environ.copy()
        env.pop("TMUX", None)
        return env

    def tmux_cmd(self, *parts, check=True):
        return subprocess.run(
            [self.tmux, "-L", self.socket_name, *parts],
            env=self.clean_tmux_env(),
            stderr=None if check else subprocess.DEVNULL,
            text=True,
            check=check,
        )

    def session_exists(self):
        return self.tmux_cmd("has-session", "-t", self.session, check=False).returncode == 0

    def start_inside_tmux_if_needed(self):
        if os.environ.get("VPA_TMUX_CHILD") == "1":
            return False

        if self.args.shutdown:
            self.shutdown_session()
            return True

        if self.session_exists():
            if not self.args.replace_session:
                print(
                    f"tmux session '{self.session}' already exists.\n"
                    f"Attach with: tmux -L {self.socket_name} attach -t {self.session}\n"
                    "Or restart cleanly with: rosrun vpa_duckierace start_main_game.py --replace-session"
                )
                return True
            self.shutdown_session()

        child_args = [sys.executable, str(Path(__file__).resolve()), *sys.argv[1:]]
        child_args = [arg for arg in child_args if arg != "--replace-session"]
        command = " ".join(shell_quote(arg) for arg in child_args)
        command = f"cd {shell_quote(str(PACKAGE_DIR))}; VPA_TMUX_CHILD=1 exec {command}"
        self.tmux_cmd("new-session", "-d", "-s", self.session, "-n", "00_control", "bash")
        self.configure_session()
        self.tmux_cmd("send-keys", "-t", f"{self.session}:00_control", command, "C-m")
        time.sleep(0.3)
        self.tmux_cmd("attach-session", "-t", self.session)
        return True

    def configure_session(self):
        self.tmux_cmd("set-option", "-t", self.session, "-g", "mouse", "on")
        self.tmux_cmd("set-option", "-t", self.session, "-g", "status", "on")
        self.tmux_cmd("set-option", "-t", self.session, "-g", "status-interval", "1")
        self.tmux_cmd("set-option", "-t", self.session, "-g", "status-left-length", "30")
        self.tmux_cmd("set-option", "-t", self.session, "-g", "status-right-length", "80")
        self.tmux_cmd("set-window-option", "-t", self.session, "-g", "automatic-rename", "off")

    def shutdown_session(self):
        if self.session_exists():
            print(f"[launcher] shutting down tmux session '{self.session}'")
            self.tmux_cmd("kill-session", "-t", self.session, check=False)
        else:
            print(f"[launcher] tmux session '{self.session}' is not running")

    def ros_setup(self):
        return textwrap.dedent(
            f"""
            source /opt/ros/noetic/setup.bash 2>/dev/null || true
            source "{WORKSPACE_DIR}/devel/setup.bash" 2>/dev/null || true
            export ROS_MASTER_URI="${{ROS_MASTER_URI:-http://127.0.0.1:11311}}"
            """
        ).strip()

    def write_script(self, name, body):
        path = STATE_DIR / f"{self.session}_{name}.sh"
        path.write_text(
            textwrap.dedent(
                f"""\
                #!/usr/bin/env bash
                set -o pipefail
                trap 'exit 130' INT TERM
                {body.strip()}
                """
            ),
            encoding="utf-8",
        )
        path.chmod(0o755)
        return path

    def wait_for_enter(self, label):
        if self.args.no_step:
            print(f"[launcher] starting {label}")
            return
        input(f"\nPress Enter to open {label}...")

    def open_window(self, name, label, body):
        self.wait_for_enter(label)
        script_path = self.write_script(name, body)
        self.tmux_cmd("new-window", "-t", self.session, "-n", name, f"bash {shell_quote(str(script_path))}")
        self.started_windows.append(name)
        print(f"[launcher] opened tmux window: {name} - {label}")

    def open_managed_roslaunch(self, name, label, command, health_check=None):
        health_block = ""
        if health_check:
            health_block = textwrap.dedent(
                f"""
                if ! timeout 8 bash -lc {shell_quote(health_check)}; then
                  echo "[watchdog] health check failed for {label}; stopping command"
                  kill "$child" 2>/dev/null || true
                  wait "$child" 2>/dev/null || true
                  child=""
                fi
                """
            )

        body = f"""
        {self.ros_setup()}
        cd "{PACKAGE_DIR}"
        while true; do
          echo
          echo "[launcher] starting: {command}"
          bash -lc {shell_quote(command)} &
          child=$!
          while kill -0 "$child" 2>/dev/null; do
            sleep 5
            {health_block}
          done
          wait "$child"
          rc=$?
          echo "[launcher] {label} exited with code $rc"
          echo "[launcher] restarting in 3 seconds; use window 99_shutdown or Ctrl+C in 00_control to stop all"
          sleep 3
        done
        """
        self.open_window(name, label, body)

    def run_blocking(self, label, command, cwd=PACKAGE_DIR, timeout=None, expect=None):
        print(f"[launcher] waiting: {label}")
        env = os.environ.copy()
        env.setdefault("ROS_MASTER_URI", "http://127.0.0.1:11311")
        proc = subprocess.Popen(
            ["bash", "-lc", self.ros_setup() + "\n" + command],
            cwd=str(cwd),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            preexec_fn=os.setsid,
            env=env,
        )
        try:
            deadline = None if timeout is None else time.time() + timeout
            output = []
            while True:
                line = proc.stdout.readline()
                if line:
                    output.append(line)
                    print(f"[{label}] {line}", end="", flush=True)
                    if expect and expect in line:
                        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                        proc.wait(timeout=5)
                        return True, "".join(output)

                if proc.poll() is not None:
                    rest = proc.stdout.read() or ""
                    if rest:
                        output.append(rest)
                        print(f"[{label}] {rest}", end="", flush=True)
                    if expect:
                        return expect in "".join(output), "".join(output)
                    return proc.returncode == 0, "".join(output)

                if deadline and time.time() > deadline:
                    os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                    try:
                        proc.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                    return False, "".join(output)

                time.sleep(0.05)
        finally:
            if proc.poll() is None:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                except ProcessLookupError:
                    pass

    def wait_for_roscore(self):
        for _ in range(self.args.ros_timeout):
            ok, _ = self.run_blocking("roscore-check", "rostopic list >/dev/null", timeout=3)
            if ok:
                return True
            time.sleep(1)
        return False

    def wait_for_camera_topics(self):
        for _ in range(self.args.camera_timeout):
            ok, out = self.run_blocking("camera-topic-check", "rostopic list | grep image || true", timeout=5)
            if ok and "/usb_cam_1/" in out and "/usb_cam_2/" in out and "image" in out:
                return True
            time.sleep(1)
        return False

    def capture_image_in_tmux(self, filename, image_topic):
        self.wait_for_enter(f"capture {filename} from {image_topic}")
        TEST_DIR.mkdir(parents=True, exist_ok=True)
        destination = TEST_DIR / filename
        destination.unlink(missing_ok=True)
        log = STATE_DIR / f"{self.session}_{filename}.log"
        log.unlink(missing_ok=True)
        done = STATE_DIR / f"{self.session}_{filename}.done"
        done.unlink(missing_ok=True)
        body = f"""
        {self.ros_setup()}
        cd "{PACKAGE_DIR}"
        rosrun image_view image_saver image:={image_topic} _filename_format:=test/{filename} 2>&1 | tee "{log}"
        touch "{done}"
        echo "[launcher] image_saver finished; this window will close in 3 seconds"
        sleep 3
        """
        script_path = self.write_script(f"capture_{filename}", body)
        window_name = f"cap_{filename}"
        self.tmux_cmd("new-window", "-t", self.session, "-n", window_name, f"bash {shell_quote(str(script_path))}")
        print(f"[launcher] opened tmux window: {window_name}")

        deadline = time.time() + self.args.image_timeout
        while time.time() < deadline:
            text = log.read_text(encoding="utf-8", errors="replace") if log.exists() else ""
            if f"Saved image test/{filename}" in text or f"Saved image {filename}" in text or destination.exists():
                self.tmux_cmd("kill-window", "-t", f"{self.session}:{window_name}", check=False)
                time.sleep(0.3)
                print(f"[launcher] saved {destination}")
                return
            if done.exists():
                break
            time.sleep(0.2)
        raise RuntimeError(f"Timed out before image_saver saved {filename}")

    def virtual_driver_command(self):
        robot_names = [robot["name"] for robot in self.robots]
        all_robots = "[" + ", ".join(robot_names) + "]"
        parts = []
        for robot_name in robot_names:
            mode = self.args.virtual_driver_mode
            node_name = f"vd_{sanitize_ros_name(robot_name)}"
            parts.append(
                "rosrun vpa_duckierace virtual_driver_node.py "
                f"__name:={shell_word(node_name)} "
                f"_robot_name:={shell_word(robot_name)} "
                f"_driving_mode:={shell_word(mode)} "
                f"_all_robots:={shell_quote(all_robots)} "
                f"_control_rate:={shell_word(str(self.args.virtual_driver_rate))} &"
            )
        parts.append("trap 'kill $(jobs -p) 2>/dev/null || true; wait' INT TERM EXIT")
        parts.append("wait")
        return "\n".join(parts)

    def write_runtime_configs(self):
        display_config = STATE_DIR / f"{self.session}_robot_display_config.yaml"
        tag_map = STATE_DIR / f"{self.session}_tag_robot_map.yaml"

        display_lines = ["robots:"]
        tag_lines = []
        missing_tags = []
        for idx, robot in enumerate(self.robots, start=1):
            name = robot["name"]
            tag_id = robot.get("tag")
            if tag_id is None:
                missing_tags.append(name)
            display_name = robot.get("display_name") or f"ROBOT{idx}"
            color = robot.get("color") or DISPLAY_COLORS[(idx - 1) % len(DISPLAY_COLORS)]
            display_lines.extend(
                [
                    f"  {name}:",
                    f"    display_name: {display_name}",
                    f"    color: {color}",
                ]
            )
            if tag_id is not None:
                tag_lines.append(f"{name}: {tag_id}")

        if missing_tags:
            missing = ", ".join(missing_tags)
            raise RuntimeError(
                f"Missing AprilTag id for robot(s): {missing}. "
                "Add --robot-tag name=id, for example --robot-tag luna=8."
            )

        display_config.write_text("\n".join(display_lines) + "\n", encoding="utf-8")
        tag_map.write_text("\n".join(tag_lines) + "\n", encoding="utf-8")
        print(f"[launcher] runtime GUI config: {display_config}")
        print(f"[launcher] runtime tag map: {tag_map}")
        return display_config, tag_map

    def create_shutdown_window(self):
        body = f"""
        echo "Press Enter here to shutdown all DuckieRace tmux windows."
        read -r _
        tmux -L {shell_quote(self.socket_name)} kill-session -t {shell_quote(self.session)}
        """
        self.open_window("99_shutdown", "shutdown control window", body)

    def start_sequence(self):
        display_config, tag_map = self.write_runtime_configs()
        self.create_shutdown_window()

        self.open_managed_roslaunch(
            "01_roscore",
            "roscore",
            "roscore",
            health_check="rostopic list >/dev/null",
        )
        if not self.wait_for_roscore():
            raise RuntimeError("roscore did not become healthy in time")

        self.open_managed_roslaunch(
            "02_camera",
            "racecontrol_camera.launch",
            "roslaunch vpa_duckierace racecontrol_camera.launch",
            health_check="rostopic list | grep -q '/usb_cam_1/' && rostopic list | grep -q '/usb_cam_2/'",
        )
        if not self.wait_for_camera_topics():
            raise RuntimeError("Camera topics for usb_cam_1 and usb_cam_2 did not appear in time")

        self.open_window(
            "03_rqt",
            "rqt_image_view",
            f"""
            {self.ros_setup()}
            cd "{PACKAGE_DIR}"
            exec rqt_image_view
            """,
        )

        self.capture_image_in_tmux("image1.png", "/usb_cam_1/image_raw")
        self.capture_image_in_tmux("image2.png", "/usb_cam_2/image_raw")

        self.open_window(
            "10_editor_cam1",
            "zone_drag_editor camera 1",
            f"""
            {self.ros_setup()}
            cd "{PACKAGE_DIR}"
            exec python3 test/zone_drag_editor.py --camera 1
            """,
        )

        self.open_window(
            "11_editor_cam2",
            "zone_drag_editor camera 2",
            f"""
            {self.ros_setup()}
            cd "{PACKAGE_DIR}"
            exec python3 test/zone_drag_editor.py --camera 2
            """,
        )

        self.open_managed_roslaunch(
            "12_racecontrol",
            "start_racecontrol.launch",
            f"roslaunch vpa_duckierace start_racecontrol.launch tag_robot_map:={shell_word(str(tag_map))}",
        )

        for idx, robot in enumerate(self.robots, start=1):
            robot_name = robot["name"]
            ip = robot["ip"]
            self.open_window(
                f"13_robot_{idx}",
                f"{robot_name} SSH {ip}",
                f"""
                cd "{PACKAGE_DIR}"
                echo "Connecting to {self.args.ssh_user}@{ip}"
                echo "Type the robot password manually, then press Enter."
                exec ssh {shell_word(self.args.ssh_user)}@{shell_word(ip)}
                """,
            )

        self.open_managed_roslaunch(
            "16_virtual_driver",
            "virtual_driver_node.py",
            self.virtual_driver_command(),
        )

        self.open_window(
            "17_race_gui",
            "race_gui.py",
            f"""
            {self.ros_setup()}
            cd "{PACKAGE_DIR}"
            exec rosrun vpa_duckierace race_gui.py _config_path:={shell_quote(str(display_config))}
            """,
        )

        self.tmux_cmd("select-window", "-t", f"{self.session}:00_control", check=False)
        print("\n[launcher] all requested windows have been opened.")
        print(f"[launcher] switch windows with Ctrl+b then window number/name, or run: tmux -L {self.socket_name} attach -t {self.session}")
        print("[launcher] use the 99_shutdown window, or press Ctrl+C here, to stop everything.")
        while True:
            time.sleep(1)


def shell_quote(value):
    return "'" + str(value).replace("'", "'\"'\"'") + "'"


def shell_word(value):
    value = str(value)
    if all(ch.isalnum() or ch in "_-./:=" for ch in value):
        return value
    return shell_quote(value)


def sanitize_ros_name(value):
    return "".join(ch if ch.isalnum() or ch == "_" else "_" for ch in value)


def parse_robot_spec(spec):
    if "=" not in spec:
        raise argparse.ArgumentTypeError("Robot must use name=ip format, for example daisy=192.168.1.8")
    name, ip = spec.split("=", 1)
    name = name.strip()
    ip = ip.strip()
    if not name or not ip:
        raise argparse.ArgumentTypeError("Robot must use name=ip format, for example daisy=192.168.1.8")
    return {"name": name, "ip": ip}


def parse_robot_tag_spec(spec):
    if "=" not in spec:
        raise argparse.ArgumentTypeError("Robot tag must use name=id format, for example luna=8")
    name, tag_id = spec.split("=", 1)
    name = name.strip()
    tag_id = tag_id.strip()
    if not name or not tag_id:
        raise argparse.ArgumentTypeError("Robot tag must use name=id format, for example luna=8")
    try:
        tag_id_int = int(tag_id)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("Robot tag id must be an integer") from exc
    return name, tag_id_int


def parse_args():
    parser = argparse.ArgumentParser(description="Start the DuckieRace main game workflow in tmux.")
    parser.add_argument("--session", default="vpa_game", help="tmux session name.")
    parser.add_argument("--tmux-socket", default="vpa_game", help="tmux socket name used with tmux -L.")
    parser.add_argument("--replace-session", action="store_true", help="Kill an existing session with this name first.")
    parser.add_argument("--shutdown", action="store_true", help="Kill the configured tmux session and exit.")
    parser.add_argument("--no-step", action="store_true", help="Do not wait for Enter before each tmux window.")
    parser.add_argument(
        "--robot",
        action="append",
        type=parse_robot_spec,
        default=[],
        help="Robot name and IP in name=ip format. Repeat for multiple robots.",
    )
    parser.add_argument(
        "--robot-ip",
        action="append",
        default=[],
        help="Backward-compatible robot IP only. Name is inferred from known IPs or robotN.",
    )
    parser.add_argument(
        "--robot-tag",
        action="append",
        type=parse_robot_tag_spec,
        default=[],
        help="AprilTag id in name=id format. Repeat for robots not in the built-in tag list.",
    )
    parser.add_argument("--ssh-user", default="vpaadmin", help="Robot SSH username.")
    parser.add_argument("--virtual-driver-mode", default="conservative", help="Initial virtual driver mode.")
    parser.add_argument("--virtual-driver-rate", type=float, default=2.0, help="Virtual driver control rate.")
    parser.add_argument("--no-default-robots", action="store_true")
    parser.add_argument("--ros-timeout", type=int, default=30)
    parser.add_argument("--camera-timeout", type=int, default=45)
    parser.add_argument("--image-timeout", type=int, default=20)
    args = parser.parse_args()
    tag_overrides = dict(args.robot_tag)
    robots = list(args.robot)
    for idx, ip in enumerate(args.robot_ip, start=1):
        robots.append({"name": KNOWN_ROBOT_NAMES_BY_IP.get(ip, f"robot{idx}"), "ip": ip})
    if not robots and not args.no_default_robots:
        robots = [{"name": name, "ip": ip} for name, ip in DEFAULT_ROBOTS]
    for robot in robots:
        name = robot["name"]
        robot["tag"] = tag_overrides.get(name, KNOWN_ROBOT_TAGS.get(name))
    args.robots = robots
    return args


def main():
    args = parse_args()
    launcher = TmuxLauncher(args)
    if launcher.start_inside_tmux_if_needed():
        return 0

    try:
        launcher.start_sequence()
    except KeyboardInterrupt:
        launcher.shutdown_session()
    except Exception as exc:
        print(f"[launcher] startup failed: {exc}", file=sys.stderr)
        print(f"[launcher] use 'tmux -L {args.tmux_socket} kill-session -t {args.session}' if you want to clean up.")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

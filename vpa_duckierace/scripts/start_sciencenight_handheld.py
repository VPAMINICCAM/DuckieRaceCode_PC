#!/usr/bin/python3
import argparse
import getpass
import os
import select
import signal
import socket
import subprocess
import sys
import threading
import time

import paramiko


ROBOTS = {
    "fiona": "192.168.1.13",
    "lucas": "192.168.1.10",
}


def pc_reachable_ip():
    out = subprocess.check_output(
        ["bash", "-lc", "ip -4 -o addr show scope global"], text=True
    )
    candidates = []
    for line in out.splitlines():
        parts = line.split()
        if "inet" in parts:
            ip = parts[parts.index("inet") + 1].split("/", 1)[0]
            candidates.append(ip)
    for ip in candidates:
        if ip.startswith("192.168.1."):
            return ip
    if candidates:
        return candidates[0]
    raise RuntimeError("No global IPv4 address found for PC2")


def ssh_port_open(ip):
    for attempt in range(5):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.settimeout(2.0)
            if sock.connect_ex((ip, 22)) == 0:
                return True
        if attempt < 4:
            time.sleep(0.5)
    return False


def start_local(label, command, processes):
    proc = subprocess.Popen(
        ["bash", "-lc", command],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
        preexec_fn=os.setsid,
    )
    processes.append((label, proc))

    def pump():
        for line in proc.stdout:
            print(f"[{label}] {line}", end="")

    threading.Thread(target=pump, daemon=True).start()
    return proc


def remote_robot(robot, ip, pc_ip, username, password, stop_event):
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    try:
        client.connect(
            hostname=ip,
            username=username,
            password=password,
            timeout=8,
            look_for_keys=True,
            allow_agent=True,
        )
        cmd = (
            "bash -lc '"
            "source /opt/ros/noetic/setup.bash; "
            "source ~/catkin_ws/devel/setup.bash; "
            f"export ROS_MASTER_URI=http://{pc_ip}:11311; "
            f"export ROS_IP={ip}; "
            "unset ROS_HOSTNAME; "
            f"export robot_name={robot}; "
            "exec roslaunch vpa_robot_operation duckierace_start.launch use_physical_joy:=true"
            "'"
        )
        transport = client.get_transport()
        channel = transport.open_session()
        channel.get_pty()
        channel.exec_command(cmd)
        print(f"[{robot}] started on {ip}, ROS master http://{pc_ip}:11311")
        while not stop_event.is_set():
            readable, _, _ = select.select([channel], [], [], 0.2)
            if channel in readable and channel.recv_ready():
                data = channel.recv(4096).decode(errors="replace")
                for line in data.splitlines():
                    print(f"[{robot}] {line}")
            if channel.exit_status_ready():
                print(f"[{robot}] exited with {channel.recv_exit_status()}")
                return
        try:
            channel.send("\x03")
            time.sleep(1.0)
            channel.close()
        finally:
            client.close()
    except Exception as exc:
        print(f"[{robot}] failed: {exc}", file=sys.stderr)
        try:
            client.close()
        except Exception:
            pass


def main():
    parser = argparse.ArgumentParser(
        description="Start Science Night GUI/zone manager and physical-joystick robot launches."
    )
    parser.add_argument(
        "--robots",
        nargs="*",
        default=list(ROBOTS),
        choices=list(ROBOTS),
        help="Robots to start; unreachable robots are skipped.",
    )
    parser.add_argument("--skip-zone-manager", action="store_true")
    parser.add_argument("--skip-gui", action="store_true")
    parser.add_argument("--no-robots", action="store_true")
    parser.add_argument("--ssh-user", default="vpaadmin")
    args, _ = parser.parse_known_args()

    pc_ip = pc_reachable_ip()
    ros_master = "http://127.0.0.1:11311"
    print(f"[pc2] reachable IP for robots: {pc_ip}")
    print("[pc2] assumes camera bridge is already publishing /usb_cam_1 and /usb_cam_2")
    print(f"[pc2] requested robots: {', '.join(args.robots)}")

    processes = []
    stop_event = threading.Event()
    threads = []

    def shutdown(signum=None, frame=None):
        stop_event.set()
        for label, proc in processes:
            if proc.poll() is None:
                print(f"[pc2] stopping {label}")
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                except ProcessLookupError:
                    pass
        time.sleep(1.0)
        for label, proc in processes:
            if proc.poll() is None:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                except ProcessLookupError:
                    pass

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    ros_prefix = (
        "source /opt/ros/noetic/setup.bash; "
        "source ~/catkin_ws/devel/setup.bash; "
        f"export ROS_MASTER_URI={ros_master}; "
        f"export ROS_IP={pc_ip}; "
        "unset ROS_HOSTNAME; "
    )
    if not args.skip_zone_manager:
        start_local(
            "zone",
            ros_prefix + "exec ~/catkin_ws/src/vpa_duckierace/scripts/start_zone_manager_for_bridge.sh",
            processes,
        )
    if not args.skip_gui:
        start_local(
            "gui",
            ros_prefix + "export DISPLAY=${DISPLAY:-:0}; exec rosrun vpa_duckierace race_gui.py",
            processes,
        )

    if not args.no_robots and args.robots:
        reachable = [(robot, ROBOTS[robot]) for robot in args.robots if ssh_port_open(ROBOTS[robot])]
        skipped = sorted(set(args.robots) - {robot for robot, _ in reachable})
        for robot in skipped:
            print(f"[{robot}] skipped: SSH not reachable at {ROBOTS[robot]}")
        password = os.environ.get("PI_PASSWORD")
        if reachable and password is None:
            password = getpass.getpass(
                f"Password for {args.ssh_user}@PI robots (or press Enter for SSH key only): "
            ) or None
        for robot, ip in reachable:
            t = threading.Thread(
                target=remote_robot,
                args=(robot, ip, pc_ip, args.ssh_user, password, stop_event),
                daemon=True,
            )
            t.start()
            threads.append(t)

    try:
        while not stop_event.is_set():
            time.sleep(0.5)
            if processes and all(proc.poll() is not None for _, proc in processes):
                print("[pc2] all local processes exited")
                stop_event.set()
    finally:
        shutdown()
        for t in threads:
            t.join(timeout=2.0)


if __name__ == "__main__":
    main()

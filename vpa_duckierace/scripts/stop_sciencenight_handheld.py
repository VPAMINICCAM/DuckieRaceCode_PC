#!/usr/bin/python3
import argparse
import os
import socket
import subprocess
import sys

ROBOTS = {
    "fiona": "192.168.1.13",
    "lucas": "192.168.1.10",
}

LOCAL_NODES = [
    "/race_gui_node",
    "/zone_state_aggregator",
    "/cam_zone_manager1",
    "/cam_zone_manager2",
]


def run_local(command):
    return subprocess.run(["bash", "-lc", command], text=True)


def ssh_port_open(ip):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.settimeout(1.5)
        return sock.connect_ex((ip, 22)) == 0


def stop_local():
    ros_prefix = (
        "source /opt/ros/noetic/setup.bash; "
        "source ~/catkin_ws/devel/setup.bash; "
        "export ROS_MASTER_URI=http://127.0.0.1:11311; "
    )
    for node in LOCAL_NODES:
        run_local(ros_prefix + f"rosnode kill {node} >/dev/null 2>&1 || true")

    run_local("pkill -f 'rosrun vpa_duckierace race_gui.py' >/dev/null 2>&1 || true")
    run_local("pkill -f 'roslaunch vpa_duckierace zone_manager.launch' >/dev/null 2>&1 || true")
    run_local("pkill -f 'start_sciencenight_handheld.py' >/dev/null 2>&1 || true")
    print("[pc2] requested stop for GUI, zone manager, and starter script")


def stop_robot(robot, ip, username, password):
    import paramiko

    if not ssh_port_open(ip):
        print(f"[{robot}] skipped: SSH not reachable at {ip}")
        return

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
        command = (
            "bash -lc \""
            "pkill -INT -f 'roslaunch vpa_robot_operation duckierace_start.launch' || true; "
            "sleep 1; "
            "pkill -TERM -f 'roslaunch vpa_robot_operation duckierace_start.launch' || true; "
            "pkill -TERM -f 'vpa_robot_operation/scripts/duckierace.py' || true; "
            "pkill -TERM -f 'vpa_robot_interface/scripts/wheel_driver.py' || true; "
            "pkill -TERM -f 'vpa_robot_interface/scripts/wheel_encoders.py' || true; "
            "pkill -TERM -f 'joy_node' || true; "
            "pkill -TERM -f 'usb_cam_node' || true"
            "\""
        )
        stdin, stdout, stderr = client.exec_command(command, timeout=10)
        code = stdout.channel.recv_exit_status()
        err = stderr.read().decode(errors="replace").strip()
        if err:
            print(f"[{robot}] stderr: {err}")
        print(f"[{robot}] stop requested at {ip} (exit {code})")
    except Exception as exc:
        print(f"[{robot}] failed: {exc}", file=sys.stderr)
    finally:
        client.close()


def main():
    parser = argparse.ArgumentParser(
        description="Stop Science Night GUI/zone manager and robot-side DuckieRace launches."
    )
    parser.add_argument(
        "--robots",
        nargs="*",
        default=list(ROBOTS),
        choices=list(ROBOTS),
        help="Robots to stop.",
    )
    parser.add_argument("--no-local", action="store_true", help="Do not stop PC2 GUI/zone nodes.")
    parser.add_argument("--no-robots", action="store_true", help="Do not stop robot-side launches.")
    parser.add_argument("--ssh-user", default="vpaadmin")
    args = parser.parse_args()

    print(f"[pc2] requested stop robots: {', '.join(args.robots)}")

    if not args.no_local:
        stop_local()

    if not args.no_robots and args.robots:
        password = os.environ.get("PI_PASSWORD")
        for robot in args.robots:
            stop_robot(robot, ROBOTS[robot], args.ssh_user, password)


if __name__ == "__main__":
    main()

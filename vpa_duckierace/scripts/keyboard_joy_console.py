#!/usr/bin/env python3
import argparse
import select
import sys
import termios
import time
import tty

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool


BTN_X = 0
BTN_B = 2
BTN_Y = 3
BTN_L1 = 4
BTN_R1 = 5

KEY_TO_BUTTON = {
    "b": BTN_B,
    "x": BTN_X,
    "y": BTN_Y,
    "l": BTN_L1,
    "r": BTN_R1,
}


class KeyboardJoyConsole:
    def __init__(self, robots, hold_time, rate_hz):
        rospy.init_node("keyboard_joy_console", anonymous=True)
        self.robots = robots
        self.robot_idx = 0
        self.hold_time = hold_time
        self.rate = rospy.Rate(rate_hz)
        self.button_until = {}
        self.joy_pubs = {}
        self.local_brake_pubs = {}
        self.global_brake_pub = rospy.Publisher("/global_brake", Bool, queue_size=1, latch=True)

    @property
    def robot(self):
        return self.robots[self.robot_idx]

    def joy_pub(self, robot):
        if robot not in self.joy_pubs:
            self.joy_pubs[robot] = rospy.Publisher(f"/{robot}/joy", Joy, queue_size=1)
        return self.joy_pubs[robot]

    def local_brake_pub(self, robot):
        if robot not in self.local_brake_pubs:
            self.local_brake_pubs[robot] = rospy.Publisher(
                f"/{robot}/local_brake", Bool, queue_size=1, latch=True
            )
        return self.local_brake_pubs[robot]

    def make_joy(self):
        now = time.time()
        buttons = [0] * 12
        for idx, until in list(self.button_until.items()):
            if until > now:
                buttons[idx] = 1
            else:
                self.button_until.pop(idx, None)
        msg = Joy()
        msg.header.stamp = rospy.Time.now()
        msg.axes = [0.0] * 8
        msg.buttons = buttons
        return msg

    def pulse_button(self, button_idx):
        self.button_until[button_idx] = time.time() + self.hold_time

    def clear_buttons(self):
        self.button_until.clear()
        self.joy_pub(self.robot).publish(self.make_zero_joy())

    @staticmethod
    def make_zero_joy():
        msg = Joy()
        msg.header.stamp = rospy.Time.now()
        msg.axes = [0.0] * 8
        msg.buttons = [0] * 12
        return msg

    def switch_robot(self, idx):
        if idx == self.robot_idx or not 0 <= idx < len(self.robots):
            return
        old_robot = self.robot
        self.joy_pub(old_robot).publish(self.make_zero_joy())
        self.button_until.clear()
        self.robot_idx = idx
        self.joy_pub(self.robot).publish(self.make_zero_joy())
        self.print_status(prefix=f"switched {old_robot} -> {self.robot}")

    def emergency_stop(self):
        self.clear_buttons()
        self.global_brake_pub.publish(Bool(data=True))
        self.local_brake_pub(self.robot).publish(Bool(data=True))
        self.print_status(prefix=f"STOP {self.robot}")

    def release_brakes(self):
        self.global_brake_pub.publish(Bool(data=False))
        self.local_brake_pub(self.robot).publish(Bool(data=False))
        self.print_status(prefix=f"release brakes {self.robot}")

    def print_help(self):
        robot_list = " ".join(f"{i + 1}:{name}" for i, name in enumerate(self.robots))
        print(
            "\nKeyboard Joy Console\n"
            f"robots: {robot_list}\n"
            "keys: b=B/yellow  x=X/brake-toggle  y=Y/charge  l=L1/faster  r=R1/slower\n"
            "      1-9 select robot  Tab cycle  Space release buttons\n"
            "      s emergency stop  u release global+local brakes  h help  q quit\n",
            flush=True,
        )

    def print_status(self, prefix=""):
        active = []
        now = time.time()
        for key, idx in KEY_TO_BUTTON.items():
            if self.button_until.get(idx, 0.0) > now:
                active.append(key.upper())
        print(
            f"\r{prefix:<28} robot={self.robot:<8} active={','.join(active) or '-':<12}",
            end="",
            flush=True,
        )

    def handle_key(self, ch):
        if ch in KEY_TO_BUTTON:
            self.pulse_button(KEY_TO_BUTTON[ch])
        elif ch == " ":
            self.clear_buttons()
        elif ch == "\t":
            self.switch_robot((self.robot_idx + 1) % len(self.robots))
        elif ch.isdigit() and ch != "0":
            self.switch_robot(int(ch) - 1)
        elif ch == "s":
            self.emergency_stop()
        elif ch == "u":
            self.release_brakes()
        elif ch == "h":
            self.print_help()
        elif ch == "q":
            raise KeyboardInterrupt

    def run(self):
        self.print_help()
        settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            while not rospy.is_shutdown():
                readable, _, _ = select.select([sys.stdin], [], [], 0.0)
                while readable:
                    ch = sys.stdin.read(1)
                    self.handle_key(ch)
                    readable, _, _ = select.select([sys.stdin], [], [], 0.0)

                self.joy_pub(self.robot).publish(self.make_joy())
                self.print_status()
                self.rate.sleep()
        except KeyboardInterrupt:
            pass
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            self.clear_buttons()
            print("\nexited, buttons released", flush=True)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--robots",
        default="daisy,lucas",
        help="Comma-separated robot namespaces. Number keys select this order.",
    )
    parser.add_argument("--hold-time", type=float, default=0.35)
    parser.add_argument("--rate", type=float, default=20.0)
    return parser.parse_args(rospy.myargv(argv=sys.argv)[1:])


if __name__ == "__main__":
    args = parse_args()
    robots = [name.strip() for name in args.robots.split(",") if name.strip()]
    if not robots:
        raise SystemExit("at least one robot is required")
    KeyboardJoyConsole(robots, args.hold_time, args.rate).run()

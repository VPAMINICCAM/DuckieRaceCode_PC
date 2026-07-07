#!/usr/bin/env python3
import rospy
import yaml
import time
from std_msgs.msg import Float32, Bool
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button
import os

class RaceGUI:
    def __init__(self):
        rospy.init_node("race_gui_node")
        self.last_detect = False
        # Load robot display config
        config_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'config', 'robot_display_config.yaml')
        rospy.loginfo(f"Loading config from: {config_path}")
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        self.robots = config['robots']
        self.display_names = {k: v['display_name'] for k, v in self.robots.items()}
        self.charging_cost_per_percent = rospy.get_param("~charging_cost_per_percent", 2.0)

        self.robot_states = {}
        for name in self.robots:
            self.robot_states[name] = {
                'power': 75.0,
                'last_power': None,
                'speed': 0.0,
                'brake': True,
                'in_fuel': False,
                'in_charge_gate': False,
                'charge_wait_time': 0.0,
                'charge_wait_last': None,
                'total_charged_energy': 0.0,
                'charging_events': 0,
                'charging_active': False,
                'laps': 0.0
            }
            rospy.Subscriber(f"/{name}/power_level", Float32, self.make_callback(name, 'power'))
            rospy.Subscriber(f"/{name}/speed_percent", Float32, self.make_callback(name, 'speed'))
            rospy.Subscriber(f"/{name}/local_brake", Bool, self.make_callback(name, 'brake'))
            rospy.Subscriber(f"/{name}/in_fuel_zone", Bool, self.make_callback(name, 'in_fuel'))
            rospy.Subscriber(f"/{name}/in_charge_gate_zone", Bool, self.make_callback(name, 'in_charge_gate'))
            rospy.Subscriber(f"/{name}/lap_count", Float32, self.make_callback(name, 'laps'))

        self.global_brake_pub = rospy.Publisher("/global_brake", Bool, queue_size=1, latch=True)
        self.global_brake_pub.publish(Bool(data=True))
        self.reset_laps_pub = rospy.Publisher("/reset_laps", Bool, queue_size=1)
        self.reset_power_speed_pubs = {
            name: rospy.Publisher(f"/{name}/reset_power_speed", Bool, queue_size=1)
            for name in self.robots
        }

        # GUI setup
        self.state = 'waiting'  # 'waiting', 'countdown', 'running', 'finished'
        self.countdown = 5
        self.race_limit = 300
        self.countdown_start = None
        self.start_time = None

        self.fig, self.ax = plt.subplots(figsize=(10, 6))
        plt.subplots_adjust(top=0.8, right=0.75)
        self.anim = FuncAnimation(self.fig, self.update_gui, interval=200)

        # Start/Stop toggle button
        self.toggle_ax = plt.axes([0.8, 0.85, 0.15, 0.08])
        self.toggle_button = Button(self.toggle_ax, 'Start Game')
        self.toggle_button.on_clicked(self.toggle_game)

        plt.show()

    def make_callback(self, robot_name, field):
        def callback(msg):
            if field == 'power':
                self.update_power_stats(robot_name, msg.data)
            elif field in ['speed', 'laps']:
                self.robot_states[robot_name][field] = msg.data
            else:
                self.robot_states[robot_name][field] = msg.data
                if field == 'in_fuel' and not msg.data:
                    self.robot_states[robot_name]['charging_active'] = False
        return callback

    def update_power_stats(self, robot_name, power):
        state = self.robot_states[robot_name]
        last_power = state['last_power']
        state['power'] = power
        if last_power is None:
            state['last_power'] = power
            return

        delta = power - last_power
        if self.state == 'running' and state['in_fuel'] and delta > 0.01:
            state['total_charged_energy'] += delta
            if not state['charging_active']:
                state['charging_events'] += 1
                state['charging_active'] = True
        elif not state['in_fuel']:
            state['charging_active'] = False
        state['last_power'] = power

    def toggle_game(self, event):
        if self.state in ['waiting', 'running', 'finished']:
            if self.state == 'waiting':
                # Start game countdown
                self.reset_laps()
                self.reset_charge_wait_times()
                self.reset_charging_stats()
                self.countdown_start = time.time()
                self.state = 'countdown'
                self.toggle_button.label.set_text("Stop Game")
            elif self.state == 'running':
                # Stop game
                self.state = 'waiting'
                self.global_brake_pub.publish(Bool(data=True))
                self.reset_laps()
                self.reset_power_speed()
                self.toggle_button.label.set_text("Start Game")
            elif self.state == 'finished':
                self.reset_laps()
                self.reset_charge_wait_times()
                self.reset_charging_stats()
                self.countdown_start = time.time()
                self.state = 'countdown'
                self.toggle_button.label.set_text("Stop Game")

    def reset_laps(self):
        for state in self.robot_states.values():
            state['laps'] = 0.0
        for _ in range(3):
            self.reset_laps_pub.publish(Bool(data=True))

    def reset_power_speed(self):
        for name, state in self.robot_states.items():
            state['power'] = 75.0
            state['speed'] = 0.25
            for _ in range(3):
                self.reset_power_speed_pubs[name].publish(Bool(data=True))

    def reset_charge_wait_times(self):
        for state in self.robot_states.values():
            state['charge_wait_time'] = 0.0
            state['charge_wait_last'] = None

    def reset_charging_stats(self):
        for state in self.robot_states.values():
            state['total_charged_energy'] = 0.0
            state['charging_events'] = 0
            state['charging_active'] = False
            state['last_power'] = state['power']

    def finish_game(self):
        self.state = 'finished'
        for _ in range(5):
            self.global_brake_pub.publish(Bool(data=True))
        self.toggle_button.label.set_text("Start Game")

    def update_gui(self, frame):
        self.ax.clear()
        self.ax.axis('off')

        # Title/Header
        if self.state == 'waiting':
            title = "Waiting to Start..."
        elif self.state == 'countdown':
            seconds_left = int(self.countdown - (time.time() - self.countdown_start))
            if seconds_left <= 0:
                self.start_time = time.time()
                self.state = 'running'
                self.global_brake_pub.publish(Bool(data=False))
            title = f"Race Starting In: {max(seconds_left, 0)}s"
        elif self.state == 'running':
            elapsed = time.time() - self.start_time
            if elapsed >= self.race_limit:
                self.finish_game()
                title = f"Race Finished: {self.race_limit}s"
            else:
                title = f"Race Running: {int(elapsed)}s / {self.race_limit}s"
        elif self.state == 'finished':
            title = f"Race Finished: {self.race_limit}s"
        self.ax.set_title(title, fontsize=20)

        # Robot display blocks
        spacing = 1.0 / (len(self.robots) + 1)
        for idx, (robot_name, state) in enumerate(self.robot_states.items()):
            y = 0.7 - idx * spacing

            display_name = self.display_names[robot_name]
            self.ax.text(0.05, y, f"{display_name} ({robot_name})", fontsize=14)

            # Power bar
            bar_x1 = 0.38
            self.ax.add_patch(patches.Rectangle((bar_x1, y - 0.025), 0.2, 0.05, fill=False))
            self.ax.add_patch(patches.Rectangle(
                (bar_x1, y - 0.025), 0.2 * min(state['power'], 100.0) / 100.0, 0.05, color='orange'))
            self.ax.text(bar_x1 + 0.1, y + 0.035, f"Power: {int(state['power'])}%", fontsize=10, ha='center')

            # Speed bar (scaled between duckierace.py min/max speed)
            bar_x2 = 0.65
            self.ax.add_patch(patches.Rectangle((bar_x2, y - 0.025), 0.2, 0.05, fill=False))
            spd = (state['speed'] - 0.15)/0.20

            spd_per_to_display = max(0, min(100, 100 * spd))
            self.ax.add_patch(patches.Rectangle(
                (bar_x2, y - 0.025), 0.2 * spd_per_to_display / 100.0, 0.05, color='blue'))
            self.ax.text(bar_x2 + 0.1, y + 0.035, f"Speed: {int(spd_per_to_display)}%", fontsize=10, ha='center')

            # Brake indicator
            brake_color = 'red' if state['brake'] else 'green'
            self.ax.add_patch(patches.Circle((0.87, y), 0.015, color=brake_color))
            self.ax.text(0.89, y, "Brake", verticalalignment='center')

            # Fuel zone indicator
            if state['in_fuel']:
                self.ax.text(0.02, y-0.05, "OK to Fuel", fontsize=12, verticalalignment='center', color='green')
            else:
                self.ax.text(0.02, y-0.05, "NG to Fuel", fontsize=12, verticalalignment='center', color='red')

            now = time.time()
            if self.state == 'running' and state['in_charge_gate'] and state['brake']:
                if state['charge_wait_last'] is not None:
                    state['charge_wait_time'] += now - state['charge_wait_last']
                state['charge_wait_last'] = now
            else:
                state['charge_wait_last'] = None
            self.ax.text(0.38, y-0.05, f"Gate Wait: {state['charge_wait_time']:.1f}s", fontsize=12, verticalalignment='center')
            charging_cost = state['total_charged_energy'] * self.charging_cost_per_percent
            self.ax.text(
                0.38, y-0.09,
                f"Charged: {state['total_charged_energy']:.1f}%  Cost: EUR {charging_cost:.2f}  Events: {state['charging_events']}",
                fontsize=10,
                verticalalignment='center')

            # Lap counter
            self.ax.text(1.08, y, f"Laps: {state['laps']:.0f}", fontsize=14, verticalalignment='center')


if __name__ == '__main__':
    RaceGUI()

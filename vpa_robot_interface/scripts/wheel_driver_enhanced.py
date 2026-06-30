#!/usr/bin/python3

import rospy
import socket

from wheel_driver import WheelDriver
from pid_controller.feedfoward_PID_Absolute import PIDController_Enhanced as PIDController
from vpa_robot_interface.cfg import omegaConfig
from vpa_robot_interface.msg import WheelsCmd,WheelsEncoder

from dynamic_reconfigure.server import Server
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

class pidConfig:
    def __init__(self,kp=0.1, ki=0.0, kd=0.0, kff=0.0, bff=0.0, u_min=0, u_max=1,tau_aw=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.kff = kff
        self.bff = bff
        self.u_min = u_min
        self.u_max = u_max 
        self.tau_aw = tau_aw

class WheelDriverEnhanced(WheelDriver):
    def __init__(self):

        self.robot_name = socket.gethostname()

        rospy.on_shutdown(self.shut_hook)
        self.driver = WheelDriver()

        self.estop          = True
        self.local_estop    = True

        self.direct_mode    = rospy.get_param('~direct_mode',False)

            
        self.left_conrtoller_default = pidConfig()
        self.right_controller_default = pidConfig()

        self.left_omega_controller = PIDController(
            kp=self.left_conrtoller_default.kp,
            ki=self.left_conrtoller_default.ki,
            kd=self.left_conrtoller_default.kd,
            kff=self.left_conrtoller_default.kff,
            bff=self.left_conrtoller_default.bff,
            u_min=self.left_conrtoller_default.u_min,
            u_max=self.left_conrtoller_default.u_max
        )
        self.right_omega_controller = PIDController(
            kp=self.right_controller_default.kp,
            ki=self.right_controller_default.ki,
            kd=self.right_controller_default.kd,
            kff=self.right_controller_default.kff,
            bff=self.right_controller_default.bff,
            u_min=self.right_controller_default.u_min,
            u_max=self.right_controller_default.u_max
        )

        self._baseline  = 0.1       # gap between wheels m
        self._radius    = 0.0318    # radius of wheels

        self.left_omega     = 0
        self.right_omega    = 0

        if self.direct_mode:
            rospy.logwarn("%s: Not supported in this version")
            rospy.signal_shutdown("Direct mode is not supported in this version.")
            return
        else:
            self.sub_wheel_enc = rospy.Subscriber("wheel_omega",WheelsEncoder,self.wheel_omega_cb,queue_size=1)
            self.sub_car_cmd   = rospy.Subscriber("cmd_vel", Twist, self.car_cmd_cb)

        self.sub_e_stop         = rospy.Subscriber("/global_brake", Bool, self.estop_cb, queue_size=1)
        self.sub_local_e_stop   = rospy.Subscriber("local_brake", Bool, self.estop_local_cb, queue_size=1)
        self.debug_pub = rospy.Publisher("wheel_info", WheelsCmd, queue_size=1)

        self.srv_left = Server(omegaConfig, self.dynamic_reconfigure_callback_left, namespace='left_wheel')
        self.srv_right = Server(omegaConfig, self.dynamic_reconfigure_callback_right, namespace='right_wheel')

    def estop_cb(self, msg:Bool):
        if msg.data != self.estop:
            self.estop = msg.data
            rospy.loginfo("%s: Global brake state changed to %s", self.robot_name, self.estop)
    
    def car_cmd_cb(self, msg:Twist):
        if self.estop or self.local_estop:
            self.driver.set_wheels_throttle(left=0, right=0)
            self.left_omega_controller.reset()
            self.right_omega_controller.reset()
            return
        if msg.linear.x == 0:
            # If no linear velocity, set both wheels to zero throttle
            self.driver.set_wheels_throttle(left=0, right=0)
            self.left_omega_controller.reset()
            self.right_omega_controller.reset()
            return
        # Convert linear.x (m/s) and angular.z (rad/s) to wheel angular velocity (rad/s)
        # v = r * omega  =>  omega = v / r
        left_omega = (msg.linear.x - (self._baseline / 2.0) * msg.angular.z) / self._radius
        right_omega = (msg.linear.x + (self._baseline / 2.0) * msg.angular.z) / self._radius

        left_throttle = self.left_omega_controller.update(ref=left_omega, meas=self.left_omega,dt=0.1, current_time=None)
        print(f"Left omega (rad/s): {left_omega}, Measured (rad/s): {self.left_omega}, Throttle: {left_throttle}")
        right_throttle = self.right_omega_controller.update(ref=right_omega, meas=self.right_omega, dt=0.1, current_time=None)
        # Simplify the time gap for i and d terms, assuming a constant loop rate of 10Hz
        
        self.driver.set_wheels_throttle(left=left_throttle, right=right_throttle)
        # Publish debug message
        debug_msg = WheelsCmd()
        debug_msg.vel_left      = left_omega
        debug_msg.vel_right     = right_omega
        debug_msg.throttle_left = left_throttle
        debug_msg.throttle_right = right_throttle
        self.debug_pub.publish(debug_msg)

    def estop_local_cb(self, msg:Bool):
        if msg.data != self.local_estop:
            self.local_estop = msg.data
            rospy.loginfo("%s: Local brake state changed to %s", self.robot_name, self.local_estop)

    def wheel_omega_cb(self, msg:WheelsEncoder):
        self.left_omega     = msg.omega_left
        self.right_omega    = msg.omega_right

    def dynamic_reconfigure_callback_left(self, config, level):
        self.left_omega_controller.kp = config['kp']
        self.left_omega_controller.ki = config['ki']
        self.left_omega_controller.kd = config['kd']
        self.left_omega_controller.kff = config['kff']
        self.left_omega_controller.bff = config['bff']
        self.left_omega_controller.u_min = config['u_min']
        self.left_omega_controller.u_max = config['u_max']
        self.left_omega_controller.tau_aw = config['tau_aw']
        # Organize and pretty-print the config dictionary for better readability

        config_to_log = {k: v for k, v in config.items() if k != 'groups'}
        config_str = "\n".join([f"    {k}: {v}" for k, v in config_to_log.items()])
        rospy.loginfo("%s: Left wheel PID configuration updated:\n%s", self.robot_name, config_str)
        return config

    def dynamic_reconfigure_callback_right(self, config, level):
        self.right_omega_controller.kp = config['kp']
        self.right_omega_controller.ki = config['ki']
        self.right_omega_controller.kd = config['kd']
        self.right_omega_controller.kff = config['kff']
        self.right_omega_controller.bff = config['bff']
        self.right_omega_controller.u_min = config['u_min']
        self.right_omega_controller.u_max = config['u_max']
        self.right_omega_controller.tau_aw = config['tau_aw']
        config_to_log = {k: v for k, v in config.items() if k != 'groups'}
        config_str = "\n".join([f"    {k}: {v}" for k, v in config_to_log.items()])
        rospy.loginfo("%s: Right wheel PID configuration updated:\n%s", self.robot_name, config_str)
        return config    

    def shut_hook(self):
        rospy.loginfo("%s: Shutting down WheelDrivers", self.robot_name)
        self.estop = True
        self.driver.set_wheels_throttle(left=0,right=0)
        self.driver = None
    
if __name__ == '__main__':
    rospy.init_node('wheel_driver_enhanced', anonymous=False)
    driver = WheelDriverEnhanced()
    rospy.loginfo("%s: WheelDriverEnhanced initialized", driver.robot_name)
    rospy.spin()
#!/usr/bin/python3

import rospy
import socket
from math import fabs, floor
import os
from dt_config.dt_hardware_settings import MotorDirection, HATv2
import json

from vpa_robot_interface.msg import WheelsCmd,WheelsEncoder
from vpa_robot_interface.cfg import omegaConfig, yawConfig

from pid_controller.pi_format import PI_controller
from pid_controller.feedforward_pi_format import FeedforwardPIController

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from sensor_msgs.msg import Imu

from dynamic_reconfigure.server import Server

class WheelDriver:

    LEFT_MOTOR_MIN_PWM  = 60        #: Minimum speed for left motor
    LEFT_MOTOR_MAX_PWM  = 255       #: Maximum speed for left motor
    RIGHT_MOTOR_MIN_PWM = 60        #: Minimum speed for right motor
    RIGHT_MOTOR_MAX_PWM = 255       #: Maximum speed for right motor
    SPEED_TOLERANCE     = 1.0e-2    #: Speed tolerance level

    def __init__(self) -> None:
        self.hat        = HATv2()
        self.leftMotor  = self.hat.get_motor(1, "left")
        self.rightMotor = self.hat.get_motor(2, "right")

        self.leftThrottle   = 0.0
        self.rightThrottle  = 0.0
        self._pwm_update()

    def set_wheels_throttle(self, left: float, right: float):
        """Sets speed of motors.

        Args:
           left (:obj:`float`): speed for the left wheel, should be between -1 and 1
           right (:obj:`float`): speed for the right wheel, should be between -1 and 1
           is_test_cmd (:obj:`bool`): whether this is a command issue by the hardware test

        """
        self.leftThrottle  = left
        self.rightThrottle = right
        self._pwm_update()

    def _pwm_value(self, v, min_pwm, max_pwm):
        """Transforms the requested speed into an int8 number.

        Args:
            v (:obj:`float`): requested speed, should be between -1 and 1.
            min_pwm (:obj:`int8`): minimum speed as int8
            max_pwm (:obj:`int8`): maximum speed as int8
        """
        pwm = 0
        if fabs(v) > self.SPEED_TOLERANCE:
            pwm = int(floor(fabs(v) * (max_pwm - min_pwm) + min_pwm))
        return min(pwm, max_pwm)

    def _pwm_update(self):
        """Sends commands to the microcontroller.

        Updates the current PWM signals (left and right) according to the
        linear velocities of the motors. The requested speed gets
        tresholded.
        """
        vl = self.leftThrottle
        vr = self.rightThrottle

        pwml = self._pwm_value(vl, self.LEFT_MOTOR_MIN_PWM, self.LEFT_MOTOR_MAX_PWM)
        pwmr = self._pwm_value(vr, self.RIGHT_MOTOR_MIN_PWM, self.RIGHT_MOTOR_MAX_PWM)
        leftMotorMode   = 0
        rightMotorMode  = 0

        if fabs(vl) < self.SPEED_TOLERANCE:
            pwml = 0
        elif vl > 0:
            leftMotorMode = MotorDirection.FORWARD
        elif vl < 0:
            leftMotorMode = MotorDirection.BACKWARD

        if fabs(vr) < self.SPEED_TOLERANCE:
            pwmr = 0
        elif vr > 0:
            rightMotorMode = MotorDirection.FORWARD
        elif vr < 0:
            rightMotorMode = MotorDirection.BACKWARD

        self.leftMotor.set(leftMotorMode, pwml)
        self.rightMotor.set(rightMotorMode, pwmr)

    def __del__(self):
        """Destructor method.

        Releases the motors and deletes tho object.
        """
        self.leftMotor.set(MotorDirection.RELEASE)
        self.rightMotor.set(MotorDirection.RELEASE)
        del self.hat
    
class WheelDriverNode:
    DEFAULT_KP = 0.04
    DEFAULT_KI = 0.03
    DEFAULT_KFF = 0.038
    DEFAULT_BFF = 0

    def __init__(self) -> None:

        rospy.on_shutdown(self.shut_hook)
        
        # Get the vehicle name
        # self.veh_name         = rospy.get_namespace().strip("/")
        # if len(self.veh_name) == 0:
        #     self.veh_name = 'db19'
        self.veh_name       = socket.gethostname()
            
        self.direct_mode    = rospy.get_param('~direct_mode',False)

        self.driver = WheelDriver()
        script_dir = os.path.dirname(os.path.abspath(__file__))
        filepath = os.path.join(script_dir,'adafruit_drivers/kinematics.py')
        self.log_dir = os.path.join(script_dir, 'logs')

        self.kp, self.ki, self.kff, self.bff = self._read_settings('left_wheel')
        self.omega_controller_left = FeedforwardPIController(kp=self.kp, ki=self.ki, kff=self.kff, bff=self.bff, integral_limit=30, output_limit=1.0)
        
        self.kp, self.ki, self.kff, self.bff = self._read_settings('right_wheel')
        self.omega_controller_right = FeedforwardPIController(kp=self.kp, ki=self.ki, kff=self.kff, bff=self.bff, integral_limit=30, output_limit=1.0)

        self.omega_left_ref     = 0
        self.omega_right_ref    = 0

        self.omega_left_sig     = 0
        self.omega_right_sig    = 0

        self.throttle_left      = 0
        self.throttle_right     = 0

        # Kinematics

        self._v_max     = 1         # max longitudinal speed m/s
        self._omega_max = 8         # max yaw rate rad/s
        self._baseline  = 0.1       # gap between wheels m
        self._radius    = 0.0318    # radius of wheels

        # Global brake
        self.if_dyna_trim = rospy.get_param('~dyna_trim', False)
        if self.if_dyna_trim:
            self.yaw_pid = PI_controller(kp=0.5, ki=0.05)
            self.yaw_trim = 0.0
            self.yaw_setpoint = 0.0
            self.yaw = 0.0
        else:
            self.yaw_trim = 0.0
        
        self.estop         = True
        rospy.loginfo("%s: global brake activated",self.veh_name)
        self.local_estop   = True
        rospy.loginfo("%s: local brake activated",self.veh_name)
        # Subscribers
        # self.sub_cmd     = rospy.Subscriber("wheels_cmd", WheelsCmd, self.wheels_cmd_cb, queue_size=1)

        self.pub_wheel_debug = rospy.Publisher('wheel_ref',WheelsCmd,queue_size=1)
        if not self.direct_mode:
            self.sub_wheel_enc = rospy.Subscriber("wheel_omega",WheelsEncoder,self.wheel_omega_cb,queue_size=1)
            self.sub_car_cmd   = rospy.Subscriber("cmd_vel", Twist, self.car_cmd_cb)
        else:
            self.sub_wheel_cmd = rospy.Subscriber("throttle",WheelsCmd,self.wheel_direct_cb,queue_size=1)
            
        self.sub_e_stop         = rospy.Subscriber("/global_brake", Bool, self.estop_cb, queue_size=1)
        self.sub_local_e_stop   = rospy.Subscriber("local_brake", Bool, self.estop_local_cb, queue_size=1)

        rospy.Subscriber("robot_interface_shutdown", Bool, self.signal_shut)
        self.dyna_trim = rospy.get_param('~dyna_trim', False)
        

        # self.trim_pid = PI_controller(kp=0.1, ki=0.01)
        if self.dyna_trim:
            self.sub_imu = rospy.Subscriber("imu", Imu, self.imu_cb)
        # self.pub_wheel_dir = rospy.Publisher('wheel_direction')
        
        self.srv_left = Server(omegaConfig, self.dynamic_reconfigure_callback_left, namespace='left_wheel')
        self.srv_right = Server(omegaConfig, self.dynamic_reconfigure_callback_right, namespace='right_wheel')
        if self.if_dyna_trim:
            self.srv_yaw = Server(yawConfig, self.dynamic_reconfigure_callback_yaw, namespace='yaw_pid')
        rospy.loginfo("%s: wheel drivers ready",self.veh_name)

        
    def signal_shut(self,msg:Bool):
        if msg.data:
            rospy.signal_shutdown('wheel driver node shutdown')

    def car_cmd_cb(self,msg_car_cmd:Twist) -> None:
        msg_car_cmd.linear.x    = max(min(msg_car_cmd.linear.x,self._v_max),-self._v_max)
        msg_car_cmd.angular.z   = max(min(msg_car_cmd.angular.z,self._omega_max),-self._omega_max)
        self.yaw_setpoint = -msg_car_cmd.angular.z  # Negate the yaw setpoint
        self.omega_right_ref    = 0
        self.omega_left_ref     = 0
        if not self.estop:
            if msg_car_cmd.linear.x != 0:
                self.omega_right_ref    = ((msg_car_cmd.linear.x + 0.5 * msg_car_cmd.angular.z * self._baseline) / self._radius) 
                self.omega_left_ref     = ((msg_car_cmd.linear.x - 0.5 * msg_car_cmd.angular.z * self._baseline) / self._radius) 
        

        #print('ref',self.omega_left_ref,self.omega_right_ref)
        msg_wheel_cmd = WheelsCmd()
        msg_wheel_cmd.vel_left  = self.omega_left_ref
        msg_wheel_cmd.vel_right = self.omega_right_ref
        msg_wheel_cmd.throttle_left = self.throttle_left
        msg_wheel_cmd.throttle_right = self.throttle_right
        self.pub_wheel_debug.publish(msg_wheel_cmd)

    def estop_cb(self,msg:Bool) -> None:
        self.estop = msg.data
        rospy.loginfo_once('%s: global brake: %s',self.veh_name,str(msg.data))

    def estop_local_cb(self,msg:Bool) -> None:
        self.local_estop = msg.data
        rospy.loginfo_once('%s: local brake: %s',self.veh_name,str(msg.data))        
    
    def shut_hook(self) -> None:
        self.estop = True
        self.driver.set_wheels_throttle(left=0,right=0)
        self.driver = None
        rospy.loginfo("%s: Wheel driver shutdown",self.veh_name)

    def wheel_direct_cb(self,msg:WheelsCmd) -> None:
        
        self.throttle_left  = msg.throttle_left
        self.throttle_right = msg.throttle_right
        
        if not self.estop and not self.local_estop:
            self.driver.set_wheels_throttle(left=self.throttle_left,right=self.throttle_right)
        else:
            self.driver.set_wheels_throttle(left=0,right=0)
            self.omega_controller_left.reset()
            self.omega_controller_right.reset()
    
    def wheel_omega_cb(self,msg:WheelsEncoder) -> None:

        self.omega_left_sig     = msg.omega_left
        self.omega_right_sig    = msg.omega_right
        #print('signal',self.omega_left_sig,self.omega_right_sig)

        # Calculate the current yaw rate from wheel speeds
        if self.if_dyna_trim:
            current_yaw_rate = (self.omega_right_sig - self.omega_left_sig) * self._radius / self._baseline
            self.yaw += current_yaw_rate * 1/20
            # Update the yaw PID controller
            self.yaw_trim = self.yaw_pid.pi_control(self.yaw_setpoint, current_yaw_rate,False)
            if self.omega_left_ref <=0 or self.omega_right_ref <=0:
                self.yaw_trim = 0
                self.yaw_pid.reset_controller()
                self.yaw = 0
                # Restrict yaw_trim to ±0.2
                self.yaw_trim = max(min(self.yaw_trim, 1), -1)


            # print('yaw',self.yaw,'trim',self.yaw_trim,'output',_output)

        self.throttle_left      = self.omega_controller_left.update(self.omega_left_ref*(1-self.yaw_trim),self.omega_left_sig,1/20)
        self.throttle_right     = self.omega_controller_right.update(self.omega_right_ref*(1+self.yaw_trim),self.omega_right_sig,1/20)

        if self.omega_left_sig <= 0.15/(self._radius):
            self.throttle_left <= 0.5 # anti-sliding
        if self.omega_right_sig <= 0.15/(self._radius):
            self.throttle_right <= 0.5
        
        if self.omega_left_ref == 0:
            self.throttle_left = 0
            self.omega_controller_left.reset()

        if self.omega_right_ref == 0:
            self.throttle_right = 0
            self.omega_controller_right.reset()     
        if not self.estop and not self.local_estop:
            self.driver.set_wheels_throttle(left=self.throttle_left,right=self.throttle_right)
        else:
            self.driver.set_wheels_throttle(left=0,right=0)
            self.omega_controller_left.reset()
            self.omega_controller_right.reset()

        # Apply the yaw trim to the throttle
        self.throttle_left -= self.yaw_trim
        self.throttle_right += self.yaw_trim

        # Ensure throttle values are within bounds
        self.throttle_left = max(min(self.throttle_left, 1.0), -1.0)
        self.throttle_right = max(min(self.throttle_right, 1.0), -1.0)
            
    def imu_cb(self, msg: Imu) -> None:
        """Callback function to handle IMU data."""
        self.yaw_measure = msg.angular_velocity.z
        # rospy.loginfo(f"Received IMU data: {msg}, Updated trim: {self.trim}")

    def dynamic_reconfigure_callback_left(self, config, level):
        self.kp_left = config.kp
        self.ki_left = config.ki
        self.kff_left = config.kff
        self.bff_left = config.bff
        rospy.loginfo(f"Dynamic reconfigure callback (left wheel): kp={self.kp_left}, ki={self.ki_left}, kff={self.kff_left}, bff={self.bff_left}")
        self.omega_controller_left.changeparam(kp=self.kp_left, ki=self.ki_left, kff=self.kff_left, bff=self.bff_left)
        self._log_settings('left_wheel', config)
        return config

    def dynamic_reconfigure_callback_right(self, config, level):
        self.kp_right = config.kp
        self.ki_right = config.ki
        self.kff_right = config.kff
        self.bff_right = config.bff
        rospy.loginfo(f"Dynamic reconfigure callback (right wheel): kp={self.kp_right}, ki={self.ki_right}, kff={self.kff_right}, bff={self.bff_right}")
        self.omega_controller_right.changeparam(kp=self.kp_right, ki=self.ki_right, kff=self.kff_right, bff=self.bff_right)
        self._log_settings('right_wheel', config)
        return config

    def dynamic_reconfigure_callback_yaw(self, config, level):
        self.kp_yaw = config.kp_yaw
        self.ki_yaw = config.ki_yaw
        rospy.loginfo(f"Dynamic reconfigure callback (yaw PID): kp_yaw={self.kp_yaw}, ki_yaw={self.ki_yaw}")
        self.yaw_pid.update_controller_param(kp=self.kp_yaw, ki=self.ki_yaw)
        return config

    def _log_settings(self, wheel, config):
        log_file = os.path.join(self.log_dir, f'{wheel}_settings.json')
        os.makedirs(self.log_dir, exist_ok=True)
        with open(log_file, 'w') as f:
            json.dump(config, f, indent=4)
        rospy.loginfo(f"Settings for {wheel} logged to {log_file}")

    def _read_settings(self, wheel):
        log_file = os.path.join(self.log_dir, f'{wheel}_settings.json')
        if os.path.exists(log_file):
            with open(log_file, 'r') as f:
                config = json.load(f)
                kp = config.get('kp', self.DEFAULT_KP)
                ki = config.get('ki', self.DEFAULT_KI)
                kff = config.get('kff', self.DEFAULT_KFF)
                bff = config.get('bff', self.DEFAULT_BFF)
                rospy.loginfo(f"Settings for {wheel} read from {log_file}")
        else:
            kp = self.DEFAULT_KP
            ki = self.DEFAULT_KI
            kff = self.DEFAULT_KFF
            bff = self.DEFAULT_BFF
            rospy.loginfo(f"Default settings applied for {wheel}")
        return kp, ki, kff, bff

if __name__ == '__main__':

    try:
        rospy.init_node("wheel_driver")
        N = WheelDriverNode()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('Keyboard Shutdown')

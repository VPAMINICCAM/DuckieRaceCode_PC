#!/usr/bin/env python3
# Refactored for better object-oriented structure

from enum import Enum
import smbus
import socket
import rospy
import numpy as np
from sensor_msgs.msg import Temperature, Imu
from std_msgs.msg import Bool
from tf.transformations import quaternion_about_axis

class MPU6050Registers(Enum):
    CONFIG = 0x1A
    PWR_MGMT_1 = 0x6B
    ACCEL_CONFIG = 0x1C
    ACCEL_XOUT_H = 0x3B
    ACCEL_XOUT_L = 0x3C
    ACCEL_YOUT_H = 0x3D
    ACCEL_YOUT_L = 0x3E
    ACCEL_ZOUT_H = 0x3F
    ACCEL_ZOUT_L = 0x40
    GYRO_CONFIG = 0x1B
    GYRO_XOUT_H = 0x43
    GYRO_XOUT_L = 0x44
    GYRO_YOUT_H = 0x45
    GYRO_YOUT_L = 0x46
    GYRO_ZOUT_H = 0x47
    GYRO_ZOUT_L = 0x48
    TEMP_H = 0x41
    TEMP_L = 0x42


class MPU6050Node:
    def __init__(self):
        rospy.init_node('imu_node')

        self.robot_name = socket.gethostname()
        self.bus = smbus.SMBus(rospy.get_param('~bus', 6))
        self.addr = self._get_device_address(rospy.get_param('~device_address', 0x68))
        self.imu_frame = rospy.get_param('~imu_frame', 'imu_link')
        self.publish_freq = rospy.get_param('~imu_pub_freq', 20)

        self.dlpf_cfg = rospy.get_param('~dlpf_cfg', 2)  # Default to 92Hz bandwidth

        self.temp_pub = rospy.Publisher('temperature', Temperature, queue_size=1)
        self.imu_pub = rospy.Publisher('imu', Imu, queue_size=1)

        # Initialize MPU6050
        self.bus.write_byte_data(self.addr, MPU6050Registers.PWR_MGMT_1.value, 0)
        self.set_dlpf(self.dlpf_cfg)

        # Start publishing timers
        self.imu_timer = rospy.Timer(rospy.Duration(1 / self.publish_freq), self.publish_imu)
        self.temp_timer = rospy.Timer(rospy.Duration(10), self.publish_temp)

        rospy.on_shutdown(self.cleanup)

        rospy.loginfo(f"{self.robot_name}: IMU sensor ready")

    @staticmethod
    def _get_device_address(addr):
        if isinstance(addr, str):
            return int(addr, 16)
        return addr

    def set_dlpf(self, dlpf_cfg):
        """Set the Digital Low Pass Filter configuration."""
        if not (0 <= dlpf_cfg <= 6):
            rospy.logwarn(f"Invalid DLPF_CFG value {dlpf_cfg}. Must be between 0 and 6. Defaulting to 2.")
            dlpf_cfg = 2
        self.bus.write_byte_data(self.addr, MPU6050Registers.CONFIG.value, dlpf_cfg)
        rospy.loginfo(f"DLPF set to configuration {dlpf_cfg}.")

    def read_word(self, register):
        high = self.bus.read_byte_data(self.addr, register.value)
        low = self.bus.read_byte_data(self.addr, register.value + 1)
        return (high << 8) + low

    def read_word_2c(self, register):
        val = self.read_word(register)
        if val >= 0x8000:
            return -((65535 - val) + 1)
        return val

    def publish_temp(self, timer_event):
        temp_msg = Temperature()
        temp_msg.header.frame_id = self.imu_frame
        temp_msg.temperature = self.read_word_2c(MPU6050Registers.TEMP_H) / 340.0 + 36.53
        temp_msg.header.stamp = rospy.Time.now()
        self.temp_pub.publish(temp_msg)

    def publish_imu(self, timer_event):
        imu_msg = Imu()
        imu_msg.header.frame_id = self.imu_frame

        # Read acceleration values
        accel_x = self.read_word_2c(MPU6050Registers.ACCEL_XOUT_H) / 16384.0
        accel_y = self.read_word_2c(MPU6050Registers.ACCEL_YOUT_H) / 16384.0
        accel_z = self.read_word_2c(MPU6050Registers.ACCEL_ZOUT_H) / 16384.0

        # Calculate orientation quaternion
        accel = np.array([accel_x, accel_y, accel_z])
        ref = np.array([0, 0, 1])
        acceln = accel / np.linalg.norm(accel)
        axis = np.cross(acceln, ref)
        angle = np.arccos(np.dot(acceln, ref))
        orientation = quaternion_about_axis(angle, axis)

        # Read gyro values
        gyro_x = self.read_word_2c(MPU6050Registers.GYRO_XOUT_H) / 131.0 * (np.pi / 180)
        gyro_y = self.read_word_2c(MPU6050Registers.GYRO_YOUT_H) / 131.0 * (np.pi / 180)
        gyro_z = self.read_word_2c(MPU6050Registers.GYRO_ZOUT_H) / 131.0 * (np.pi / 180)

        # Populate IMU message
        imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w = orientation
        imu_msg.linear_acceleration.x = accel_x
        imu_msg.linear_acceleration.y = accel_y
        imu_msg.linear_acceleration.z = accel_z
        imu_msg.angular_velocity.x = gyro_x
        imu_msg.angular_velocity.y = gyro_y
        imu_msg.angular_velocity.z = gyro_z
        imu_msg.header.stamp = rospy.Time.now()

        self.imu_pub.publish(imu_msg)

    def run(self):
        rospy.spin()

    def cleanup(self):
        """Cleanup resources on shutdown."""
        rospy.loginfo(f"{self.robot_name}: Cleaning up resources.")
        self.imu_timer.shutdown()
        self.temp_timer.shutdown()


if __name__ == '__main__':
    node = MPU6050Node()
    node.run()
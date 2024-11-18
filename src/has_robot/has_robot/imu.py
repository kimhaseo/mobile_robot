import serial
import time
import re
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from math import sin, cos, radians
import math


ser = serial.Serial("/dev/ttyahrs", 9600, timeout=0.1)

class ImuNode(Node):
    def __init__(self):
        super().__init__("imu_node")
        self.publisher_ = self.create_publisher(Imu, "imu", 10)
        self.timer_ = self.create_timer(0.1, self.publish_imu_data)

    def publish_imu_data(self):
        if ser.readline():
            angle = ser.readline()
            serial_string = angle.hex()

            data1 = re.findall(r'5551..................5552', str(serial_string))
            data2 = re.findall(r'5552..................5553', str(serial_string))
            data3 = re.findall(r'5553..................5554', str(serial_string))

            if len(data1) & len(data2) & len(data3) != 0:

                if len(data1[0]) == 26:
                    acc = (data1[0])
                    AxH = acc[4:6]
                    AxH = int(AxH, 16)
                    AxL = acc[6:8]
                    AxL = int(AxL, 16)
                    AyH = acc[8:10]
                    AyH = int(AyH, 16)
                    AyL = acc[10:12]
                    AyL = int(AyL, 16)
                    AzH = acc[12:14]
                    AzH = int(AzH, 16)
                    AzL = acc[14:16]
                    AzL = int(AzL, 16)

                    ax = ((AxH << 8) | AxL) / 32768 * 16 * 9.8
                    ay = ((AyH << 8) | AyL) / 32768 * 16 * 9.8
                    az = ((AzH << 8) | AzL) / 32768 * 16 * 9.8

                    imu_msg = Imu()
                    imu_msg.header.frame_id = "base_imu"  # Replace with your desired frame id
                    imu_msg.linear_acceleration.x = ax
                    imu_msg.linear_acceleration.y = ay
                    imu_msg.linear_acceleration.z = az
                    # Set other fields of the IMU message (orientation, angular velocity) if available

                if len(data2[0]) == 26:
                    agv = (data2[0])
                    wxL = agv[4:6]
                    wxL = int(wxL, 16)
                    wxH = agv[6:8]
                    wxH = int(wxH, 16)
                    wyL = agv[8:10]
                    wyL = int(wyL, 16)
                    wyH = agv[10:12]
                    wyH = int(wyH, 16)
                    wzL = agv[12:14]
                    wzL = int(wzL, 16)
                    wzH = agv[14:16]
                    wzH = int(wzH, 16)

                    wx = ((wxH << 8) | wxL) / 32768 * 2000
                    wy = ((wyH << 8) | wyL) / 32768 * 2000
                    wz = ((wzH << 8) | wzL) / 32768 * 2000

                    wx_rad = math.radians(wx)
                    wy_rad = math.radians(wy)
                    wz_rad = math.radians(wz)

                    imu_msg.angular_velocity.x = wx_rad
                    imu_msg.angular_velocity.y = wy_rad
                    imu_msg.angular_velocity.z = wz_rad

                if len(data3[0]) == 26:
                    ang = (data3[0])
                    RollL = ang[4:6]
                    RollL = int(RollL, 16)
                    RollH = ang[6:8]
                    RollH = int(RollH, 16)
                    PitchL = ang[8:10]
                    PitchL = int(PitchL, 16)
                    PitchH = ang[10:12]
                    PitchH = int(PitchH, 16)
                    YawL = ang[12:14]
                    YawL = int(YawL, 16)
                    YawH = ang[14:16]
                    YawH = int(YawH, 16)


                    roll = ((RollH << 8) | RollL) / 32768 * 180
                    pitch = ((PitchH << 8) | PitchL) / 32768 * 180
                    yaw = ((YawH << 8) | YawL) / 32768 * 180
                    yaw = wrap_to_180(yaw)
                    yaw= -(yaw)
                    
                    # Convert to quaternion
                    roll_rad = radians(roll)
                    pitch_rad = radians(pitch)
                    yaw_rad = -radians(yaw)

                    qx = np.sin(roll_rad/2) * np.cos(pitch_rad/2) * np.cos(yaw_rad/2) - np.cos(roll_rad/2) * np.sin(pitch_rad/2) * np.sin(yaw_rad/2)
                    qy = np.cos(roll_rad/2) * np.sin(pitch_rad/2) * np.cos(yaw_rad/2) + np.sin(roll_rad/2) * np.cos(pitch_rad/2) * np.sin(yaw_rad/2)
                    qz = np.cos(roll_rad/2) * np.cos(pitch_rad/2) * np.sin(yaw_rad/2) - np.sin(roll_rad/2) * np.sin(pitch_rad/2) * np.cos(yaw_rad/2)
                    qw = np.cos(roll_rad/2) * np.cos(pitch_rad/2) * np.cos(yaw_rad/2) + np.sin(roll_rad/2) * np.sin(pitch_rad/2) * np.sin(yaw_rad/2)
                    
                    imu_msg.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
                    print(yaw_rad)
                self.publisher_.publish(imu_msg)

def wrap_to_180(angle):
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle

def main(args=None):
    rclpy.init(args=args)
    imu_node = ImuNode()
    rclpy.spin(imu_node)
    imu_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
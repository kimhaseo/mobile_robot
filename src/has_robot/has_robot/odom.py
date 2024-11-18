import RPi.GPIO as GPIO
import time
import math
import rclpy
from std_msgs.msg import Int32
from sensor_msgs.msg import Imu
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist
from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import numpy as np

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

l_encPinA = 14
l_encPinB = 15
r_encPinA = 17
r_encPinB = 27

GPIO.setup(l_encPinA, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(l_encPinB, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(r_encPinA, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(r_encPinB, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# 엔코더 관련 변수


# ROS 노드 생성
class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')

        # 퍼블리셔 생성
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        # IMU 토픽 구독
        self.imu_sub = self.create_subscription(Imu, 'imu', self.imu_callback, 10)

        self.timer_ = self.create_timer(0.1, self.run)

        # 엔코더 관련 변수
        self.l_encoderPos = 0
        self.r_encoderPos = 0
        self.wheel_radius = 0.0315  # 바퀴 반지름 (미터)
        self.encoder_resolution = 4532  # 엔코더 해상도 (한 주기당 펄스 수)

        # 초기 값 설정
        self.last_l_encoderPos = self.l_encoderPos
        self.last_r_encoderPos = self.r_encoderPos
        self.last_time = time.time()

        # 모션 변수 초기화
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        # 엔코더 콜백 등록
        GPIO.add_event_detect(l_encPinA, GPIO.BOTH, callback=self.l_encoderA)
        GPIO.add_event_detect(l_encPinB, GPIO.BOTH, callback=self.l_encoderB)
        GPIO.add_event_detect(r_encPinA, GPIO.BOTH, callback=self.r_encoderA)
        GPIO.add_event_detect(r_encPinB, GPIO.BOTH, callback=self.r_encoderB)

        self.rate = self.create_rate(10)  # 10 Hz

    def imu_callback(self, msg):
        quaternion = msg.orientation
        _, _, theta = self.euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        self.theta = theta  # self.theta 업데이트

    def l_encoderA(self, channel):
        if GPIO.input(l_encPinA) == GPIO.input(l_encPinB):
            self.l_encoderPos += 1
        else:
            self.l_encoderPos -= 1

    def l_encoderB(self, channel):
        if GPIO.input(l_encPinA) == GPIO.input(l_encPinB):
            self.l_encoderPos -= 1
        else:
            self.l_encoderPos += 1

    def r_encoderA(self, channel):
        if GPIO.input(r_encPinA) == GPIO.input(r_encPinB):
            self.r_encoderPos += 1
        else:
            self.r_encoderPos -= 1

    def r_encoderB(self, channel):
        if GPIO.input(r_encPinA) == GPIO.input(r_encPinB):
            self.r_encoderPos -= 1
        else:
            self.r_encoderPos += 1

    def quaternion_from_euler(self, roll, pitch, yaw):
        # cy = math.cos(yaw * 0.5)
        # sy = math.sin(yaw * 0.5)
        # cp = math.cos(pitch * 0.5)
        # sp = math.sin(pitch * 0.5)
        # cr = math.cos(roll * 0.5)
        # sr = math.sin(roll * 0.5)

        q = [0] * 4
        # q[0] = cy * cp * cr + sy * sp * sr
        # q[1] = cy * cp * sr - sy * sp * cr
        # q[2] = sy * cp * sr + cy * sp * cr
        # q[3] = sy * cp * cr - cy * sp * sr

        q[0] = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        q[1] = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        q[2] = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        q[3] = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)


        return q


    def euler_from_quaternion(self, quaternion):
        x = quaternion[0]
        y = quaternion[1]
        z = quaternion[2]
        w = quaternion[3]

        # roll (x 축 회전)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # pitch (y 축 회전)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # 범위를 벗어나면 90도 사용
        else:
            pitch = math.asin(sinp)

        # yaw (z 축 회전)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def run(self):
        # 엔코더 값과 시간 간격 계산
        self.l_encoder_count = self.l_encoderPos - self.last_l_encoderPos
        self.r_encoder_count = self.r_encoderPos - self.last_r_encoderPos
        dt = time.time() - self.last_time

        # 엔코더 값을 사용하여 속도 계산
        l_wheel_velocity = (self.l_encoder_count / self.encoder_resolution) * (2 * math.pi * self.wheel_radius) / dt
        r_wheel_velocity = (self.r_encoder_count / self.encoder_resolution) * (2 * math.pi * self.wheel_radius) / dt

        # 엔코더 값과 시간 업데이트
        self.last_l_encoderPos = self.l_encoderPos
        self.last_r_encoderPos = self.r_encoderPos
        self.last_time = time.time()

        # 모션 변수 업데이트
        self.linear_velocity = (l_wheel_velocity + r_wheel_velocity) / 2

        self.angular_velocity = (r_wheel_velocity - l_wheel_velocity) / (2 * self.wheel_radius)
        
        # 오도메트리 업데이트
        self.x += self.linear_velocity * math.cos(self.theta) * dt
        self.y += -(self.linear_velocity * math.sin(self.theta) * dt)
        if self.theta < -math.pi:
            self.theta += 2 * math.pi
        elif self.theta > math.pi:
            self.theta -= 2 * math.pi

        # 오도메트리 메시지 생성
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        quaternion = self.quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation = Quaternion(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])
        odom_msg.twist.twist.linear.x = self.linear_velocity
        odom_msg.twist.twist.angular.z = self.angular_velocity

        print(odom_msg.pose.pose.position.x,'--',odom_msg.pose.pose.position.y)

        # 오도메트리 메시지 발행
        self.odom_pub.publish(odom_msg)



def main(args=None):
    rclpy.init(args=args)
    odometry_node = OdometryNode()
    rclpy.spin(odometry_node)
    odometry_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


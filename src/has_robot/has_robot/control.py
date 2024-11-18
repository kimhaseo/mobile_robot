import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, Vector3, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from math import atan2, sqrt, asin
import RPi.GPIO as GPIO
from rclpy.time import Time

class PIDControlNode(Node):
    def __init__(self):
        super().__init__('pid_control_node')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('kp_linear', 200.0),    # 선형 속도 PID 게인
                ('ki_linear', 20.0),
                ('kd_linear', 0.05),
                ('kp_angular', 3.0),   # 각속도 PID 게인
                ('ki_angular', 0.3),
                ('kd_angular', 0.05),
                ('kp_position', 1.0),  # 위치 PID 게인
                ('ki_position', 0.03),
                ('kd_position', 0.01),
                ('kp_yaw', 6.0),  # 각도 PID 게인
                ('ki_yaw', 0.25),
                ('kd_yaw', 0.001),
            ]
        )

        self.last_time = self.get_clock().now()

        self.kp_linear = self.get_parameter('kp_linear').value
        self.ki_linear = self.get_parameter('ki_linear').value
        self.kd_linear = self.get_parameter('kd_linear').value
        self.kp_angular = self.get_parameter('kp_angular').value
        self.ki_angular = self.get_parameter('ki_angular').value
        self.kd_angular = self.get_parameter('kd_angular').value
        self.kp_position = self.get_parameter('kp_position').value
        self.ki_position = self.get_parameter('ki_position').value
        self.kd_position = self.get_parameter('kd_position').value
        self.kp_yaw = self.get_parameter('kp_yaw').value
        self.ki_yaw = self.get_parameter('ki_yaw').value
        self.kd_yaw = self.get_parameter('kd_yaw').value

        self.target_linear_velocity = 0.0  # 원하는 선형 속도 값 설정
        self.target_angular_velocity = 0.0  # 원하는 각속도 값 설정
        self.current_linear_velocity = 0.0
        self.current_angular_velocity = 0.0
        self.current_position = (0.0, 0.0)
        self.current_yaw = 0.0

        self.linear_error_sum = 0.0
        self.angular_error_sum = 0.0
        self.position_error_sum = 0.0
        self.yaw_error_sum = 0.0
        self.current_yaw = 0.0

        self.last_linear_error = 0.0
        self.last_angular_error = 0.0
        self.last_position_error = 0.0
        self.last_yaw_error = 0.0
        self.target_position = (0.0, 0.0)
        self.target_yaw = 0.0

        # GPIO 핀 번호 설정
        self.l_in1_pin = 23
        self.l_in2_pin = 24
        self.r_in1_pin = 10
        self.r_in2_pin = 9

        self.l_pwm_pin = 18
        self.r_pwm_pin = 22

        GPIO.setwarnings(False)
        # GPIO 초기화 및 설정
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.l_in1_pin, GPIO.OUT)
        GPIO.setup(self.l_in2_pin, GPIO.OUT)
        GPIO.setup(self.r_in1_pin, GPIO.OUT)
        GPIO.setup(self.r_in2_pin, GPIO.OUT)
        GPIO.setup(self.l_pwm_pin, GPIO.OUT)
        GPIO.setup(self.r_pwm_pin, GPIO.OUT)

        self.l_pwm = GPIO.PWM(self.l_pwm_pin, 100)
        self.l_pwm.start(0)
        self.r_pwm = GPIO.PWM(self.r_pwm_pin, 100)
        self.r_pwm.start(0)


        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )
        self.goal_pose_subscription = self.create_subscription(
            PoseStamped,
            'goal_pose',
            self.goal_pose_callback,
            10
        )
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.timer = self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        current_time = self.get_clock().now()  # 현재 시간 측정
        dt = (current_time - self.last_time).nanoseconds / 1e9  # 시간 차이 계산 (초 단위로 변환)
        linear_control = self.calculate_linear_control(dt)  # 시간(dt)을 인수로 추가하여 호출
        angular_control = self.calculate_angular_control(dt)  # 시간(dt)을 인수로 추가하여 호출
        position_control, yaw_control = self.calculate_pose_control(dt)  # 시간(dt)을 인수로 추가하여 호출
        self.control_motor(linear_control, angular_control, position_control, yaw_control)
        self.last_time = current_time  # 이전 시간 업데이트


    def cmd_vel_callback(self, msg):
        self.target_linear_velocity = msg.linear.x
        self.target_angular_velocity = msg.angular.z

    def goal_pose_callback(self, msg):
        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y
        _, _, goal_yaw = self.quaternion_to_euler(
            msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)

        self.target_position = (goal_x, goal_y)
        # self.target_yaw = goal_yaw

    def odom_callback(self, msg):
        self.current_linear_velocity = msg.twist.twist.linear.x
        self.current_angular_velocity = msg.twist.twist.angular.z
        self.current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        _, _, theta = self.quaternion_to_euler(
            msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        self.current_yaw = theta  # self.theta 업데이트

    def quaternion_to_euler(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = atan2(t3, t4)

        return roll_x, pitch_y, yaw_z
        
    def calculate_linear_control(self, dt):

        linear_error = self.target_linear_velocity - self.current_linear_velocity
        linear_error = max(min(linear_error, 100), -100)
        self.linear_error_sum += linear_error
        self.linear_error_sum = max(min(self.linear_error_sum, 100), -100)
        linear_error_diff = (linear_error - self.last_linear_error)/ dt  # 미분항 계산 (변화율 / 시간)
        linear_control = (
            self.kp_linear * linear_error +
            self.ki_linear * self.linear_error_sum +
            self.kd_linear * linear_error_diff
        )
        self.last_linear_error = linear_error

        return linear_control


    def calculate_angular_control(self, dt):
        angular_error = self.target_angular_velocity - self.current_angular_velocity
        self.angular_error_sum += angular_error
        self.angular_error_sum = max(min(self.angular_error_sum, 100), -100)
        angular_error_diff = (angular_error - self.last_angular_error)/ dt  # 미분항 계산 (변화율 / 시간)

        angular_control = (
            self.kp_angular * angular_error +
            self.ki_angular * self.angular_error_sum +
            self.kd_angular * angular_error_diff
        )
        self.last_angular_error = angular_error
        return angular_control

    def calculate_pose_control(self, dt):

        self.current_position = list(self.current_position)
        self.current_position[0] = int(self.current_position[0] * 100) / 100
        self.current_position[1] = int(self.current_position[1] * 100) / 100
        self.current_position = tuple(self.current_position)
        print(self.current_position[0])
        position_error = sqrt(
            (self.target_position[0] - self.current_position[0])**2 +(self.target_position[1] - self.current_position[1])**2
        )
        if position_error < 0.001:
            position_error=0

        x1, y1 = self.current_position
        # 목표 위치
        x2, y2 = self.target_position
        direction_vector = (x2 - x1, y2 - y1)
        self.target_yaw = -math.atan2(direction_vector[1], direction_vector[0])

        yaw_error = self.target_yaw - self.current_yaw
        if yaw_error < -math.pi:
            yaw_error += 2 * math.pi

        # 만약 에러가 파이보다 크다면, 2파이를 빼줍니다.
        if yaw_error > math.pi:
            yaw_error -= 2 * math.pi
        yaw_error = max(min(yaw_error, 100), -100)

        self.position_error_sum += position_error
        self.yaw_error_sum += yaw_error
        
        self.position_error_sum = max(min(self.position_error_sum, 1000), -1000)
        self.yaw_error_sum = max(min(self.yaw_error_sum, 30), -30)

        position_error_diff = (position_error - self.last_position_error)/ dt  # 미분항 계산 (변화율 / 시간))
        yaw_error_diff = (yaw_error - self.last_yaw_error)/ dt  # 미분항 계산 (변화율 / 시간)

        position_control = (
            self.kp_position * position_error +
            self.ki_position * self.position_error_sum +
            self.kd_position * position_error_diff 
        )

        yaw_control = (
            self.kp_yaw * yaw_error +
            self.ki_yaw * self.yaw_error_sum +
            self.kd_yaw * yaw_error_diff
        )
        if position_error < 0.03 :
            position_control = 0 

        return position_control, yaw_control

    def control_motor(self, linear_control, angular_control, position_control, yaw_control):
        
        # 모터 제어 코드 추가

        # l_pwm_value = linear_control - angular_control
        # r_pwm_value = linear_control + angular_control

        l_pwm_value = -yaw_control + position_control
        r_pwm_value = yaw_control + position_control

        print(position_control,'--',self.target_yaw,'--',self.current_position)
        l_pwm_value = min(max(l_pwm_value, -100), 100)
        r_pwm_value = min(max(r_pwm_value, -100), 100)
        
        ## 전진
        if l_pwm_value >= 0:           
            GPIO.output(self.l_in1_pin, GPIO.LOW)
            GPIO.output(self.l_in2_pin, GPIO.HIGH)
        else:
            GPIO.output(self.l_in1_pin, GPIO.HIGH)
            GPIO.output(self.l_in2_pin, GPIO.LOW)
        ## 후진

        if r_pwm_value >= 0:             
            GPIO.output(self.r_in1_pin, GPIO.HIGH)
            GPIO.output(self.r_in2_pin, GPIO.LOW)
        else:                                         
            GPIO.output(self.r_in1_pin, GPIO.LOW)
            GPIO.output(self.r_in2_pin, GPIO.HIGH)

        self.l_pwm.ChangeDutyCycle(abs(l_pwm_value))
        self.r_pwm.ChangeDutyCycle(abs(r_pwm_value))


def main(args=None):
    rclpy.init(args=args)

    pid_control_node = PIDControlNode()

    rclpy.spin(pid_control_node)

    pid_control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
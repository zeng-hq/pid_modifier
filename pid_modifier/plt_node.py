import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor
import subprocess
import matplotlib.pyplot as plt

from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.srv import SetMode, CommandBool
from mavros_msgs.msg import PositionTarget
import numpy as np
import math
import time
import re
# from math import radians


qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
  

def set_param(node_name, param_name, param_value):
    # rclpy.init()
    # node_name = '/mavros/param'
    # param_name = 'MC_PITCHRATE_P'
    # param_value = float(param_value)

    command = ['ros2', 'param', 'set', node_name, param_name, str(param_value)]
    try:
        subprocess.run(command, check=True)

    except subprocess.CalledProcessError as e:
        print('Failed to set parameter:', str(e))
    # rclpy.shutdown()

def get_param(node_name, param_name):
    command = ['ros2', 'param', 'get', node_name, param_name]
    try:
        result = subprocess.run(command, check=True, stdout=subprocess.PIPE)
        output = result.stdout.decode('utf-8')
        return output

    except subprocess.CalledProcessError as e:
        print('Failed to set parameter:', str(e))

MAX_DATA_POINTS = 200

class PIDval():
    def __init__(self) -> None:
        self.params = ['MC_PITCHRATE_P',
                       'MC_PITCHRATE_I',
                       'MC_PITCHRATE_D',
                       'MC_ROLLRATE_P',
                       'MC_ROLLRATE_I',
                       'MC_ROLLRATE_D',
                       'MC_YAWRATE_P',
                       'MC_YAWRATE_I',
                       'MC_YAWRATE_D']
        
        self.values = {'MC_PITCHRATE_P' : 0.15,
                       'MC_PITCHRATE_I' : 0.2,
                       'MC_PITCHRATE_D' : 0.003,
                       'MC_ROLLRATE_P' : 0.15,
                       'MC_ROLLRATE_I' : 0.2,
                       'MC_ROLLRATE_D' : 0.003,
                       'MC_YAWRATE_P' : 0.2,
                       'MC_YAWRATE_I' : 0.1,
                       'MC_YAWRATE_D' : 0.0
        }

class EulerAngleSubscriber(Node):
    def __init__(self):
        super().__init__('euler_angle_subscriber')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.euler_angle_callback,
            qos_profile
        )
        self.euler_angles = {'roll': [], 'pitch': [], 'yaw': []}
        # self.start_time = rclpy.clock.Clock().now()
        self.declare_parameter('threshold', 0.02)
        self.declare_parameter('window', 4)
        ParameterDescriptor(read_only=False)
        self.threshold = self.get_parameter('threshold').get_parameter_value().double_value
        self.get_logger().info(f'default threshold: {self.threshold}')
        self.window = self.get_parameter('window').get_parameter_value().integer_value
        self.get_logger().info(f'default window_size: {self.window}')
        self.timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        self.threshold = self.get_parameter('threshold').get_parameter_value().double_value
        # self.get_logger().info(f'threshold: {self.threshold}')
        self.window = self.get_parameter('window').get_parameter_value().integer_value
        # self.get_logger().info(f'window_size: {self.window}')

    def euler_angle_callback(self, msg):
        euler_angles = self.quaternion_to_euler(msg.pose.orientation)
        self.euler_angles['roll'].append(euler_angles[0])
        self.euler_angles['pitch'].append(euler_angles[1])
        self.euler_angles['yaw'].append(euler_angles[2])
        self.trim_data()

    def trim_data(self):
        if len(self.euler_angles['roll']) >= MAX_DATA_POINTS:
            # print('trim')
            for angle in self.euler_angles.values():
                del angle[0]
        # self.start_time = rclpy.clock.Clock().now()

    def quaternion_to_euler(self, orientation):
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w
        roll = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
        pitch = math.asin(2*(w*y - z*x))
        yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        return roll, pitch, yaw
    
def main(args=None):
    rclpy.init(args=args)
    euler_angle_subscriber = EulerAngleSubscriber()
    target_pid = PIDval()
    # for i in target_pid.params:
    #     set_param('/mavros/param', i, target_pid.values[i])


    # plt.figure()
    # plt.title('Euler Angles')
    # plt.xlabel('Time')
    # plt.ylabel('Angle (rad)')
    # plt.grid(True)

    while rclpy.ok():
        # current_time = rclpy.clock.Clock().now()
        # elapsed_time = current_time - start_time
        # print(elapsed_time)
        for i in range(10):
            rclpy.spin_once(euler_angle_subscriber)
        while rclpy.ok():            
            pit = np.array(euler_angle_subscriber.euler_angles['pitch'])
            window = euler_angle_subscriber.window
            threshold = euler_angle_subscriber.threshold

            x = np.arange(len(pit))
            smoothed_pit = np.convolve(np.array(euler_angle_subscriber.euler_angles['pitch']), np.ones(window)/window, mode='same')

            ##
            res_x = x[window-1:-window]
            res_pit = pit[window-1:-window]
            res_smoothed_pit = smoothed_pit[window-1:-window]
            diff = np.abs(res_smoothed_pit - res_pit)
            significant_indices = diff > threshold
            significant_x = res_x[significant_indices]
            significant_pit = res_pit[significant_indices]    

            # plt.title(f'pitch_p = {}')
            plt.plot(res_x, res_pit, label='PITCH')
            # plt.plot(res_x, res_smoothed_pit, label='average PITCH')
            # plt.scatter(significant_x, significant_pit, color='red', label='Significant Oscillations')
            plt.legend()
            plt.xlabel('PKG')
            plt.ylabel('EULER')
            plt.title('')
            plt.pause(0.01)
            plt.clf()
            # print(euler_angle_subscriber.euler_angles['pitch'][-1:])
            # print(f'window:{window} threshold:{threshold}')
            rclpy.spin_once(euler_angle_subscriber)

        # rclpy.spin_once(euler_angle_subscriber)
    print("exit")
    euler_angle_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
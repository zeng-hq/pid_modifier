import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import SetMode, CommandBool
from mavros_msgs.msg import PositionTarget, AttitudeTarget

import time, math

qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

class AttitudeController(Node):
    def __init__(self):
        super().__init__('attitude_controller')

        self.subscription = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.ori_callback, qos_profile)

        self.pos_local_publisher_ = self.create_publisher(PoseStamped, 'mavros/setpoint_position/local', qos_profile)
        self.attitude_publisher_ = self.create_publisher(AttitudeTarget, 'mavros/setpoint_raw/attitude', qos_profile)

        self.initpos = PoseStamped()
        self.initpos.pose.position.x = 400.0
        self.initpos.pose.position.y = 100.0
        self.initpos.pose.position.z = 20.0

        self.target1 = AttitudeTarget().orientation
        self.target1.x = 1.0
        self.target1.y = 0.0
        self.target1.z = 0.0
        self.target1.w = 1.0
        self.attitude = AttitudeTarget()
        self.attitude.type_mask = AttitudeTarget.IGNORE_PITCH_RATE \
                                | AttitudeTarget.IGNORE_YAW_RATE \
                                | AttitudeTarget.IGNORE_ROLL_RATE \
                                | AttitudeTarget.IGNORE_THRUST                      
        self.attitude.orientation = self.target1
        # self.attitude.header.stamp = self.get_clock().now().to_msg()
        # self.attitude.coordinate_frame = PositionTarget.FRAME_BODY_NED
        # self.attitude.type_mask = PositionTarget.FORCE \
        #                         | PositionTarget.IGNORE_VX \
        #                         | PositionTarget.IGNORE_VY \
        #                         | PositionTarget.IGNORE_VZ \
        #                         | PositionTarget.IGNORE_AFX \
        #                         | PositionTarget.IGNORE_AFY \
        #                         | PositionTarget.IGNORE_AFZ \
        #                         | PositionTarget.IGNORE_YAW_RATE
                                # | PositionTarget.IGNORE_PX \
                                # | PositionTarget.IGNORE_PY \
                                # | PositionTarget.IGNORE_PZ \
        # self.attitude.type_mask = 

        # msg.yaw = 0.0


        # for i in range(1):
        #     self.fly_to_init()
        #     time.sleep(0.1)

        # self.create_timer(0.1, self.attitude_callback)
    def ori_callback(self, msg):
        print(msg.pose.orientation)

    def fly_to_init(self):
        self.pos_local_publisher_.publish(self.initpos)
        
    def attitude_callback(self):
        self.attitude_publisher_.publish(self.attitude)
    
def main(args=None):
    time.sleep(1)
    rclpy.init(args=args)
    attitude_controller = AttitudeController()
    for i in range(10000):
        attitude_controller.fly_to_init()
        time.sleep(0.1)
    print('att')
    while True:
        attitude_controller.attitude_callback()
        time.sleep(0.1)
        # rclpy.spin_once(attitude_controller)
    attitude_controller.destroy_node()
    rclpy.shutdown()
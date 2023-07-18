import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import SetMode, CommandBool
from mavros_msgs.msg import PositionTarget

import time

qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

class OffboardController(Node):
    def __init__(self):
        super().__init__('offboard_controller')

        self.set_mode_service_ = self.create_client(SetMode, '/mavros/set_mode')
        self.arm_service_ = self.create_client(CommandBool, '/mavros/cmd/arming')

        self.setpoint_publisher_ = self.create_publisher(PositionTarget, '/mavros/setpoint_raw/local', qos_profile)
        self.pos_local_publisher_ = self.create_publisher(PoseStamped, 'mavros/setpoint_position/local', qos_profile)

        while not self.set_mode_service_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting offboard...')
        while not self.arm_service_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting armed...')

    def set_offboard_mode(self):
        msg = SetMode.Request()
        msg.custom_mode = 'OFFBOARD'

        future = self.set_mode_service_.call_async(msg)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            if future.result().mode_sent:
                self.get_logger().info('Offboard')
            else:
                self.get_logger().info('! Offboard FAILED !')
        else:
            self.get_logger().info('FAILED')

    def arm_drone(self):
        msg = PoseStamped()
        msg.pose.position.x = 0.0
        msg.pose.position.y = 0.0
        msg.pose.position.z = -20.0
        self.pos_local_publisher_.publish(msg)
        # for i in range(100):
        #     self.pos_local_publisher_.publish(msg)

        armCMD = CommandBool.Request()
        armCMD.value = True        
        future = self.arm_service_.call_async(armCMD)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            if future.result().success:
                self.get_logger().info('Armed')
                # self.takeoff()

            else:
                self.get_logger().info('Disarmed')
        else:
            self.get_logger().info('Failed')

    def set_pos(self):
        msg = PoseStamped()
        msg.pose.position.x = 0.0
        msg.pose.position.y = 0.0
        msg.pose.position.z = 22.0

        self.pos_local_publisher_.publish(msg)

    def set_desired_attitude(self):
        msg = PositionTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.coordinate_frame = PositionTarget.FRAME_BODY_NED
        msg.type_mask = PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ | PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ | PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ | PositionTarget.IGNORE_YAW_RATE
        msg.position.z = -50.0  # 设置飞行高度为负值（负值表示相对于起始高度的偏移）

        # msg.yaw = 0.0
        msg.position.x = 0.1
        msg.position.y = 0.0
        msg.position.z = 0.0

        self.setpoint_publisher_.publish(msg)

def offboard_control(args=None):
    rclpy.init(args=args)
    offboard_controller = OffboardController()

    offboard_controller.arm_drone()

    # offboard_controller.set_desired_attitude()
    while rclpy.ok():
        offboard_controller.set_offboard_mode()
        offboard_controller.set_pos()
        rclpy.spin_once(offboard_controller)
    offboard_controller.destroy_node()
    rclpy.shutdown()

##########################################################333
        
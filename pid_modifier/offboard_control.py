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
        msg.pose.position.z = 20.0
        self.pos_local_publisher_.publish(msg)
        for i in range(10):
            self.pos_local_publisher_.publish(msg)

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



def main(args=None):
    rclpy.init(args=args)
    offboard_controller = OffboardController()

    offboard_controller.arm_drone()

    while rclpy.ok():
        offboard_controller.set_offboard_mode()
        # offboard_controller.set_pos()
        time.sleep(0.3)
        rclpy.spin_once(offboard_controller)
    offboard_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
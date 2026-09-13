#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from px4_msgs.msg import VehicleCommand


class ArmDisarmNode(Node):

    def __init__(self):
        super().__init__('arm_disarm_node')

        # Publisher to PX4
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            10
        )

        self.state = 0

        # Give ROS 2 / PX4 communication a moment to initialize
        self.timer = self.create_timer(2.0, self.timer_callback)

        self.get_logger().info('Arm/Disarm node started')

    def timer_callback(self):

        # State 0: Arm
        if self.state == 0:
            self.get_logger().info('Sending ARM command...')
            self.arm()

            self.state = 1

            # Change timer so we wait 5 seconds before disarming
            self.timer.cancel()
            self.timer = self.create_timer(5.0, self.timer_callback)

        # State 1: Disarm
        elif self.state == 1:
            self.get_logger().info('Sending DISARM command...')
            self.disarm()

            self.state = 2
            self.timer.cancel()

            self.get_logger().info('Arm/Disarm sequence complete.')

    def arm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0
        )

    def disarm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=0.0
        )

    def publish_vehicle_command(
        self,
        command,
        param1=0.0,
        param2=0.0
    ):

        msg = VehicleCommand()

        msg.param1 = param1
        msg.param2 = param2

        msg.command = command

        # PX4 autopilot
        msg.target_system = 1
        msg.target_component = 1

        # ROS 2 node
        msg.source_system = 1
        msg.source_component = 1

        msg.from_external = True

        # PX4 expects microseconds
        msg.timestamp = (
            self.get_clock().now().nanoseconds // 1000
        )

        self.vehicle_command_publisher.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = ArmDisarmNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
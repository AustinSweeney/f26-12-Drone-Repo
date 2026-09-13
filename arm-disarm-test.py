#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy
)

from px4_msgs.msg import VehicleCommand, VehicleStatus


class ArmDisarmNode(Node):

    def __init__(self):
        super().__init__('arm_disarm_node')

        command_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        status_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            command_qos
        )

        self.status_subscriber = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            status_qos
        )

        self.arming_state = None
        self.state = 0
        self.state_entered_time = self.get_clock().now()

        self.confirm_timeout = 5.0
        self.armed_duration = 5.0

        self.timer = self.create_timer(0.2, self.timer_callback)

        self.get_logger().info('Arm/Disarm node started')

    def vehicle_status_callback(self, msg: VehicleStatus):
        self.arming_state = msg.arming_state

    def seconds_in_state(self):
        return (
            self.get_clock().now() -
            self.state_entered_time
        ).nanoseconds / 1e9

    def set_state(self, new_state):
        self.state = new_state
        self.state_entered_time = self.get_clock().now()

    def timer_callback(self):

        if self.arming_state is None:
            return

        if self.state == 0:
            self.get_logger().info('Sending ARM command...')
            self.arm()
            self.set_state(1)

        elif self.state == 1:

            if self.arming_state == VehicleStatus.ARMING_STATE_ARMED:
                self.get_logger().info('Arm confirmed.')
                self.set_state(2)

            elif self.seconds_in_state() > self.confirm_timeout:
                self.get_logger().error(
                    'Arm command not confirmed within timeout.'
                )
                self.set_state(4)

        elif self.state == 2:

            if self.seconds_in_state() > self.armed_duration:
                self.get_logger().info('Sending DISARM command...')
                self.disarm()
                self.set_state(3)

        elif self.state == 3:

            if self.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
                self.get_logger().info('Disarm confirmed.')
                self.get_logger().info(
                    'Arm/Disarm sequence complete.'
                )
                self.set_state(4)

            elif self.seconds_in_state() > self.confirm_timeout:
                self.get_logger().error(
                    'Disarm command not confirmed within timeout.'
                )
                self.set_state(4)

        elif self.state == 4:
            self.timer.cancel()

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

        msg.target_system = 1
        msg.target_component = 1

        msg.source_system = 1
        msg.source_component = 1

        msg.from_external = True

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
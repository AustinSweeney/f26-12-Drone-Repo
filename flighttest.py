#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand
)


class SimpleTakeoffLand(Node):

    def __init__(self):
        super().__init__('simple_takeoff_land')

        # -----------------------------------------------------
        # Publishers
        # -----------------------------------------------------

        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            10
        )

        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            10
        )

        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            10
        )

        # -----------------------------------------------------
        # Mission variables
        # -----------------------------------------------------

        self.counter = 0

        # 10 Hz timer
        self.timer = self.create_timer(
            0.1,
            self.timer_callback
        )

        self.get_logger().info(
            'Simple takeoff/landing node started'
        )

    # =========================================================
    # Main mission
    # =========================================================

    def timer_callback(self):

        # Always publish OffboardControlMode
        self.publish_offboard_control_mode()

        # -----------------------------------------------------
        # 0 - 1 second
        #
        # Send setpoints before requesting Offboard mode.
        # PX4 requires a stream of setpoints before allowing
        # Offboard mode.
        # -----------------------------------------------------

        if self.counter < 10:

            self.publish_position_setpoint(
                0.0,
                0.0,
                0.0
            )

        # -----------------------------------------------------
        # At 1 second
        #
        # Enter Offboard mode and ARM.
        # -----------------------------------------------------

        elif self.counter == 10:

            self.get_logger().info(
                'Entering OFFBOARD mode'
            )

            self.set_offboard_mode()

            self.get_logger().info(
                'Arming drone'
            )

            self.arm()

            # Immediately begin commanding takeoff altitude
            self.publish_position_setpoint(
                0.0,
                0.0,
                -1.0
            )

        # -----------------------------------------------------
        # 1 - 6 seconds
        #
        # Fly upward to 1 meter.
        #
        # PX4 uses NED coordinates:
        #
        # Z negative = UP
        # Z positive = DOWN
        #
        # -1.0 = approximately 1 meter / 3.3 feet up
        # -----------------------------------------------------

        elif self.counter < 60:

            self.publish_position_setpoint(
                0.0,
                0.0,
                -1.0
            )

        # -----------------------------------------------------
        # 6 - 11 seconds
        #
        # Hold at approximately 1 meter.
        # -----------------------------------------------------

        elif self.counter < 110:

            self.publish_position_setpoint(
                0.0,
                0.0,
                -1.0
            )

        # -----------------------------------------------------
        # 11 - 16 seconds
        #
        # Descend back to starting altitude.
        # -----------------------------------------------------

        elif self.counter < 160:

            self.publish_position_setpoint(
                0.0,
                0.0,
                0.0
            )

        # -----------------------------------------------------
        # At 16 seconds
        #
        # DISARM
        # -----------------------------------------------------

        elif self.counter == 160:

            self.get_logger().info(
                'Disarming drone'
            )

            self.disarm()

        # -----------------------------------------------------
        # Stop node mission
        # -----------------------------------------------------

        elif self.counter > 170:

            self.get_logger().info(
                'Mission complete'
            )

            self.timer.cancel()

        self.counter += 1

    # =========================================================
    # Offboard control mode
    # =========================================================

    def publish_offboard_control_mode(self):

        msg = OffboardControlMode()

        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False

        msg.timestamp = (
            self.get_clock().now().nanoseconds // 1000
        )

        self.offboard_control_mode_pub.publish(msg)

    # =========================================================
    # Position setpoint
    # =========================================================

    def publish_position_setpoint(
        self,
        x,
        y,
        z
    ):

        msg = TrajectorySetpoint()

        msg.position = [
            float(x),
            float(y),
            float(z)
        ]

        msg.yaw = 0.0

        msg.timestamp = (
            self.get_clock().now().nanoseconds // 1000
        )

        self.trajectory_setpoint_pub.publish(msg)

    # =========================================================
    # Set PX4 to Offboard mode
    # =========================================================

    def set_offboard_mode(self):

        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=6.0
        )

    # =========================================================
    # ARM
    # =========================================================

    def arm(self):

        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0
        )

    # =========================================================
    # DISARM
    # =========================================================

    def disarm(self):

        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=0.0
        )

    # =========================================================
    # Generic PX4 vehicle command
    # =========================================================

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

        self.vehicle_command_pub.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = SimpleTakeoffLand()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info(
            'Node interrupted'
        )

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
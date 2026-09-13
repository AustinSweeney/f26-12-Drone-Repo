#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleGlobalPosition,
    VehicleLocalPosition,
)


MAX_RADIUS_M = 10.0
STEP_M = 2.0
FLIGHT_ALTITUDE_M = 5.0
SETPOINT_RATE_HZ = 10.0
WAYPOINT_TOLERANCE_M = 0.75
ALTITUDE_TOLERANCE_M = 0.50
RETURN_HOVER_SECONDS = 3.0
WAYPOINT_TIMEOUT_SECONDS = 30.0
OFFBOARD_WARMUP_COUNT = 20


class SquareSpiralDrone(Node):

    # Initializes ROS 2 publishers, subscribers, mission variables, and the control timer.
    def __init__(self):

        super().__init__("square_spiral_drone")

        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode,
            "/fmu/in/offboard_control_mode",
            10,
        )

        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            "/fmu/in/trajectory_setpoint",
            10,
        )

        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            "/fmu/in/vehicle_command",
            10,
        )

        self.global_position_sub = self.create_subscription(
            VehicleGlobalPosition,
            "/fmu/out/vehicle_global_position",
            self.global_position_callback,
            qos_profile_sensor_data,
        )

        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",
            self.local_position_callback,
            qos_profile_sensor_data,
        )

        self.current_lat = None
        self.current_lon = None
        self.current_gps_alt = None

        self.current_x = None
        self.current_y = None
        self.current_z = None

        self.origin_lat = None
        self.origin_lon = None
        self.origin_gps_alt = None

        self.origin_x = None
        self.origin_y = None
        self.origin_z = None

        self.origin_initialized = False

        self.waypoints = []
        self.current_waypoint_index = 0

        self.state = "WAITING_FOR_POSITION"

        self.offboard_counter = 0
        self.waypoint_start_time = None
        self.return_hover_start = None
        self.land_command_sent = False

        timer_period = 1.0 / SETPOINT_RATE_HZ

        self.timer = self.create_timer(
            timer_period,
            self.timer_callback,
        )

        self.get_logger().info(
            "Square Spiral Drone node started."
        )

        self.get_logger().info(
            "Waiting for GPS and local position..."
        )

    # Updates the current GPS latitude, longitude, and altitude.
    def global_position_callback(self, msg):

        self.current_lat = msg.lat
        self.current_lon = msg.lon
        self.current_gps_alt = msg.alt

    # Updates the current PX4 local NED position.
    def local_position_callback(self, msg):

        if hasattr(msg, "xy_valid"):
            if not msg.xy_valid:
                return

        if hasattr(msg, "z_valid"):
            if not msg.z_valid:
                return

        self.current_x = msg.x
        self.current_y = msg.y
        self.current_z = msg.z

    # Records the starting GPS and local position as the mission origin.
    def initialize_origin(self):

        if self.origin_initialized:
            return True

        if self.current_lat is None:
            return False

        if self.current_lon is None:
            return False

        if self.current_x is None:
            return False

        if self.current_y is None:
            return False

        if self.current_z is None:
            return False

        self.origin_lat = self.current_lat
        self.origin_lon = self.current_lon
        self.origin_gps_alt = self.current_gps_alt

        self.origin_x = self.current_x
        self.origin_y = self.current_y
        self.origin_z = self.current_z

        self.origin_initialized = True

        self.waypoints = self.generate_square_spiral()

        self.get_logger().info(
            "\n"
            "========================================\n"
            "MISSION ORIGIN ESTABLISHED\n"
            "========================================\n"
            f"GPS Latitude:  {self.origin_lat:.8f}\n"
            f"GPS Longitude: {self.origin_lon:.8f}\n"
            f"GPS Altitude:  {self.origin_gps_alt:.2f} m\n"
            "\n"
            "PX4 Local NED Origin:\n"
            f"North/X: {self.origin_x:.2f} m\n"
            f"East/Y:  {self.origin_y:.2f} m\n"
            f"Down/Z:  {self.origin_z:.2f} m\n"
            "========================================"
        )

        return True

    # Generates square-spiral waypoints as North/East offsets from the mission origin.
    def generate_square_spiral(self):

        points = []

        directions = [
            (0.0, 1.0),
            (1.0, 0.0),
            (0.0, -1.0),
            (-1.0, 0.0),
        ]

        north = 0.0
        east = 0.0

        direction_index = 0
        segment_length = STEP_M
        stop_generation = False

        while not stop_generation:

            for _ in range(2):

                dn, de = directions[direction_index]

                candidate_north = north + dn * segment_length
                candidate_east = east + de * segment_length

                radius = math.sqrt(
                    candidate_north ** 2
                    + candidate_east ** 2
                )

                if radius > MAX_RADIUS_M:
                    stop_generation = True
                    break

                north = candidate_north
                east = candidate_east

                points.append(
                    (north, east)
                )

                direction_index = (
                    direction_index + 1
                ) % 4

            segment_length += STEP_M

        self.get_logger().info(
            f"Generated {len(points)} spiral waypoints."
        )

        for i, point in enumerate(points):

            north, east = point

            distance = math.sqrt(
                north ** 2
                + east ** 2
            )

            self.get_logger().info(
                f"Waypoint {i + 1}: "
                f"N={north:.2f} m, "
                f"E={east:.2f} m, "
                f"radius={distance:.2f} m"
            )

        return points

    # Calculates the PX4 local Z coordinate for the desired flight altitude.
    def target_z(self):

        return (
            self.origin_z
            - FLIGHT_ALTITUDE_M
        )

    # Converts a North/East spiral offset into an absolute PX4 local position.
    def spiral_offset_to_local(
        self,
        north_offset,
        east_offset,
    ):

        target_x = (
            self.origin_x
            + north_offset
        )

        target_y = (
            self.origin_y
            + east_offset
        )

        return target_x, target_y

    # Publishes the PX4 Offboard control heartbeat for position control.
    def publish_offboard_control_mode(self):

        msg = OffboardControlMode()

        msg.timestamp = self.get_timestamp()

        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False

        if hasattr(msg, "thrust_and_torque"):
            msg.thrust_and_torque = False

        if hasattr(msg, "direct_actuator"):
            msg.direct_actuator = False

        self.offboard_control_mode_pub.publish(msg)

    # Publishes a local NED position setpoint to PX4.
    def publish_position_setpoint(
        self,
        x,
        y,
        z,
        yaw=0.0,
    ):

        msg = TrajectorySetpoint()

        msg.timestamp = self.get_timestamp()

        msg.position = [
            float(x),
            float(y),
            float(z),
        ]

        nan = float("nan")

        msg.velocity = [
            nan,
            nan,
            nan,
        ]

        msg.acceleration = [
            nan,
            nan,
            nan,
        ]

        msg.yaw = float(yaw)
        msg.yawspeed = nan

        self.trajectory_setpoint_pub.publish(msg)

    # Sends a PX4 vehicle command with optional command parameters.
    def publish_vehicle_command(
        self,
        command,
        param1=0.0,
        param2=0.0,
        param3=0.0,
        param4=0.0,
        param5=0.0,
        param6=0.0,
        param7=0.0,
    ):

        msg = VehicleCommand()

        msg.timestamp = self.get_timestamp()

        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.param3 = float(param3)
        msg.param4 = float(param4)
        msg.param5 = float(param5)
        msg.param6 = float(param6)
        msg.param7 = float(param7)

        msg.command = command

        msg.target_system = 1
        msg.target_component = 1

        msg.source_system = 1
        msg.source_component = 1

        msg.from_external = True

        self.vehicle_command_pub.publish(msg)

    # Sends the command to arm the drone.
    def arm(self):

        self.get_logger().info(
            "Sending ARM command."
        )

        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0,
        )

    # Sends the command to switch PX4 into Offboard mode.
    def enter_offboard_mode(self):

        self.get_logger().info(
            "Requesting OFFBOARD mode."
        )

        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=6.0,
        )

    # Sends the PX4 landing command and changes the mission state to landing.
    def land(self):

        if self.land_command_sent:
            return

        self.get_logger().info(
            "Sending PX4 LAND command."
        )

        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_LAND
        )

        self.land_command_sent = True
        self.state = "LANDING"

    # Calculates the horizontal distance from the drone to a target position.
    def horizontal_distance_to(
        self,
        target_x,
        target_y,
    ):

        dx = target_x - self.current_x
        dy = target_y - self.current_y

        return math.sqrt(
            dx ** 2
            + dy ** 2
        )

    # Calculates the vertical distance from the drone to a target altitude.
    def vertical_distance_to(
        self,
        target_z,
    ):

        return abs(
            target_z
            - self.current_z
        )

    # Checks whether the drone is within the allowed tolerance of a target position.
    def target_reached(
        self,
        target_x,
        target_y,
        target_z,
    ):

        horizontal_distance = (
            self.horizontal_distance_to(
                target_x,
                target_y,
            )
        )

        vertical_distance = (
            self.vertical_distance_to(
                target_z
            )
        )

        return (
            horizontal_distance
            <= WAYPOINT_TOLERANCE_M
            and
            vertical_distance
            <= ALTITUDE_TOLERANCE_M
        )

    # Resets the timer used to detect waypoint timeouts.
    def reset_waypoint_timer(self):

        self.waypoint_start_time = (
            self.get_clock().now()
        )

    # Checks whether the current waypoint has exceeded its allowed travel time.
    def waypoint_timed_out(self):

        if self.waypoint_start_time is None:
            return False

        elapsed = (
            self.get_clock().now()
            - self.waypoint_start_time
        ).nanoseconds / 1e9

        return (
            elapsed
            > WAYPOINT_TIMEOUT_SECONDS
        )

    # Runs the mission state machine for takeoff, spiral flight, return, hover, and landing.
    def timer_callback(self):

        if not self.origin_initialized:

            if not self.initialize_origin():
                return

            self.state = "OFFBOARD_WARMUP"

        if self.state != "LANDING":
            self.publish_offboard_control_mode()

        if self.state == "OFFBOARD_WARMUP":

            self.publish_position_setpoint(
                self.origin_x,
                self.origin_y,
                self.target_z(),
            )

            self.offboard_counter += 1

            if (
                self.offboard_counter
                >= OFFBOARD_WARMUP_COUNT
            ):

                self.get_logger().info(
                    "Offboard setpoint warmup complete."
                )

                self.enter_offboard_mode()
                self.arm()

                self.state = "TAKEOFF"

                self.reset_waypoint_timer()

            return

        if self.state == "TAKEOFF":

            target_x = self.origin_x
            target_y = self.origin_y
            target_z = self.target_z()

            self.publish_position_setpoint(
                target_x,
                target_y,
                target_z,
            )

            if self.target_reached(
                target_x,
                target_y,
                target_z,
            ):

                self.get_logger().info(
                    f"Takeoff complete. "
                    f"Altitude approximately "
                    f"{FLIGHT_ALTITUDE_M:.1f} m."
                )

                self.current_waypoint_index = 0
                self.state = "SPIRAL"

                self.reset_waypoint_timer()

                return

            if self.waypoint_timed_out():

                self.get_logger().error(
                    "Takeoff timeout. "
                    "Requesting landing."
                )

                self.land()

            return

        if self.state == "SPIRAL":

            if (
                self.current_waypoint_index
                >= len(self.waypoints)
            ):

                self.get_logger().info(
                    "Square spiral complete."
                )

                self.get_logger().info(
                    "Returning to mission origin."
                )

                self.state = "RETURN_TO_ORIGIN"

                self.reset_waypoint_timer()

                return

            (
                north_offset,
                east_offset,
            ) = self.waypoints[
                self.current_waypoint_index
            ]

            target_x, target_y = (
                self.spiral_offset_to_local(
                    north_offset,
                    east_offset,
                )
            )

            target_z = self.target_z()

            self.publish_position_setpoint(
                target_x,
                target_y,
                target_z,
            )

            if self.target_reached(
                target_x,
                target_y,
                target_z,
            ):

                self.get_logger().info(
                    f"Reached spiral waypoint "
                    f"{self.current_waypoint_index + 1}/"
                    f"{len(self.waypoints)} "
                    f"[N offset={north_offset:.2f}, "
                    f"E offset={east_offset:.2f}]"
                )

                self.current_waypoint_index += 1

                self.reset_waypoint_timer()

                return

            if self.waypoint_timed_out():

                self.get_logger().error(
                    "Spiral waypoint timeout. "
                    "Returning to origin."
                )

                self.state = "RETURN_TO_ORIGIN"

                self.reset_waypoint_timer()

            return

        if self.state == "RETURN_TO_ORIGIN":

            target_x = self.origin_x
            target_y = self.origin_y
            target_z = self.target_z()

            self.publish_position_setpoint(
                target_x,
                target_y,
                target_z,
            )

            horizontal_distance = (
                self.horizontal_distance_to(
                    target_x,
                    target_y,
                )
            )

            self.get_logger().debug(
                f"Distance to origin: "
                f"{horizontal_distance:.2f} m"
            )

            if self.target_reached(
                target_x,
                target_y,
                target_z,
            ):

                self.get_logger().info(
                    "Drone has returned to origin."
                )

                self.get_logger().info(
                    f"Hovering for "
                    f"{RETURN_HOVER_SECONDS:.1f} seconds "
                    f"before landing."
                )

                self.return_hover_start = (
                    self.get_clock().now()
                )

                self.state = "HOVER_AT_ORIGIN"

                return

            if self.waypoint_timed_out():

                self.get_logger().error(
                    "Return-to-origin timeout. "
                    "Requesting landing."
                )

                self.land()

            return

        if self.state == "HOVER_AT_ORIGIN":

            target_x = self.origin_x
            target_y = self.origin_y
            target_z = self.target_z()

            self.publish_position_setpoint(
                target_x,
                target_y,
                target_z,
            )

            elapsed = (
                self.get_clock().now()
                - self.return_hover_start
            ).nanoseconds / 1e9

            if elapsed >= RETURN_HOVER_SECONDS:

                self.get_logger().info(
                    "Origin hover complete."
                )

                self.land()

            return

        if self.state == "LANDING":
            return

    # Converts the ROS 2 clock timestamp from nanoseconds to PX4 microseconds.
    def get_timestamp(self):

        return (
            self.get_clock()
            .now()
            .nanoseconds
            // 1000
        )


# Starts the ROS 2 node and keeps it running until shutdown.
def main(args=None):

    rclpy.init(args=args)

    node = SquareSpiralDrone()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:

        node.get_logger().warn(
            "Mission node stopped by user."
        )

    finally:

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
F26-12 Drone Repo README

The environment is setup for
Ubuntu 22.04
ROS 2 Humble
PX4 Autopilot
SITL
Gazebo
Micro XRCE-DDS Agent
'px4_msgs'
Python 3
rclpy


For the arm/disarm confirmation the PX4 uses fmu/out/vehicle_status_v2
the ROS 2 message type is px4_msgs/msg/VehicleStatus

If you're using an older version of PX4 or different px4_msgs use:

self.status_subscriber = self.create_subscription(
    VehicleStatus,
    '/fmu/out/vehicle_status',
    self.vehicle_status_callback,
    status_qos
)

instead

If you need to figure out what vehicle status topic to use run the command:
ros2 topic list | grep vehicle_status



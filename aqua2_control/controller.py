#! /usr/bin/env python3
import time
from enum import Enum
from collections import deque

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from ir_utils.param_tools import get_namespace

# Messages
from aqua2_interfaces.msg import Waypoint, AutopilotCommand, RobodevelState, Distance
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray

# Services
from std_srvs.srv import Empty, Trigger
from aqua2_interfaces.srv import SetString, SetInt
from robot_localization.srv import SetPose


class ap_modes(Enum):
    off = 0
    angles = 2
    depth = 4


class modes(Enum):
    safemode = 0
    swimmode = 4


class Controller(Node):
    """Aqua2 controller with a MAVROS-style interface."""

    def __init__(self):
        super().__init__("aqua2_controller", namespace=get_namespace())
        self.get_logger().info("Initializing AquaController.")

        # ROS param configurable defaults
        self.declare_parameter("depth", 0.5)
        self.declare_parameter("speed", 0.5)
        self.declare_parameter("acceptance_radius", 1.1)

        self.depth = float(self.get_parameter("depth").value)
        self.speed = float(self.get_parameter("speed").value)
        self.vehicle_position = np.array([0., 0., 0.])

        # State & navigation-related fields
        self.is_calibrated = False
        self.mode = -1
        self.ap_mode = -1
        self.acceptance_radius = 1.5  # temporary, will be set below

        # Distance-to-waypoint state (fed by Distance topic)
        self.distance_to_target = float("inf")
        self.target_x = None
        self.target_y = None

        # Velocity estimation (used for ETA)
        self.velocity = 0.01
        self.velocity_buffer = deque([0.01])

        # Subscribers
        self.create_subscription(
            RobodevelState,
            "system/state",
            self.rd_state_callback,
            1,
        )
        self.create_subscription(
            Distance,
            "navigation/distance_to_wp",
            self.distance_callback,
            1,
        )
        self.create_subscription(
            Odometry,
            "navigation/local_position",
            self.vehicle_odom_callback,
            1,
        )

        # Publishers
        self.cmd_req_pub = self.create_publisher(
            AutopilotCommand,
            "navigation/command_request",
            1,
        )
        self.local_wp_pub = self.create_publisher(
            Waypoint,
            "navigation/navigate_waypoint",
            1,
        )
        self.eta_publisher = self.create_publisher(
            Float32MultiArray,
            "/waypoint_eta",
            qos_profile_sensor_data,
        )

        # Service Clients
        self.calib_client = self.create_client(Empty, "system/calibrate")
        self.mode_client = self.create_client(SetString, "system/set_mode")
        self.ap_mode_client = self.create_client(SetInt, "autopilot/set_autopilot_mode")
        self.local_pose_client = self.create_client(SetPose, "set_pose")
        self.reset_imu_client = self.create_client(Empty, "imu/zero_heading")
        self.reset_dvl_client = self.create_client(Trigger, "dvl/reset_odometry")

        self.robot_initialized = False

    def init_robot(self):
        if not self.robot_initialized:
            # Initialization sequence
            self.calibrate()
            self.set_mode("swimmode")
            self.set_autopilot_mode("depth")
            self.set_acceptance_radius(
                float(self.get_parameter("acceptance_radius").value)
            )
            self.zero_dvl()
            self.zero_local_pose()
            self.zero_heading()
        self.robot_initialized = True

    # ------------------- Velocity / Odom -------------------

    def vehicle_odom_callback(self, msg: Odometry):
        """Callback for odom: computes nominal planar linear velocity."""
        self.vehicle_position[0] = msg.pose.pose.position.x
        self.vehicle_position[1] = msg.pose.pose.position.y
        self.vehicle_position[2] = msg.pose.pose.position.z

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        velocity = np.hypot(vx, vy)

        if len(self.velocity_buffer) > 50:
            self.velocity_buffer.popleft()
        if velocity > 0.2:
            self.velocity_buffer.append(velocity)

        self.velocity = float(np.mean(self.velocity_buffer))

    # ------------------- Shutdown -------------------

    def shutdown(self):
        """Cleanly shut down rclpy."""
        rclpy.shutdown()

    # ------------------- State / Distance Callbacks -------------------

    def rd_state_callback(self, msg: RobodevelState):
        self.is_calibrated = msg.calibrated
        self.mode = msg.mode
        self.ap_mode = msg.ap_mode

    def distance_callback(self, msg: Distance):
        """Update distance to current target waypoint (from navigation node)."""
        # Only update if the distance is for our active target
        if np.linalg.norm([msg.target_x - self.target_x, msg.target_y - self.target_y]) < 1e-3:
            self.distance_to_target = msg.distance

    # ------------------- Navigation Core -------------------

    def set_acceptance_radius(self, radius: float):
        if radius < 1.0:
            self.get_logger().warn(
                f"Radius {radius} is too small. Keeping previous value ({self.acceptance_radius})."
            )
        else:
            self.acceptance_radius = float(radius)

    def at_waypoint(self, _waypoint) -> bool:
        """Waypoint arg kept for compatibility."""
        return self.distance_to_target < self.acceptance_radius

    def go2waypoint(self, waypoint, timeout: float = 900.0) -> bool:
        """Go to a local-frame waypoint, publishing ETA.

        Args:
            waypoint (list/ndarray): [x, y] or [x, y, depth] in the local frame.
            timeout (float): Maximum time (s) to keep trying to reach the waypoint.
        """
        self.init_robot() # Ensure the robot is initialized
        waypoint_arr = np.asarray(waypoint, dtype=float).flatten()
        if waypoint_arr.size < 2:
            self.get_logger().error(
                f"go2waypoint requires at least x and y, got {waypoint_arr}"
            )
            return False

        # Aqua's user-facing local frame (x, y) ->
        # internal navigation frame expects swapped axes (x <- y, y <- x).
        self.target_x = float(waypoint_arr[0])
        self.target_y = float(waypoint_arr[1])
        depth = float(waypoint_arr[2]) if waypoint_arr.size >= 3 else self.depth
        self.distance_to_target = float("inf")

        self.get_logger().info(
            f"Going to waypoint (local): x={self.target_x}, y={self.target_y}, depth={depth}, "
            f"speed={self.speed}, acceptance_radius={self.acceptance_radius}"
        )

        # Publish waypoint command
        wp_msg = Waypoint()
        wp_msg.coord_frame = 0
        wp_msg.target_x = self.target_x
        wp_msg.target_y = self.target_y
        wp_msg.target_depth = depth
        wp_msg.speed = self.speed
        
        clock = self.get_clock()
        start_time = clock.now().to_msg().sec

        while not self.at_waypoint(waypoint_arr):
            now = clock.now().to_msg().sec
            self.local_wp_pub.publish(wp_msg)

            # Publish ETA using distance from navigation/distance_to_wp
            dist = float(self.distance_to_target)
            if np.isfinite(dist) and dist >= 0.0:
                velocity = max(self.velocity, 1e-3)
                eta = dist / velocity
                eta_msg = Float32MultiArray(
                    data=[float(eta), dist, float(velocity)]
                )
                self.eta_publisher.publish(eta_msg)

            # Timeout check
            if (now - start_time) > timeout:
                self.get_logger().info(
                    f"Timeout: Failed to go to waypoint x={self.target_x}, y={self.target_y}"
                )
                return False

            # Let callbacks process sensor / distance updates at ~10Hz
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info(
            f"Waypoint reached: x={self.target_x}, y={self.target_y}, depth={depth}"
        )
        return True

    # ------------------- "RC-style" command API -------------------

    def pub_rc_override(
        self,
        cmd,
        timeout: float = 0.0,
        publisher_period_sec: float = 0.03,
    ) -> bool:
        """Aqua-specific implementation of a MAVROS-like pub_rc_override interface.

        Args:
            cmd (list): [Forward, Heave (Up/Down), Yaw, Pitch, Roll, Depth]
                        Forward & Heave in [-1.0, 1.0]; others in appropriate units.
            timeout (float): Number of seconds to keep re-publishing the command.
            publisher_period_sec (float): Period between republishes while timeout > 0.
        """
        if cmd is None or len(cmd) < 6:
            self.get_logger().error(
                "pub_rc_override expects cmd with at least 6 elements: "
                "[Forward, Heave, Yaw, Pitch, Roll, Depth]"
            )
            return False

        def _norm_unit(v: float) -> float:
            v = max(min(float(v), 1.0), -1.0)
            # map [-1, 1] -> [0, 1]
            return 0.5 * (v + 1.0)

        surge = _norm_unit(cmd[0])
        heave = _norm_unit(cmd[1])
        yaw = float(cmd[2])
        pitch = float(cmd[3])
        roll = float(cmd[4])
        depth = float(cmd[5])

        msg = AutopilotCommand()
        msg.surge = surge
        msg.heave = heave
        msg.target_yaw = yaw
        msg.target_pitch = pitch
        msg.target_roll = roll
        msg.target_depth = depth

        self.cmd_req_pub.publish(msg)

        if timeout <= 0.0:
            return True

        clock = self.get_clock()
        start_time = clock.now().to_msg().sec

        # If timeout > 0, continue publishing at the given period
        while clock.now().to_msg().sec - start_time < timeout:
            rclpy.spin_once(self, timeout_sec=publisher_period_sec)
            self.cmd_req_pub.publish(msg)

        return True

    # ------------------- Initialization / Services -------------------

    def zero_local_pose(self, timeout: float = 5.0) -> bool:
        self.get_logger().info("Zeroing local pose...")
        if not self.local_pose_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(
                "Zero local pose failed! Cannot find service server."
            )
            return False

        future = self.local_pose_client.call_async(SetPose.Request())
        start_time = time.time()

        while not future.done():
            if time.time() - start_time >= timeout:
                self.get_logger().error(
                    "Zero local pose failed! Timed out before zeroing finished."
                )
                return False
            rclpy.spin_once(self, timeout_sec=0.1)

        return True

    def zero_dvl(self, timeout: float = 5.0) -> bool:
        self.get_logger().info("Zeroing dvl...")
        if not self.reset_dvl_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(
                "Zero dvl failed! Cannot find service server."
            )
            return False

        future = self.reset_dvl_client.call_async(Trigger.Request())
        start_time = time.time()

        while not future.done():
            if time.time() - start_time >= timeout:
                self.get_logger().error(
                    "Zero dvl failed! Timed out before zeroing finished."
                )
                return False
            rclpy.spin_once(self, timeout_sec=0.1)

        return True

    def zero_heading(self, timeout: float = 5.0) -> bool:
        self.get_logger().info("Zeroing heading...")
        if not self.reset_imu_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(
                "Zero heading failed! Cannot find service server."
            )
            return False

        future = self.reset_imu_client.call_async(Empty.Request())
        start_time = time.time()

        while not future.done():
            if time.time() - start_time >= timeout:
                self.get_logger().error(
                    "Zero heading failed! Timed out before heading zero finished."
                )
                return False
            rclpy.spin_once(self, timeout_sec=0.1)

        return True

    def calibrate(self, timeout: float = 15.0) -> bool:
        if self.is_calibrated:
            self.get_logger().info(
                "AquaController not calling calibrate, robot is already calibrated."
            )
            return True

        self.get_logger().info("AquaController calibrating.")
        if not self.calib_client.wait_for_service():
            self.get_logger().error(
                "Calibrate failed! Cannot find service server."
            )
            return False

        future = self.calib_client.call_async(Empty.Request())
        start_time = time.time()

        while (not future.done()) or (not self.is_calibrated):
            if time.time() - start_time >= timeout:
                self.get_logger().error(
                    "Calibration failed! Timed out before calibration completed."
                )
                return False
            rclpy.spin_once(self, timeout_sec=0.1)

        time.sleep(5)
        return True

    def set_mode(self, mode: str, timeout: float = 10.0) -> bool:
        if mode not in modes.__members__:
            self.get_logger().error(
                f"AquaController failed to set mode, invalid mode: {mode}"
            )
            return False

        self.get_logger().info(f"AquaController setting mode to: {mode}")
        req = SetString.Request()
        req.value = mode
        future = self.mode_client.call_async(req)
        start_time = time.time()

        while (not future.done()) or (modes[mode].value != self.mode):
            if time.time() - start_time >= timeout:
                self.get_logger().error(
                    "Set Mode Failed! Timed out before Set Mode finished."
                )
                return False
            rclpy.spin_once(self, timeout_sec=0.1)

        return True

    def set_autopilot_mode(self, mode: str, timeout: float = 10.0) -> bool:
        if mode not in ap_modes.__members__:
            self.get_logger().error(
                f"AquaController failed to set autopilot mode, invalid mode: {mode}"
            )
            return False

        self.get_logger().info(f"AquaController setting autopilot mode to: {mode}")
        req = SetInt.Request()
        req.value = ap_modes[mode].value
        future = self.ap_mode_client.call_async(req)
        start_time = time.time()

        while (not future.done()) or (self.ap_mode != ap_modes[mode].value):
            if time.time() - start_time >= timeout:
                self.get_logger().error(
                    "Set AutoPilot Mode Failed! Timed out before Set AutoPilot Mode finished."
                )
                return False
            rclpy.spin_once(self, timeout_sec=0.1)

        return True

    # ------------------- Demo mission -------------------

    def mission(self):
        waypoints = [
            [10.1, 0.1],
            [10.0, 2.0],
            [0.0, 2.0],
            [0.0, 4.0],
            [10.1, 4.2],
            [10.0, 6.0],
            [0.0, 6.0],
        ]

        for i, wp in enumerate(waypoints):
            self.get_logger().info(f"Visiting waypoint {i}")
            self.go2waypoint(wp)
        self.get_logger().info("Mission Complete!")


def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    try:
        node.mission()
    finally:
        node.shutdown()


if __name__ == "__main__":
    main()

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from ff_interface.srv import RobotControl
import math

class Navigation:
    def __init__(self, node: Node):
        self.node = node
        
        # Publisher for robot velocity control
        self.cmd_vel_pub = node.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriber for robot position (e.g., from localization or ArUCo detection)
        self.current_pose = None
        self.pose_sub = node.create_subscription(
            PoseStamped, '/current_pose', self.pose_callback, 10
        )

        # Service server for manual robot control
        self.control_srv = node.create_service(
            RobotControl, '/RobotControl', self.manual_control_callback
        )

    def pose_callback(self, msg: PoseStamped):
        """Callback to update the robot's current position."""
        self.current_pose = msg.pose

    async def go_to(self, relative_x, relative_y, angle_between, target_x, target_y):
        """
        현재 위치 (relative_x, relative_y, angle_between)와 목표 위치 (target_x, target_y)를 기반으로 이동
        """
        # 목표와의 거리 및 각도 계산
        delta_x = target_x - relative_x
        delta_y = target_y - relative_y
        distance_to_target = math.sqrt(delta_x ** 2 + delta_y ** 2)
        angle_to_target = math.atan2(delta_y, delta_x)

        # 현재 방향과 목표 방향의 각도 차이
        angle_diff = angle_to_target - angle_between

        # 회전
        while abs(angle_diff) > 0.1:  # 허용 오차 0.1 라디안
            twist = Twist()
            twist.angular.z = 0.3 if angle_diff > 0 else -0.3
            self.cmd_vel_pub.publish(twist)
            self.node.get_logger().info(f"Adjusting angle: {angle_diff:.2f}")
            rclpy.spin_once(self.node, timeout_sec=0.1)
            angle_diff = angle_to_target - angle_between  # Update after rotation

        # 직선 이동
        while distance_to_target > 0.05:  # 허용 오차 0.05 미터
            twist = Twist()
            twist.linear.x = 0.2
            self.cmd_vel_pub.publish(twist)
            self.node.get_logger().info(f"Moving straight: distance={distance_to_target:.2f}")
            rclpy.spin_once(self.node, timeout_sec=0.1)

            # Update relative position
            delta_x = target_x - relative_x
            delta_y = target_y - relative_y
            distance_to_target = math.sqrt(delta_x ** 2 + delta_y ** 2)

        # 정지
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        self.node.get_logger().info("Goal reached.")


    def compute_distance(self, current_pose, target_pose):
        """
        Compute the Euclidean distance between the current and target pose.

        :param current_pose: Current pose as geometry_msgs/Pose
        :param target_pose: Target pose as geometry_msgs/Pose
        :return: Distance as a float
        """
        dx = target_pose.position.x - current_pose.position.x
        dy = target_pose.position.y - current_pose.position.y
        return (dx**2 + dy**2)**0.5

    def compute_velocity(self, current_pose, target_pose):
        """
        Compute a Twist message to navigate towards the target pose.

        :param current_pose: Current pose as geometry_msgs/Pose
        :param target_pose: Target pose as geometry_msgs/Pose
        :return: Twist message
        """
        twist = Twist()
        dx = target_pose.position.x - current_pose.position.x
        dy = target_pose.position.y - current_pose.position.y
        angle_to_target = atan2(dy, dx)

        # Example simple proportional control
        twist.linear.x = min(0.2, max(0.05, self.compute_distance(current_pose, target_pose) * 0.5))
        twist.angular.z = angle_to_target
        return twist

    def stop(self):
        """Stop the robot."""
        self.node.get_logger().info("Stopping the robot...")
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

    def manual_control_callback(self, request, response):
        """
        Callback for /RobotControl service to manually control the robot.

        :param request: RobotControl.Request
        :param response: RobotControl.Response
        :return: RobotControl.Response
        """
        self.node.get_logger().info(f"Received manual control command: {request.command}")
        if request.command == "stop":
            self.stop()
            response.success = True
        elif request.command == "forward":
            twist = Twist()
            twist.linear.x = 0.2
            self.cmd_vel_pub.publish(twist)
            response.success = True
        elif request.command == "backward":
            twist = Twist()
            twist.linear.x = -0.2
            self.cmd_vel_pub.publish(twist)
            response.success = True
        elif request.command == "turn_left":
            twist = Twist()
            twist.angular.z = 0.5
            self.cmd_vel_pub.publish(twist)
            response.success = True
        elif request.command == "turn_right":
            twist = Twist()
            twist.angular.z = -0.5
            self.cmd_vel_pub.publish(twist)
            response.success = True
        else:
            response.success = False
            response.message = "Unknown command"
        return response

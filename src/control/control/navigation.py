import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from ff_interface.srv import RobotControl

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

    async def go_to(self, relative_x, relative_y, angle_between):
        """
        상대 좌표와 각도를 기반으로 로봇을 이동
        """
        # 현재 각도 차이를 보정
        while abs(angle_between) > 0.1:  # 허용 오차 0.1 라디안
            twist = Twist()
            twist.angular.z = 0.3 if angle_between > 0 else -0.3
            self.cmd_vel_pub.publish(twist)
            self.node.get_logger().info(f"Adjusting angle: {angle_between:.2f}")
            rclpy.spin_once(self.node, timeout_sec=0.1)

        # 직선 이동
        distance = math.sqrt(relative_x ** 2 + relative_y ** 2)
        while distance > 0.05:  # 허용 오차 0.05 미터
            twist = Twist()
            twist.linear.x = 0.2
            self.cmd_vel_pub.publish(twist)
            self.node.get_logger().info(f"Moving straight: distance={distance:.2f}")
            rclpy.spin_once(self.node, timeout_sec=0.1)
            distance = math.sqrt(relative_x ** 2 + relative_y ** 2)

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

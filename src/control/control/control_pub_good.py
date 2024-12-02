import rclpy
import time
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image, '/image_raw', self.listener_callback, 10
        )
        self.bridge = CvBridge()
        self.model = YOLO('/home/rokey1/Gripper_Camera_jetsonNano_ws/src/image_sub/image_sub/image_subscriber/cat_box.pt')  # YOLOv8 모델 로드 (사전 훈련된 모델 사용)
        #self.model = YOLO('/home/kimdahun/Gripper_Camera_jetsonNano_ws/src/image_sub/image_sub/image_subscriber/cat_box.pt')
        self.feedback_publisher = self.create_publisher(String, '/feedback_topic', 10)
        self.detected_image_publisher=self.create_publisher(Image, '/detected_image', 10)
        self.captured_image_publisher=self.create_publisher(Image, '/cpatured_image', 10)
        self.last_processed_time = time.time()

    def listener_callback(self, msg):
        current_time = time.time()
        if current_time - self.last_processed_time < 0.5:
            return
        self.last_processed_time = current_time

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(cv_image)
        detected_message=self.bridge.cv2_to_imgmsg(cv_image,encoding='bgr8')
        self.detected_image_publisher.publisher(detected_message)

        if len(results[0].boxes) > 0:
            frame_with_boxes = results[0].plot()
            image_height, image_width, _ = frame_with_boxes.shape
            center_x = image_width // 2
            center_y = image_height // 2

            boxes = results[0].boxes.xyxy.cpu().numpy()
            for i, box in enumerate(boxes):
                xmin, ymin, xmax, ymax = map(int, box)
                box_center_x = int((xmin + xmax) / 2)
                box_center_y = int((ymin + ymax) / 2)

                if center_x - 60 <= box_center_x <= center_x + 40:
                    self.get_logger().info(f"Box {i + 1}: box_center_x in green zone")
                    captured_message=self.bridge.cv2_to_imgmsg(cv_image,encoding='bgr8')
        	    self.captured_image_publisher.publisher(captured_message)

                else:
                    feedback_msg = String()
                    feedback_msg.data = f"Box {i + 1}: Adjust left/right alignment!"
                    self.feedback_publisher.publish(feedback_msg)
                    self.get_logger().info(f"Published feedback: {feedback_msg.data}")

                if center_y + 40 <= box_center_y <= center_y + 210:
                    self.get_logger().info(f"Box {i + 1}: box_center_y in green zone")
                else:
                    feedback_msg = String()
                    feedback_msg.data = f"Box {i + 1}: Adjust up/down alignment!"
                    self.feedback_publisher.publish(feedback_msg)
                    self.get_logger().info(f"Published feedback: {feedback_msg.data}")

    def run(self):
        try:
            rclpy.spin(self)
        except KeyboardInterrupt:
            self.get_logger().info("Shutting down Image Subscriber Node.")
        finally:
            self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    try:
        node.run()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
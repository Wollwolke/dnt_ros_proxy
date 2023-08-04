import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

PATH = "../../../../compression_test/data/images/"
cnt = 0

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, '/detectedImages', 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(1, self.publish_image)

    def publish_image(self):
        global cnt
        if cnt < 10:
            img = cv2.imread(f"{PATH}image{cnt}.jpg")
            cnt += 1
            # Convert BGR image to RGB
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            img_msg = self.bridge.cv2_to_imgmsg(img, encoding='rgb8')
            img_msg.header.frame_id = "wolke"
            img_msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher_.publish(img_msg)
        else:
            pass

def main(args=None):
    rclpy.init(args=args)
    publisher = ImagePublisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

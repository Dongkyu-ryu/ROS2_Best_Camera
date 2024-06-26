import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CannyPublisher(Node):
    def __init__(self):
        super().__init__('canny_publisher')

        self.declare_parameter('camera_topic', '/camera')

        self.declare_parameter('thrs1', 2000)
        self.thrs1 = self.get_parameter('thrs1').value
        self.declare_parameter('thrs2', 4000)
        self.thrs2 = self.get_parameter('thrs2').value

        self.subscriber = self.create_subscription(Image, '/camera', self.image_callback, 10)

        self.publisher = self.create_publisher(Image, '/canny', 10)

        self.cv_bridge = CvBridge()

        
    def image_callback(self, msg):
        img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        filter_img = self.filter(img)

        pub_img = self.cv_bridge.cv2_to_imgmsg(filter_img, "mono8")

        self.publisher.publish(pub_img)

    def filter(self, img):

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        edge = cv2.Canny(gray, self.thrs1, self.thrs2, apertureSize=5)

        return edge


def main() :
    rclpy.init()
    node = CannyPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__' :
    main()
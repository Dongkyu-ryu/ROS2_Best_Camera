
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class NoisePublisher(Node):
    def __init__(self):
        super().__init__('noise_publisher')

        self.declare_parameter('camera_topic', '/camera')

        self.declare_parameter('salt_param', 240)
        self.salt_param = self.get_parameter('salt_param').value
        self.declare_parameter('pepper_param', 5)
        self.pepper_param = self.get_parameter('pepper_param').value

        self.subscriber = self.create_subscription(Image, '/camera', self.image_callback, 10)

        self.publisher = self.create_publisher(Image, '/noise', 10)
        self.cv_bridge = CvBridge()

    def image_callback(self, msg):
        img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        noise_img = self.generate_noise(img)
        noisy_img = cv2.cvtColor(noise_img, cv2.COLOR_BGR2RGB)

        pub_img = self.cv_bridge.cv2_to_imgmsg(noisy_img, "rgb8")
        self.publisher.publish(pub_img)


    def generate_noise(self, img):
        noise = np.random.randint(0, 256, img.shape, dtype=np.uint8)

        salt = noise > self.salt_param
        pepper = noise < self.pepper_param

        noise[salt] = 255
        noise[pepper] = 0

        noise_img = cv2.add(img, noise)

        return noise_img


def main():
    rclpy.init()
    node = NoisePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

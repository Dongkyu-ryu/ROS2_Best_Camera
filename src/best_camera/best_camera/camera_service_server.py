import rclpy
from rclpy.node import Node
from best_camera_msgs.srv import CaptureFrame, RecordFrame
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime

class Capture(Node):
    def __init__(self):
        super().__init__('screen_capture')
        self.bridge = CvBridge()
        self.images = {}
        self.video_writers = {}
        self.video_paths = {}
        
        topics = ['/camera', '/noise', '/canny']
        for topic in topics:
            self.create_subscription(Image, topic, self.callback(topic), 10)
                
        self.capture_service = self.create_service(CaptureFrame, 'capture', self.capture_request)
        self.record_service = self.create_service(RecordFrame, 'record', self.record_request)

    def record_frame(self, topic, image):
        cv_image = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
        self.video_writers[topic].write(cv_image)

    def callback(self, topic):
        def img_callback(image):
            self.images[topic] = image
            if topic in self.video_writers:
                self.record_frame(topic, image)
        return img_callback

    def capture_request(self, request, response):
        image = self.images.get(request.topic_name, None)
        file_path = './captured_images/'
        os.makedirs(file_path, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'captured_{request.topic_name.replace("/", "_")}_{timestamp}.jpg'
        
        if image is None:
            response.success = False
            response.message = f"No image received from {request.topic_name}."
            return response
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
            cv2.imwrite(os.path.join(file_path, filename), cv_image)
            response.success = True
            response.message = f'Video captured and saved at {file_path}'
            return response
        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f'Failed to capture and save image: {str(e)}')


    def record_request(self, request, response):
        topic = request.topic_name
        if request.start:
            if topic in self.video_writers:
                response.success = False
                response.message = "Recording is already started for this topic."
            else:
                file_path = './videos/'
                os.makedirs(file_path, exist_ok=True)

                timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
                file_name = f'video_{topic.replace("/", "_")}_{timestamp}.avi'
                fourcc = cv2.VideoWriter_fourcc(*'XVID')

                self.video_writers[topic] = cv2.VideoWriter(os.path.join(file_path, file_name), fourcc, 20.0, (480, 320))
                self.video_paths[topic] = os.path.join(file_path, file_name)

                response.success = True
                response.message = f"Recording started for {topic} at {self.video_paths[topic]}"
        else:
            if topic in self.video_writers:
                self.video_writers[topic].release()
                del self.video_writers[topic]
                response.success = True
                response.message = f"Recording stopped and saved at {self.video_paths[topic]}"
                del self.video_paths[topic]
            else:
                response.success = False
                response.message = "No recording found to stop for this topic."
        return response

def main(args=None):
    rclpy.init(args=args)
    camera = Capture()
    rclpy.spin(camera)
    camera.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
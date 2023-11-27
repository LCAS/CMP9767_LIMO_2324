# Python libs
import rclpy
from rclpy.node import Node
from rclpy import qos

# OpenCV
import cv2

# ROS libraries
import image_geometry

# ROS Messages
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

class ImageProjection(Node):
    camera_model = None

    def __init__(self):
        super().__init__('image_projection_1')
        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(Image, '/limo/depth_camera_link/image_raw', 
                                                  self.image_callback, qos_profile=qos.qos_profile_sensor_data)
        
        self.camera_info_sub = self.create_subscription(CameraInfo, '/limo/depth_camera_link/camera_info',
                                                self.camera_info_callback, 
                                                qos_profile=qos.qos_profile_sensor_data)

    def image_callback(self, data):
        # wait for the camera_info message first
        if not self.camera_model:
            return
        
        # project a point in camera coordinates (-1.0 : 1.0) into the pixel coordinates
        uv = self.camera_model.project3dToPixel((0.0, 0.0, 1.0))

        print('Pixel coordinates: ', uv)

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        cv2.circle(cv_image, (int(uv[0]),int(uv[1])), 10, 255, -1)

        #resize for visualisation
        cv_image_s = cv2.resize(cv_image, (0,0), fx=0.5, fy=0.5)

        cv2.imshow("Image window", cv_image_s)
        cv2.waitKey(1)

    def camera_info_callback(self, data):
        if not self.camera_model:
            self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(data)

def main(args=None):
    rclpy.init(args=args)
    image_projection = ImageProjection()
    rclpy.spin(image_projection)
    image_projection.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

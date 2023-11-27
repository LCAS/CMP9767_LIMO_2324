# Python libs
import rclpy
from rclpy.node import Node
from rclpy import qos

# OpenCV
import cv2

# ROS libraries
import image_geometry
from tf2_ros import Buffer, TransformListener

# ROS Messages
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
from tf2_geometry_msgs import do_transform_pose

class ImageProjection(Node):
    camera_model = None

    def __init__(self):
        super().__init__('image_projection_2')
        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(Image, '/limo/depth_camera_link/image_raw', 
                                                  self.image_callback, qos_profile=qos.qos_profile_sensor_data)
        
        self.camera_info_sub = self.create_subscription(CameraInfo, '/limo/depth_camera_link/camera_info',
                                                self.camera_info_callback, 
                                                qos_profile=qos.qos_profile_sensor_data)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def get_tf_transform(self, target_frame, source_frame):
        try:
            transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
            return transform
        except Exception as e:
            self.get_logger().warning(f"Failed to lookup transform: {str(e)}")
            return None


    def image_callback(self, data):
        if not self.camera_model:
            return

        #show the camera pose with respect to the robot's pose (base_link)
        transform = self.get_tf_transform('depth_link', 'base_link')
        if transform:
            print('Robot to camera transform:', 'T ', transform.transform.translation, 'R ', transform.transform.rotation)
        else:
            return

        #define a point in robot (base_link) coordinates
        p_robot = PoseStamped()
        p_robot.header.frame_id = "base_link"
        p_robot.pose.orientation.w = 1.0
        # specify a point 5 m in front of the robot (centre, ground)
        # base_link is 0.145 m above the ground
        p_robot.pose.position.x = 5.0
        p_robot.pose.position.y = 0.0
        p_robot.pose.position.z = -0.145
        p_camera = do_transform_pose(p_robot.pose, transform)
        print('Point in the camera coordinates')
        print(p_camera.position)        

        uv = self.camera_model.project3dToPixel((p_camera.position.x,p_camera.position.y,
            p_camera.position.z))

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

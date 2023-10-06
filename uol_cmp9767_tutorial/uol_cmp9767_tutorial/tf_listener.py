import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from tf2_geometry_msgs import do_transform_pose
import geometry_msgs.msg

class TFListener(Node):
    def __init__(self):
        super().__init__('tf_listener')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pose_pub = self.create_publisher(geometry_msgs.msg.PoseStamped, "test_pose", 10)

    def get_tf_transform(self, target_frame, source_frame):
        try:
            transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
            return transform
        except Exception as e:
            self.get_logger().warning(f"Failed to lookup transform: {str(e)}")
            return None


def main(args=None):
    rclpy.init(args=args)
    tf_listener = TFListener()

    while rclpy.ok():
        transform = tf_listener.get_tf_transform('depth_camera_link', 'base_link')
        if transform:
            # Extract the necessary data from the transform object
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            child_frame_id = transform.child_frame_id
            header = transform.header

            # Print the received transformation information
            tf_listener.get_logger().info(
                f"Received TF Transform:\n"
                f"  Child frame ID: {child_frame_id}\n"
                f"  Translation: [x: {translation.x}, y: {translation.y}, z: {translation.z}]\n"
                f"  Rotation: [x: {rotation.x}, y: {rotation.y}, z: {rotation.z}, w: {rotation.w}]\n"
                f"  Frame ID: {header.frame_id}\n"
                f"  Timestamp: {header.stamp.sec}.{header.stamp.nanosec}")
            
            # Here is an example pose, with a given a frame of reference, e.g., something detected in the camera
            p1 = geometry_msgs.msg.PoseStamped()
            p1.header.frame_id = "depth_camera_link"
            p1.pose.orientation.w = 1.0 # Neutral orientation
            p1.pose.position.z = 0.5 # half a metre away from the centre of the frame
            # we publish this so we can see it in rviz
            tf_listener.pose_pub.publish(p1)

            # here we directly transofrm the pose into another pose for the given frame of reference
            p_in_base = do_transform_pose(p1.pose, transform)
            print("Position of the object in the new frame of reference: \n", p_in_base)

        rclpy.spin_once(tf_listener)

    tf_listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

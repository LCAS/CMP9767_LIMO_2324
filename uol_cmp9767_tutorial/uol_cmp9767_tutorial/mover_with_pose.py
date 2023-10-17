import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from tf2_geometry_msgs import do_transform_pose
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from math import cos, sin

class Mover(Node):
    """
    A very simple Roamer implementation for LIMO.
    It simply goes straight until any obstacle is within
    2 m distance and then just simply turns left.
    A purely reactive approach.
    """
    def __init__(self):
        """
        On construction of the object, create a Subscriber
        to listen to lasr scans and a Publisher to control
        the robot
        """
        super().__init__('tf_listener')
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pose_pub = self.create_publisher(PoseStamped, "/nearest_obstacle", 10)
        self.subscriber = self.create_subscription(LaserScan, "/scan", self.laserscan_callback, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
    
    def laserscan_callback(self, data):
        """
        Callback called any time a new laser scan become available
        """
        
        # Find the closest of all laser readings
        min_dist = min(data.ranges[int(len(data.ranges)/2) -10 : int(len(data.ranges)/2) +10])  # focus only ahead of the robot
        # min_dist = min(data.ranges)

        # Let's create an object of type Twist to publish it later, initialised with zeros
        t = Twist()

        # If anything is closer than 0.5 metres anywhere in the
        # scan, we turn away
        if min_dist < 0.5:
            t.angular.z = 0.5
        else:
            t.linear.x = 0.2
        self.publisher.publish(t)

        # above in min_dist we found the nearest value,
        # but to display the position of the nearest value
        # we need to find which range index corresponds to that
        # min_value.  
        # find the index of the nearest point 
        # (trick from https://stackoverflow.com/a/11825864)
        # it really is a very Python-ic trick here using a getter 
        # function on the range. Can also be done with a 
        # classic for loop
        # index_min = min(
        #     list(range(len(data.ranges))),
        #     key=data.ranges.__getitem__)
        
        index_min = min(
            list(range(len(data.ranges[int(len(data.ranges)/2) -10 : int(len(data.ranges)/2) +10]))),
            key=data.ranges.__getitem__)
        
        # convert the obtained index to angle, using the 
        # reported angle_increment, and adding that to the 
        # angle_min, i.e. the angle the index 0 corresponds to.
        # (is negative, in fact -PI/2).
        alpha = data.angle_min + (index_min * data.angle_increment)

        # No time for some trigonometry to turn the 
        # polar coordinates into cartesian coordinates
        # inspired by https://stackoverflow.com/a/20926435
        # use trigonometry to create the point in laser 
        # z = 0, in the frame of the laser 
        laser_point_2d = [ 
            -cos(alpha) * min_dist, 
            sin(alpha) * min_dist,
            0.0]

        # create an empty PoseStamped to be published later.
        pose = PoseStamped()

        # keep the frame ID (the entire header here) as read from 
        # the sensor. This is general ROS practice, as it 
        # propagates the recording time from the sensor and 
        # the coordinate frame of the sensor through to
        # other results we may be publishing based on the analysis
        # of the data of that sensor
        pose._header = data.header

        # fill in the slots from the points calculated above.
        # bit tedious to do it this way, but hey... 
        pose._pose.position.x = laser_point_2d[0]
        pose._pose.position.y = laser_point_2d[1]
        pose._pose.position.z = laser_point_2d[2]

        # using my trick from https://github.com/marc-hanheide/jupyter-notebooks/blob/master/quaternion.ipynb
        # to convert to quaternion, so that the Pose always 
        # points in the direction of the laser beam. 
        # Not required if only the positions is
        # of relevance 
        # (then just set pose.pose.orientation.w = 1.0 and leave
        # others as 0).
        pose._pose.orientation.x = 0.0
        pose._pose.orientation.y = 0.0
        pose._pose.orientation.z = sin(alpha/2)
        pose._pose.orientation.w = cos(alpha/2)

        # publish the pose so it can be visualised in rviz
        self.pose_pub.publish(pose)

        # now lets actually transform this pose into a robot 
        # "base_link" frame.
        transform = self.tf_buffer.lookup_transform("base_link", "laser_link", rclpy.time.Time())
        transformed_pose = do_transform_pose(pose._pose, transform)
        print("The closest point in robot coords is at\n%s"
            % transformed_pose._position
            )

def main(args=None):
    rclpy.init(args=args)
    mover = Mover()
    rclpy.spin(mover)

    mover.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

import os
import numpy as np
import rclpy
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Pose, PoseArray
from visualization_msgs.msg import Marker

from dr_spaam.detector import Detector


class DrSpaamROS(Node):
    """ROS node to detect pedestrian using DROW3 or DR-SPAAM."""

    def __init__(self):
        super().__init__('dr_spaam_ros_node')

        self._read_params()
        self._detector = Detector(
            self.weight_file,
            model=self.detector_model,
            gpu=True,
            stride=self.stride,
            panoramic_scan=self.panoramic_scan,
        )
        self.create_pubs_subs()

    def _read_params(self):
        """
        @brief      Reads parameters from ROS server.
        """
        self.declare_parameter("weight_file", rclpy.parameter.Parameter.Type.STRING)
        self.declare_parameter("detector_model", rclpy.parameter.Parameter.Type.STRING)
        self.declare_parameter("conf_thresh", rclpy.parameter.Parameter.Type.DOUBLE)
        self.declare_parameter("stride", rclpy.parameter.Parameter.Type.INTEGER)
        self.declare_parameter("panoramic_scan", rclpy.parameter.Parameter.Type.BOOL)

        self.package_dir = get_package_share_directory('dr_spaam_ros')
        self.weight_file = os.path.join(self.package_dir,self.get_parameter("weight_file").get_parameter_value().string_value)
        self.detector_model = self.get_parameter("detector_model").get_parameter_value().string_value
        self.conf_thresh = self.get_parameter("conf_thresh").get_parameter_value().double_value
        self.stride = self.get_parameter("stride").get_parameter_value().integer_value
        self.panoramic_scan = self.get_parameter("panoramic_scan").get_parameter_value().bool_value

    def create_pubs_subs(self):
        """
        @brief      Initialize ROS connection.
        """
        # Publisher
        topic, queue_size, latch = read_publisher_param(self, "detections")
        self._dets_pub = self.create_publisher(
            PoseArray, topic, queue_size
        )

        topic, queue_size, latch = read_publisher_param(self, "rviz")
        self._rviz_pub = self.create_publisher(
            Marker, topic, queue_size
        )

        # Subscriber
        topic, queue_size = read_subscriber_param(self, "scan")
        qos_profile = rclpy.qos.QoSProfile(reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT, history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,depth=queue_size)
        self._scan_sub = self.create_subscription(
            LaserScan, topic, self._scan_callback, qos_profile
        )

    def _scan_callback(self, msg):
        if (
            self._dets_pub.get_subscription_count() == 0
            and self._rviz_pub.get_subscription_count() == 0
        ):
            return

        # TODO check the computation here
        if not self._detector.is_ready():
            self._detector.set_laser_fov(
                np.rad2deg(msg.angle_increment * len(msg.ranges))
            )

        scan = np.array(msg.ranges)
        scan[scan == 0.0] = 29.99
        scan[np.isinf(scan)] = 29.99
        scan[np.isnan(scan)] = 29.99

        # t = time.time()
        dets_xy, dets_cls, _ = self._detector(scan)
        # print("[DrSpaamROS] End-to-end inference time: %f" % (t - time.time()))

        # confidence threshold
        conf_mask = (dets_cls >= self.conf_thresh).reshape(-1)
        dets_xy = dets_xy[conf_mask]
        dets_cls = dets_cls[conf_mask]

        # convert to ros msg and publish
        dets_msg = detections_to_pose_array(dets_xy, dets_cls)
        dets_msg.header = msg.header
        self._dets_pub.publish(dets_msg)

        rviz_msg = detections_to_rviz_marker(dets_xy, dets_cls)
        rviz_msg.header = msg.header
        self._rviz_pub.publish(rviz_msg)


def detections_to_rviz_marker(dets_xy, dets_cls):
    """
    @brief     Convert detection to RViz marker msg. Each detection is marked as
               a circle approximated by line segments.
    """
    msg = Marker()
    msg.action = Marker.ADD
    msg.ns = "dr_spaam_ros"
    msg.id = 0
    msg.type = Marker.LINE_LIST

    # set quaternion so that RViz does not give warning
    msg.pose.orientation.x = 0.0
    msg.pose.orientation.y = 0.0
    msg.pose.orientation.z = 0.0
    msg.pose.orientation.w = 1.0

    msg.scale.x = 0.03  # line width
    # red color
    msg.color.r = 1.0
    msg.color.a = 1.0

    # circle
    r = 0.4
    ang = np.linspace(0, 2 * np.pi, 20)
    xy_offsets = r * np.stack((np.cos(ang), np.sin(ang)), axis=1)

    # to msg
    for d_xy, d_cls in zip(dets_xy, dets_cls):
        for i in range(len(xy_offsets) - 1):
            # start point of a segment
            p0 = Point()
            p0.x = d_xy[0] + xy_offsets[i, 0]
            p0.y = d_xy[1] + xy_offsets[i, 1]
            p0.z = 0.0
            msg.points.append(p0)

            # end point
            p1 = Point()
            p1.x = d_xy[0] + xy_offsets[i + 1, 0]
            p1.y = d_xy[1] + xy_offsets[i + 1, 1]
            p1.z = 0.0
            msg.points.append(p1)

    return msg


def detections_to_pose_array(dets_xy, dets_cls):
    pose_array = PoseArray()
    for d_xy, d_cls in zip(dets_xy, dets_cls):

        # Detector uses following frame convention:
        # x forward, y rightward, z downward, phi is angle w.r.t. x-axis
        p = Pose()
        p.position.x = float(d_xy[0])
        p.position.y = float(d_xy[1])
        p.position.z = 0.0
        pose_array.poses.append(p)

    return pose_array


def read_subscriber_param(node, name):
    """
    @brief      Convenience function to read subscriber parameter.
    """
    node.declare_parameter("subscriber." + name + ".topic", rclpy.Parameter.Type.STRING)
    node.declare_parameter("subscriber." + name + ".queue_size", rclpy.Parameter.Type.INTEGER)
    topic = node.get_parameter("subscriber." + name + ".topic").get_parameter_value().string_value
    queue_size = node.get_parameter("subscriber." + name + ".queue_size").get_parameter_value().integer_value

    return topic, queue_size


def read_publisher_param(node,name):
    """
    @brief      Convenience function to read publisher parameter.
    """
    node.declare_parameter("publisher." + name + ".topic", rclpy.Parameter.Type.STRING)
    node.declare_parameter("publisher." + name + ".queue_size", rclpy.Parameter.Type.INTEGER)
    node.declare_parameter("publisher." + name + ".latch", rclpy.Parameter.Type.BOOL)

    topic = node.get_parameter("publisher." + name + ".topic").get_parameter_value().string_value
    queue_size = node.get_parameter("publisher." + name + ".queue_size").get_parameter_value().integer_value
    latch = node.get_parameter("publisher." + name + ".latch").get_parameter_value().bool_value
    
    return topic, queue_size, latch

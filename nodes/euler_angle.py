#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import PoseWithCovarianceStamped, Vector3
from tf_transformations import euler_from_quaternion

class EulerAngle(Node):
    def __init__(self):
        super().__init__('euler_angle')
        
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.vision_pose_sub = self.create_subscription(
            msg_type=PoseWithCovarianceStamped,
            topic='vision_pose_cov',
            callback=self.on_vision_pose,
            qos_profile=qos,
        )

        self.euler_angle_pub = self.create_publisher(
            msg_type=Vector3,
            topic='roll_pitsch_yaw',
            qos_profile=1,
        )

    def on_vision_pose(self, msg: PoseWithCovarianceStamped):
        # get the vehicle orientation expressed as quaternion
        q = msg.pose.pose.orientation
        # convert the quaternion to euler angles
        (roll, pitch, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
        # yaw = self.wrap_pi(yaw)

        out_msg = Vector3()
        out_msg.header.stamp = self.get_clock().now().to_msg()
        out_msg.header.frame_id = 'map'

        out_msg.x = roll
        out_msg.y = pitch
        out_msg.z = yaw

        self.euler_angle_pub.publish(out_msg)

def main():
    rclpy.init()
    node = EulerAngle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
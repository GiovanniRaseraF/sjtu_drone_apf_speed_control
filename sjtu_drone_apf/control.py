# Author: Giovanni Rasera

import rclpy

from rclpy.node import Node
import sys
import termios
import tty

from std_msgs.msg import Empty, Bool, Int8, String
from geometry_msgs.msg import Twist, Pose, Vector3
from sensor_msgs.msg import Range, Image, Imu

# this is to test different types of topics
prefix = "/simple_drone/"

class APFConrolNode(Node):
    def __init__(self) -> None:
        self.pose = None
        super().__init__('apf_control')

        # Drone positioning
        self.sub_gt_pose = self.create_subscription(Pose, f"{prefix}gt_pose", self.cb_gt_pose, 10)

    
    def cb_gt_pose(self, p):
        self.pose = p
        # Message Structure
        """geometry_msgs.msg.Pose(
                position=geometry_msgs.msg.Point(
                    x=-9.277429850948648, 
                    y=-0.00543712248685583, 
                    z=0.05000287573880944
                ), 
                orientation=geometry_msgs.msg.Quaternion(
                    x=1.24447577829819e-05, 
                    y=-4.13604384898043e-06, 
                    z=0.00662843250080105, 
                    w=0.9999780316139969
                )
            )
        """

        pos = p.position
        x = pos.x
        y = pos.y
        z = pos.z

        print(f"x: {x}")
        print(f"y: {y}")
        print(f"z: {z}")



# run loop
def main(args=None):
    rclpy.init(args=args)
    apf_control_node = APFConrolNode()
    rclpy.spin(apf_control_node)
    apf_control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

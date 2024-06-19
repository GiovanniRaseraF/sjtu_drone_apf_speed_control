#!/usr/bin/env python3
# Copyright 2023 Georg Novotny
#
# Licensed under the GNU GENERAL PUBLIC LICENSE, Version 3.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.gnu.org/licenses/gpl-3.0.en.html
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy

from rclpy.node import Node
import sys
import termios
import tty

# Added By Giovanni Rasera
from std_msgs.msg import Empty, Bool, Int8, String
from geometry_msgs.msg import Twist, Pose, Vector3
from sensor_msgs.msg import Range, Image, Imu

MSG = """
Control Your Drone!
---------------------------
Moving around:
        w
    a   s    d
        x

t/l: takeoff/land (upper/lower case)
q/e : increase/decrease linear and angular velocity (upper/lower case)
A/D: rotate left/right
r/f : rise/fall (upper/lower case)
g: please go to goal !

h: to say Ciao sono Giovanni e Diletta

---------------------------
CTRL-C to quit
---------------------------

"""


class TeleopNode(Node):
    def __init__(self) -> None:
        self.pose = None

        super().__init__('teleop_node')

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.takeoff_publisher = self.create_publisher(Empty, 'takeoff', 10)
        self.land_publisher = self.create_publisher(Empty, 'land', 10)

        self.takeoff_sub = self.create_subscription(Empty, "takeoff", self.cb_takeoff, 10)
        
        self.sub_gt_pose = self.create_subscription(
            Pose, 
            "/simple_drone/gt_pose", 
            self.cb_gt_pose, 10)


        # Velocity parameters
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.linear_increment = 0.05
        self.angular_increment = 0.05
        self.max_linear_velocity = 1.0
        self.max_angular_velocity = 1.0

        # Start a timer to listen to keyboard inputs
        self.create_timer((1/30), self.read_keyboard_input)

    # Drone positioning 
    def cb_gt_pose(self, p):
        self.pose = p
        self.get_logger().info('pose: "%s"' % p.data)
    
    def cb_state(self, s : Int8):
        print("state")

    def cb_takeoff(self, e):
        print("empty")

    def get_velocity_msg(self) -> str:
        return "Linear Velocity: " + str(self.linear_velocity) + "\nAngular Velocity: " \
            + str(self.angular_velocity) + "\n"

    def read_keyboard_input(self) -> None:
        """
        Read keyboard inputs and publish corresponding commands
        """
        while rclpy.ok():
            # Print the instructions
            print(MSG+self.get_velocity_msg()+str(self.pose))
            # Implement a non-blocking keyboard read
            key = self.get_key()
            # Handle velocity changes
            if key.lower() == 'q':
                self.linear_velocity = min(self.linear_velocity + self.linear_increment,
                                           self.max_linear_velocity)
                self.angular_velocity = min(self.angular_velocity + self.angular_increment,
                                            self.max_angular_velocity)
            elif key.lower() == 'e':
                self.linear_velocity = max(self.linear_velocity - self.linear_increment,
                                           -self.max_linear_velocity)
                self.angular_velocity = max(self.angular_velocity - self.angular_increment,
                                            -self.max_angular_velocity)
            elif key.lower() == 'w':
                # Move forward
                linear_vec = Vector3()
                linear_vec.x = self.linear_velocity
                self.publish_cmd_vel(linear_vec)
            elif key.lower() == 's':
                # Hover
                self.publish_cmd_vel()
            elif key.lower() == 'x':
                # Move backward
                linear_vec = Vector3()
                linear_vec.x = -self.linear_velocity
                self.publish_cmd_vel(linear_vec)
            elif key == 'a':
                # Move Left
                linear_vec = Vector3()
                linear_vec.y = self.linear_velocity
                self.publish_cmd_vel(linear_vec)
            elif key == 'd':
                # Move right
                linear_vec = Vector3()
                linear_vec.y = -self.linear_velocity
                self.publish_cmd_vel(linear_vec)
            elif key == 'A':
                # Move Left
                angular_vec = Vector3()
                angular_vec.z = self.angular_velocity
                self.publish_cmd_vel(angular_vec=angular_vec)
            elif key == 'D':
                # Move right
                angular_vec = Vector3()
                angular_vec.z = -self.angular_velocity
                self.publish_cmd_vel(angular_vec=angular_vec)
            elif key.lower() == 'r':
                # Rise
                linear_vec = Vector3()
                linear_vec.z = self.linear_velocity
                self.publish_cmd_vel(linear_vec)
            elif key.lower() == 'f':
                # Fall
                linear_vec = Vector3()
                linear_vec.z = -self.angular_velocity
                self.publish_cmd_vel(linear_vec)
            # Handle other keys for different movements
            elif key.lower() == 't':
                # Takeoff
                self.takeoff_publisher.publish(Empty())
            elif key.lower() == 'l':
                # Land
                self.publish_cmd_vel()
                self.land_publisher.publish(Empty())
            elif key.lower() == 'h':
                # Say Ciao sono Giovanni
                print("Ciao sono Giovanni")

            elif key.lower() == "n": 
                # Subscribe to Drone positioning
                #self.pub_gt_pose = self.create_publisher(Pose, '/simple_drone/gt_pose', 10)
                #self.sub_state = self.create_subscription(Int8, 'state', self.cb_state, 1024)
                print("none")
            elif key.lower() == "g":
                print(f"please go to goal: from {self.pose}")
                tt = self.get_topic_names_and_types()
                print(tt)

    def get_key(self) -> str:
        """
        Function to capture keyboard input
        """
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def publish_cmd_vel(self, linear_vec: Vector3 = Vector3(),
                        angular_vec: Vector3 = Vector3()) -> None:
        """
        Publish a Twist message to cmd_vel topic
        """
        twist = Twist(linear=linear_vec, angular=angular_vec)
        self.cmd_vel_publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    teleop_node = TeleopNode()

    tt = teleop_node.get_topic_names_and_types()
    print(tt)
    t = input("ENTER> ")

    rclpy.spin(teleop_node)
    teleop_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

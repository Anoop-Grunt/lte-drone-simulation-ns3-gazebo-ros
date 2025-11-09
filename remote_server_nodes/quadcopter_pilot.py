#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math


class QuadcopterPilot(Node):
    def __init__(self):
        super().__init__('quadcopter_pilot')
        self.publisher_ = self.create_publisher(
            Twist,
            '/model_movement_commands',
            10
        )

        self.counter = 0

        self.get_logger().info('Remote server node started, publishing to /model_movement_commands')

    def timer_callback(self):
        msg = Twist()

        msg.linear.x = math.sin(self.counter * 0.1) * 2.0
        msg.linear.y = math.cos(self.counter * 0.1) * 2.0
        msg.linear.z = 0.0

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.5

        self.publisher_.publish(msg)
        self.get_logger().debug(f'Published Twist: linear.x={msg.linear.x:.2f}, linear.y={msg.linear.y:.2f}')

        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = QuadcopterPilot()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from pynput import keyboard

class QuadcopterPilot(Node):
    def __init__(self):
        super().__init__('quadcopter_pilot')
        # Publisher for Twist commands
        self.pub = self.create_publisher(Twist, '/model_movement_commands', 10)
        self.create_subscription(Float32, '/rsrp_values', self.rsrp_callback, 10)
        self.twist = Twist()
        self.linear_speed = 50.0
        self.angular_speed = 1.0
        self.pressed_keys = set()
        self.get_logger().info("Keyboard controller started. Use arrow keys or WASD. Press ESC to quit.")
        listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        listener.start()

    def rsrp_callback(self, msg):
        rsrp_value = msg.data
        self.get_logger().debug(f"Received RSRP: {rsrp_value}")

    def on_press(self, key):
        try:
            k = key.char.lower()
        except AttributeError:
            k = key.name
        
        self.pressed_keys.add(k)
        
        if k == 'esc':
            self.get_logger().info("Exiting keyboard control.")
            rclpy.shutdown()
        
        self.publish_twist()

    def on_release(self, key):
        try:
            k = key.char.lower()
        except AttributeError:
            k = key.name
        
        if k in self.pressed_keys:
            self.pressed_keys.remove(k)
        
        self.publish_twist()

    def publish_twist(self):
        # Reset twist
        self.twist.linear.x = 0.0
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0
        self.twist.angular.z = 0.0
        
        if 'w' in self.pressed_keys or 'up' in self.pressed_keys:
            self.twist.linear.x += self.linear_speed
        if 's' in self.pressed_keys or 'down' in self.pressed_keys:
            self.twist.linear.x -= self.linear_speed
        if 'a' in self.pressed_keys or 'left' in self.pressed_keys:
            self.twist.angular.z += self.angular_speed
        if 'd' in self.pressed_keys or 'right' in self.pressed_keys:
            self.twist.angular.z -= self.angular_speed
        if 'q' in self.pressed_keys:
            self.twist.linear.z += self.linear_speed  # ascend
        if 'e' in self.pressed_keys:
            self.twist.linear.z -= self.linear_speed  # descend
        
        # Publish immediately
        self.pub.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)
    node = QuadcopterPilot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

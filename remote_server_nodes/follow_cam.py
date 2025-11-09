import rclpy
from rclpy.node import Node
from rclpy.topic_endpoint_info import TopicEndpointInfo
import subprocess

class CameraTrackingSetup(Node):
    def __init__(self):
        super().__init__('camera_tracking_setup')
        self.timer = self.create_timer(0.5, self.check_and_setup)
        self.setup_done = False
    
    def gui_is_ready(self):
        """Check if GUI camera is actively publishing"""
        try:
            # Get all publishers to /gui/camera/pose
            publishers = self.get_publishers_info_by_topic('/gui/camera/pose')
            return len(publishers) > 0
        except:
            return False
    
    def check_and_setup(self):
        if self.setup_done:
            return
        
        if not self.gui_is_ready():
            self.get_logger().debug('Waiting for GUI to open...')
            return
        
        try:
            cmd = [
                'gz', 'topic', '-t', '/gui/track',
                '-m', 'gz.msgs.CameraTrack',
                '-p', 'track_mode: FOLLOW, follow_target: {name: "X3"}, follow_offset: {x: -3, y: 0.5, z: 1}, follow_pgain: 0.5'
            ]
            subprocess.run(cmd, check=True, capture_output=True, timeout=2)
            self.get_logger().info('Camera tracking enabled')
            self.setup_done = True
            self.timer.cancel()
        except Exception as e:
            self.get_logger().error(f'Failed to enable camera tracking: {e}')
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = CameraTrackingSetup()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

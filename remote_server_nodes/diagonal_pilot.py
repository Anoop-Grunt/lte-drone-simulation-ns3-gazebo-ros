#!/usr/bin/env python3
"""
DIAGONAL PILOT - Direct Path with RSRP Logging

Flies a direct diagonal path from start to goal.
Logs all connectivity metrics for comparison with RL agent and waypoint pilot.

Path: (0, 140, 25) -> (140, 10, 25) [DIAGONAL through map]
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
from ros_gz_interfaces.msg import Float32Array
from std_msgs.msg import UInt32
import numpy as np
import json
import os
from datetime import datetime


class DiagonalPilot(Node):
    def __init__(self):
        super().__init__('diagonal_pilot')
        
        # ============ SCENARIO (diagonal path) ============
        self.source = np.array([0.0, 140.0, 25.0])
        self.destination = np.array([140.0, 10.0, 25.0])
        self.GOAL_THRESHOLD = 12.0
        self.velocity = 5.0
        
        self.current_pos = self.source.copy()
        
        # ============ CONNECTIVITY TRACKING ============
        self.rsrp_values = []
        self.serving_cell = None
        self.previous_cell = None
        self.handover_history = []  # Track recent handovers for ping-pong detection
        
        # Metrics to log
        self.rsrp_history = []  # All RSRP readings
        self.serving_cell_history = []  # Track which cell over time
        self.position_history = []  # Track positions
        self.handover_count = 0
        self.ping_pong_count = 0
        self.handover_log = []
        self.good_signal_steps = 0  # RSRP > -90
        self.poor_signal_steps = 0  # RSRP < -95
        self.step_count = 0
        
        # State
        self.position_received = False
        self.mission_complete = False
        self.start_time = None
        
        # ============ ROS SETUP ============
        self.cmd_pub = self.create_publisher(Twist, '/model_movement_commands', 10)
        self.pose_sub = self.create_subscription(TFMessage, '/model/X3/pose', 
                                                 self.pose_callback, 10)
        self.rsrp_sub = self.create_subscription(Float32Array, '/rsrp_values',
                                                 self.rsrp_callback, 10)
        
        # Subscribe to NS-3 handover notifications
        self.handover_sub = self.create_subscription(
            UInt32,
            '/current_cell_id',
            self.handover_callback,
            10
        )
        
        # Control at 5 Hz (same as RL)
        self.control_timer = self.create_timer(0.2, self.control_loop)
        
        os.makedirs('/app/logs', exist_ok=True)
        
        self._print_startup()
    
    def _print_startup(self):
        self.get_logger().info("=" * 60)
        self.get_logger().info("DIAGONAL PILOT - Direct Path (with RSRP logging)")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Start: ({self.source[0]}, {self.source[1]}, {self.source[2]})")
        self.get_logger().info(f"Goal: ({self.destination[0]}, {self.destination[1]}, {self.destination[2]})")
        distance = np.linalg.norm(self.destination - self.source)
        self.get_logger().info(f"Distance: {distance:.1f}m (DIAGONAL path)")
        self.get_logger().info("=" * 60)
    
    def pose_callback(self, msg):
        for transform in msg.transforms:
            if 'X3' in transform.child_frame_id:
                self.current_pos[0] = transform.transform.translation.x
                self.current_pos[1] = transform.transform.translation.y
                self.current_pos[2] = transform.transform.translation.z
                
                if not self.position_received:
                    self.position_received = True
                    self.start_time = self.get_clock().now()
                    self.get_logger().info("Position received, starting diagonal flight...")
    
    def rsrp_callback(self, msg):
        """Store RSRP values for logging (handover detection now via NS-3)"""
        self.rsrp_values = list(msg.data)
        
        # If we don't have a serving cell yet (first callback), use strongest
        if self.serving_cell is None and len(self.rsrp_values) > 0:
            self.serving_cell = int(np.argmax(self.rsrp_values))
            self.get_logger().info(f"Initial cell: eNB{self.serving_cell}")
    
    def detect_ping_pong(self, new_cell):
        """Detect ping-pong handovers: A -> B -> A pattern"""
        if len(self.handover_history) < 1:
            return False
        
        # Check if we're going back to a recent cell
        prev_cell = self.handover_history[-1]
        if new_cell == prev_cell:
            return True
        
        return False
    
    def process_handover(self, new_cell_id):
        """Process handover event from NS-3"""
        # Check if this is actually a handover (cell changed)
        if self.serving_cell is not None and new_cell_id != self.serving_cell:
            # Detect ping-pong
            is_ping_pong = self.detect_ping_pong(new_cell_id)
            
            # Get current RSRP if available
            current_rsrp = self.rsrp_values[new_cell_id] if new_cell_id < len(self.rsrp_values) else -100.0
            
            # Log handover
            if is_ping_pong:
                self.ping_pong_count += 1
                self.get_logger().warn(
                    f"PING-PONG HO#{self.handover_count + 1}: "
                    f"eNB{self.serving_cell} ↔ eNB{new_cell_id} "
                    f"at ({self.current_pos[0]:.0f},{self.current_pos[1]:.0f})"
                )
            else:
                self.get_logger().info(
                    f"HANDOVER #{self.handover_count + 1}: "
                    f"eNB{self.serving_cell} → eNB{new_cell_id} "
                    f"at ({self.current_pos[0]:.0f},{self.current_pos[1]:.0f})"
                )
            
            # Log handover details
            self.handover_log.append({
                'step': self.step_count,
                'from_cell': int(self.serving_cell),
                'to_cell': int(new_cell_id),
                'ping_pong': is_ping_pong,
                'position': self.current_pos.tolist(),
                'rsrp': float(current_rsrp)
            })
            
            # Update history
            self.handover_history.append(self.serving_cell)
            self.handover_count += 1
            
            return is_ping_pong
        
        return False
    
    def handover_callback(self, msg):
        """Handover notification from NS-3 (accurate handover detection)"""
        new_cell_id = int(msg.data)
        
        # Process handover
        self.process_handover(new_cell_id)
        
        # Update serving cell
        self.serving_cell = new_cell_id
    
    def control_loop(self):
        if not self.position_received or self.mission_complete:
            return
        
        distance = np.linalg.norm(self.current_pos[:2] - self.destination[:2])
        
        # ===== LOG CONNECTIVITY DATA =====
        if self.rsrp_values and self.serving_cell is not None:
            current_rsrp = self.rsrp_values[int(self.serving_cell)]
            self.rsrp_history.append(current_rsrp)
            self.serving_cell_history.append(int(self.serving_cell))
            self.position_history.append(self.current_pos.tolist())
            
            # Categorize signal quality
            if current_rsrp > -90:
                self.good_signal_steps += 1
            if current_rsrp < -95:
                self.poor_signal_steps += 1
        
        self.step_count += 1
        
        # ===== CHECK GOAL =====
        if distance < self.GOAL_THRESHOLD:
            self.complete_mission(success=True)
            return
        
        # ===== TIMEOUT (same as RL: 500 steps) =====
        if self.step_count >= 500:
            self.complete_mission(success=False)
            return
        
        # ===== FLY STRAIGHT TO GOAL (DIAGONAL) =====
        direction = self.destination[:2] - self.current_pos[:2]
        direction_norm = direction / np.linalg.norm(direction)
        
        cmd = Twist()
        cmd.linear.x = float(direction_norm[0] * self.velocity)
        cmd.linear.y = float(direction_norm[1] * self.velocity)
        
        # Altitude control
        target_alt = 25.0
        alt_error = target_alt - self.current_pos[2]
        cmd.linear.z = float(np.clip(alt_error * 1.0, -3.0, 3.0))
        
        self.cmd_pub.publish(cmd)
        
        # ===== LOGGING =====
        if self.step_count % 25 == 0:
            progress_pct = 100 * (1 - distance / np.linalg.norm(self.destination - self.source))
            rsrp_str = f"{self.rsrp_history[-1]:.1f}" if self.rsrp_history else "N/A"
            
            self.get_logger().info(
                f"[Step {self.step_count}] "
                f"Pos:({self.current_pos[0]:.0f},{self.current_pos[1]:.0f}) | "
                f"Dist:{distance:.0f}m ({progress_pct:.0f}%) | "
                f"RSRP:{rsrp_str} T{self.serving_cell} | "
                f"HO:{self.handover_count}"
            )
    
    def complete_mission(self, success):
        self.mission_complete = True
        self.cmd_pub.publish(Twist())  # Stop
        
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        distance = np.linalg.norm(self.current_pos[:2] - self.destination[:2])
        
        # Calculate metrics
        avg_rsrp = np.mean(self.rsrp_history) if self.rsrp_history else -100.0
        min_rsrp = np.min(self.rsrp_history) if self.rsrp_history else -100.0
        max_rsrp = np.max(self.rsrp_history) if self.rsrp_history else -100.0
        
        total_distance = np.linalg.norm(self.destination - self.source)
        progress = total_distance - distance
        progress_pct = 100 * progress / total_distance
        
        # Print summary
        self.get_logger().info("\n" + "=" * 60)
        self.get_logger().info(f"DIAGONAL MISSION: {'SUCCESS' if success else 'TIMEOUT'}")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Time: {elapsed:.1f}s | Steps: {self.step_count}")
        self.get_logger().info(f"Progress: {progress:.1f}m ({progress_pct:.0f}%)")
        self.get_logger().info(f"Final distance: {distance:.1f}m")
        self.get_logger().info("-" * 60)
        self.get_logger().info("CONNECTIVITY METRICS:")
        self.get_logger().info(f"  Avg RSRP: {avg_rsrp:.2f} dBm")
        self.get_logger().info(f"  Min RSRP: {min_rsrp:.2f} dBm")
        self.get_logger().info(f"  Max RSRP: {max_rsrp:.2f} dBm")
        self.get_logger().info(f"  Good signal steps (>-90): {self.good_signal_steps} ({100*self.good_signal_steps/self.step_count:.1f}%)")
        self.get_logger().info(f"  Poor signal steps (<-95): {self.poor_signal_steps} ({100*self.poor_signal_steps/self.step_count:.1f}%)")
        self.get_logger().info(f"  Handovers: {self.handover_count}")
        self.get_logger().info(f"  Ping-pongs: {self.ping_pong_count}")
        self.get_logger().info("=" * 60)
        
        # Save results
        results = {
            'type': 'diagonal_direct',
            'success': success,
            'time': elapsed,
            'steps': self.step_count,
            'progress_meters': progress,
            'progress_pct': progress_pct,
            'final_distance': distance,
            'avg_rsrp': avg_rsrp,
            'min_rsrp': min_rsrp,
            'max_rsrp': max_rsrp,
            'good_signal_steps': self.good_signal_steps,
            'poor_signal_steps': self.poor_signal_steps,
            'good_signal_pct': 100 * self.good_signal_steps / self.step_count if self.step_count > 0 else 0,
            'poor_signal_pct': 100 * self.poor_signal_steps / self.step_count if self.step_count > 0 else 0,
            'handovers': self.handover_count,
            'ping_pongs': self.ping_pong_count,
            'handover_log': self.handover_log,
            'rsrp_history': self.rsrp_history,
            'position_history': self.position_history,
            'serving_cell_history': self.serving_cell_history
        }
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filepath = f'/app/logs/diagonal_results_{timestamp}.json'
        
        try:
            with open(filepath, 'w') as f:
                json.dump(results, f, indent=2)
            self.get_logger().info(f"Results saved to: {filepath}")
        except Exception as e:
            self.get_logger().error(f"Failed to save: {e}")
        
        self.get_logger().info("\nShutting down in 3 seconds...")
        import time
        time.sleep(3)
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = DiagonalPilot()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()

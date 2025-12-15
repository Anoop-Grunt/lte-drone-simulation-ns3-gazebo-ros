#!/usr/bin/env python3
"""
RL TEST PILOT - Evaluate Trained Q-table (FIXED)
Loads trained Q-table and runs test episodes with ε=0 (pure exploitation)

FIX: Properly initialize episode state to allow UAV movement
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
from ros_gz_interfaces.msg import Float32Array
import numpy as np
import pickle
import json
import os
from datetime import datetime
from collections import deque
import argparse


class RLTestPilot(Node):
    
    def __init__(self, test_episodes=20, q_table_path=None):
        super().__init__('rl_test')
        
        # ============ LOAD TRAINED Q-TABLE ============
        # Auto-detect latest Q-table if not specified
        if q_table_path is None:
            import glob
            q_tables = sorted(glob.glob('/app/models/q_table_*.pkl'))
            if not q_tables:
                self.get_logger().error("❌ No Q-table files found in /app/models/")
                raise FileNotFoundError("No Q-table found. Train a model first using rl_pilot.")
            q_table_path = q_tables[-1]  # Get most recent by filename (sorted)
            self.get_logger().info(f"🔍 Auto-detected Q-table: {q_table_path}")
        
        try:
            with open(q_table_path, 'rb') as f:
                self.q_table = pickle.load(f)
            self.get_logger().info(f"✅ Loaded Q-table: {q_table_path}")
            self.get_logger().info(f"   Q-table size: {len(self.q_table)} states")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to load Q-table: {e}")
            raise
        
        # ============ SCENARIO ============
        self.source = np.array([40.0, 25.0, 25.0])
        self.destination = np.array([130.0, 130.0, 25.0])
        
        # ============ GRID CONFIGURATION ============
        self.GRID_SIZE = 15.0
        self.MAP_MIN = 0.0
        self.MAP_MAX = 150.0
        
        self.current_pos = self.source.copy()
        self.last_pos = self.source.copy()
        
        # ============ TEST CONFIGURATION ============
        self.num_episodes = test_episodes
        self.current_episode = 0
        self.current_step = 0
        self.episode_handovers = 0
        self.episode_ping_pongs = 0
        
        # FIX: Set position_received to False initially, but episode will start once we get first pose
        self.position_received = False
        self.episode_active = False  # Will be set to True after first position
        self.episode_started = False  # Track if we've logged episode start
        
        # ============ CONNECTIVITY TRACKING ============
        self.rsrp_values = []
        self.serving_cell = None
        self.handover_history = deque(maxlen=5)
        self.good_signal_steps = 0
        self.poor_signal_steps = 0
        self.rsrp_history = []
        
        # ============ ACTIONS ============
        self.actions = ['east', 'northeast', 'north', 'northwest', 
                       'west', 'southwest', 'south', 'southeast']
        
        # ============ TEST MODE ============
        self.epsilon = 0.0  # Pure exploitation
        
        self.GOAL_THRESHOLD = 12.0
        self.progress_window = deque(maxlen=40)
        
        # ============ TEST RESULTS ============
        self.test_results = []
        self.episode_start_time = None
        
        # ============ ROS SETUP ============
        self.cmd_pub = self.create_publisher(Twist, '/model_movement_commands', 10)
        self.pose_sub = self.create_subscription(TFMessage, '/model/X3/pose', 
                                                 self.pose_callback, 10)
        self.rsrp_sub = self.create_subscription(Float32Array, '/rsrp_values',
                                                 self.rsrp_callback, 10)
        
        self.control_timer = self.create_timer(0.2, self.control_loop)
        
        os.makedirs('/app/logs', exist_ok=True)
        
        self._print_startup()
    
    def _print_startup(self):
        """Print test configuration"""
        self.get_logger().info("=" * 70)
        self.get_logger().info("🧪 RL TEST MODE - Evaluating Trained Policy")
        self.get_logger().info("=" * 70)
        self.get_logger().info(f"📍 Start: ({self.source[0]}, {self.source[1]}, {self.source[2]})")
        self.get_logger().info(f"🎯 Goal: ({self.destination[0]}, {self.destination[1]}, {self.destination[2]})")
        distance = np.linalg.norm(self.destination - self.source)
        self.get_logger().info(f"📏 Distance: {distance:.1f}m")
        self.get_logger().info(f"🧪 Test Episodes: {self.num_episodes}")
        self.get_logger().info(f"🎲 Epsilon: {self.epsilon} (pure exploitation)")
        self.get_logger().info(f"📚 Q-table states: {len(self.q_table)}")
        self.get_logger().info("=" * 70)
    
    # ============ STATE REPRESENTATION ============
    
    def position_to_grid(self, x, y):
        """Convert position to grid coordinates"""
        grid_x = int(np.clip(x / self.GRID_SIZE, 0, (self.MAP_MAX / self.GRID_SIZE) - 1))
        grid_y = int(np.clip(y / self.GRID_SIZE, 0, (self.MAP_MAX / self.GRID_SIZE) - 1))
        return (grid_x, grid_y)
    
    def get_signal_quality(self):
        """Categorize current signal quality"""
        if not self.rsrp_values or self.serving_cell is None:
            return 'unknown'
        
        rsrp = self.rsrp_values[int(self.serving_cell)]
        
        if rsrp > -85:
            return 'excellent'
        elif rsrp > -90:
            return 'good'
        elif rsrp > -95:
            return 'fair'
        else:
            return 'poor'
    
    def get_distance_band(self, distance):
        """Categorize distance to goal"""
        if distance < 20:
            return 'very_close'
        elif distance < 50:
            return 'close'
        elif distance < 80:
            return 'medium'
        else:
            return 'far'
    
    def get_state(self):
        """Get current state"""
        grid_x, grid_y = self.position_to_grid(self.current_pos[0], self.current_pos[1])
        cell = int(self.serving_cell) if self.serving_cell is not None else 0
        
        distance = np.linalg.norm(self.current_pos[:2] - self.destination[:2])
        dist_band = self.get_distance_band(distance)
        signal_qual = self.get_signal_quality()
        
        return (grid_x, grid_y, cell, dist_band, signal_qual)
    
    # ============ ACTION SELECTION ============
    
    def choose_action(self, state):
        """
        TEST MODE: Always pick best action (greedy)
        If state not in Q-table, use heuristic toward goal
        """
        if state not in self.q_table:
            # State never seen in training - use goal-directed heuristic
            self.get_logger().debug(f"⚠️ Unseen state: {state} - using heuristic")
            
            dx = self.destination[0] - self.current_pos[0]
            dy = self.destination[1] - self.current_pos[1]
            
            # Pick action toward goal
            if abs(dx) > abs(dy):
                return 'east' if dx > 0 else 'west'
            else:
                return 'north' if dy > 0 else 'south'
        
        # Pick best action from Q-table (greedy)
        return max(self.q_table[state], key=self.q_table[state].get)
    
    # ============ EXECUTION ============
    
    def execute_action(self, action):
        """Execute movement command"""
        cmd = Twist()
        velocity = 5.0
        
        directions = {
            'north': (0.0, 1.0),
            'south': (0.0, -1.0),
            'east': (1.0, 0.0),
            'west': (-1.0, 0.0),
            'northeast': (0.707, 0.707),
            'northwest': (-0.707, 0.707),
            'southeast': (0.707, -0.707),
            'southwest': (-0.707, -0.707),
        }
        
        if action in directions:
            vx, vy = directions[action]
            cmd.linear.x = float(vx * velocity)
            cmd.linear.y = float(vy * velocity)
        
        target_alt = 25.0
        alt_error = target_alt - self.current_pos[2]
        cmd.linear.z = float(np.clip(alt_error * 1.0, -3.0, 3.0))
        
        self.cmd_pub.publish(cmd)
    
    # ============ TRACKING ============
    
    def detect_ping_pong(self, new_cell):
        """Detect ping-pong handovers"""
        if len(self.handover_history) < 2:
            return False
        
        if len(self.handover_history) >= 2:
            prev_cell = self.handover_history[-1]
            prev_prev_cell = self.handover_history[-2]
            
            if new_cell == prev_prev_cell and new_cell != prev_cell:
                return True
        
        return False
    
    def track_handovers(self):
        """Track handovers and ping-pongs"""
        current_cell = int(self.serving_cell) if self.serving_cell is not None else None
        
        if not hasattr(self, '_last_cell'):
            self._last_cell = current_cell
        else:
            if current_cell is not None and self._last_cell is not None:
                if current_cell != self._last_cell:
                    is_ping_pong = self.detect_ping_pong(current_cell)
                    
                    if is_ping_pong:
                        self.episode_ping_pongs += 1
                        self.get_logger().info(f"⚠️ PING-PONG: eNB{self._last_cell} ↔ eNB{current_cell}")
                    
                    self.episode_handovers += 1
                    self.handover_history.append(self._last_cell)
                    
                    if not is_ping_pong:
                        self.get_logger().info(f"📶 HO#{self.episode_handovers}: eNB{self._last_cell} → eNB{current_cell}")
            
            self._last_cell = current_cell
    
    def track_connectivity(self):
        """Track RSRP quality"""
        if self.rsrp_values and self.serving_cell is not None:
            rsrp = self.rsrp_values[int(self.serving_cell)]
            self.rsrp_history.append(rsrp)
            
            if rsrp > -90:
                self.good_signal_steps += 1
            elif rsrp < -95:
                self.poor_signal_steps += 1
    
    # ============ ROS CALLBACKS ============
    
    def pose_callback(self, msg):
        """Position update"""
        for transform in msg.transforms:
            if 'X3' in transform.child_frame_id:
                self.current_pos[0] = transform.transform.translation.x
                self.current_pos[1] = transform.transform.translation.y
                self.current_pos[2] = transform.transform.translation.z
                
                # FIX: Set flags properly on first position update
                if not self.position_received:
                    self.position_received = True
                    self.episode_active = True  # Start episode immediately
                    self.episode_start_time = self.get_clock().now()
                    
                    distance = np.linalg.norm(self.current_pos[:2] - self.destination[:2])
                    self.get_logger().info(f"🧪 Test Episode {self.current_episode+1}/{self.num_episodes} started!")
                    self.get_logger().info(f"   Position: ({self.current_pos[0]:.1f}, {self.current_pos[1]:.1f}, {self.current_pos[2]:.1f})")
                    self.get_logger().info(f"   Distance to goal: {distance:.1f}m")
                    self.episode_started = True
    
    def rsrp_callback(self, msg):
        """RSRP update"""
        self.rsrp_values = list(msg.data)
        if len(self.rsrp_values) > 0:
            self.serving_cell = np.argmax(self.rsrp_values)
    
    # ============ MAIN CONTROL LOOP ============
    
    def control_loop(self):
        """Main loop at 5 Hz"""
        # FIX: Only wait for position, episode_active will be set automatically
        if not self.position_received:
            return
        
        if not self.episode_active:
            return
        
        distance = float(np.linalg.norm(self.current_pos[:2] - self.destination[:2]))
        
        # Check termination
        if distance < self.GOAL_THRESHOLD:
            self.end_episode(reached_goal=True)
            return
        
        # Progress timeout
        self.progress_window.append(distance)
        if len(self.progress_window) == self.progress_window.maxlen:
            progress = self.progress_window[0] - self.progress_window[-1]
            if progress < 0.5 and self.current_step > 100:
                self.get_logger().info(f"⏸️ No progress timeout")
                self.end_episode(reached_goal=False)
                return
        
        # Hard step limit
        if self.current_step >= 500:
            self.get_logger().info("⏱️ Max steps timeout")
            self.end_episode(reached_goal=False)
            return
        
        # Get state and choose action (greedy)
        state = self.get_state()
        action = self.choose_action(state)
        
        # Execute
        self.execute_action(action)
        
        # Track metrics
        self.track_handovers()
        self.track_connectivity()
        
        self.current_step += 1
        
        # Log every 25 steps
        if self.current_step % 25 == 0:
            progress_pct = 100 * (1 - distance / np.linalg.norm(self.destination - self.source))
            
            if self.rsrp_values and self.serving_cell is not None:
                rsrp = self.rsrp_values[int(self.serving_cell)]
                qual = self.get_signal_quality().upper()
                rsrp_str = f"RSRP:{rsrp:.1f}[{qual}] T{self.serving_cell}"
            else:
                rsrp_str = "RSRP:N/A"
            
            self.get_logger().info(
                f"[Test {self.current_episode+1} S{self.current_step}] "
                f"Pos:({self.current_pos[0]:.0f},{self.current_pos[1]:.0f}) | "
                f"Dist:{distance:.0f}m ({progress_pct:.0f}%) | "
                f"Act:{action.upper()} | "
                f"HO:{self.episode_handovers} PP:{self.episode_ping_pongs} | "
                f"{rsrp_str}"
            )
    
    # ============ EPISODE MANAGEMENT ============
    
    def reset_uav_in_gazebo(self):
        """Reset UAV position"""
        import subprocess
        import time
        
        try:
            cmd = [
                'gz', 'service',
                '-s', '/world/quadcopter_teleop/set_pose',
                '--reqtype', 'gz.msgs.Pose',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '2000',
                '--req',
                f'name: "X3", position: {{x: {self.source[0]}, y: {self.source[1]}, z: {self.source[2]}}}'
            ]
            
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=5.0)
            time.sleep(1.0)
            return result.returncode == 0
        except Exception as e:
            self.get_logger().error(f"Reset failed: {e}")
            return False
    
    def end_episode(self, reached_goal=False):
        """End test episode"""
        self.episode_active = False
        self.cmd_pub.publish(Twist())
        
        elapsed = (self.get_clock().now() - self.episode_start_time).nanoseconds / 1e9
        distance = np.linalg.norm(self.current_pos[:2] - self.destination[:2])
        
        total_distance = np.linalg.norm(self.destination - self.source)
        progress = total_distance - distance
        progress_pct = 100 * progress / total_distance
        
        avg_episode_rsrp = np.mean(self.rsrp_history) if self.rsrp_history else -100.0
        
        # Print summary
        self.get_logger().info("\n" + "=" * 60)
        self.get_logger().info(f"Test Episode {self.current_episode+1}/{self.num_episodes}: {'✅ SUCCESS!' if reached_goal else '⏱️ TIMEOUT'}")
        self.get_logger().info(f"  Time: {elapsed:.1f}s | Steps: {self.current_step}")
        self.get_logger().info(f"  Progress: {progress:.1f}m ({progress_pct:.0f}%)")
        self.get_logger().info(f"  Final dist: {distance:.1f}m")
        self.get_logger().info(f"  Handovers: {self.episode_handovers} (Ping-pong: {self.episode_ping_pongs})")
        
        if self.rsrp_history:
            self.get_logger().info(f"  Avg RSRP: {avg_episode_rsrp:.1f} dBm")
        
        self.get_logger().info(f"  Good signal: {self.good_signal_steps} steps | Poor: {self.poor_signal_steps} steps")
        self.get_logger().info("=" * 60)
        
        # Save results
        self.test_results.append({
            'episode': int(self.current_episode + 1),
            'success': bool(reached_goal),
            'steps': int(self.current_step),
            'time': float(elapsed),
            'progress_meters': float(progress),
            'progress_pct': float(progress_pct),
            'handovers': int(self.episode_handovers),
            'ping_pong': int(self.episode_ping_pongs),
            'avg_rsrp': float(avg_episode_rsrp),
            'min_rsrp': float(np.min(self.rsrp_history)) if self.rsrp_history else -100.0,
            'max_rsrp': float(np.max(self.rsrp_history)) if self.rsrp_history else -100.0,
            'final_distance': float(distance),
            'good_signal_steps': int(self.good_signal_steps),
            'poor_signal_steps': int(self.poor_signal_steps)
        })
        
        self.current_episode += 1
        
        if self.current_episode < self.num_episodes:
            # Reset for next test episode
            self.current_step = 0
            self.episode_handovers = 0
            self.episode_ping_pongs = 0
            self.good_signal_steps = 0
            self.poor_signal_steps = 0
            self.position_received = False
            self.episode_started = False
            self.current_pos = self.source.copy()
            self.progress_window.clear()
            self.handover_history.clear()
            self.rsrp_history = []
            
            if hasattr(self, '_last_cell'):
                self._last_cell = None
            
            self.get_logger().info(f"\n🔄 Resetting for Test Episode {self.current_episode+1}...")
            reset_success = self.reset_uav_in_gazebo()
            
            if reset_success:
                self.get_logger().info("✅ Reset successful!")
            else:
                self.get_logger().warn("⚠️ Reset failed, waiting...")
            
            import time
            time.sleep(3.0)
            self.get_logger().info(f"▶️ Starting Test Episode {self.current_episode+1}...\n")
        else:
            self.finish_testing()
    
    def finish_testing(self):
        """Testing complete"""
        self.get_logger().info("\n" + "=" * 70)
        self.get_logger().info("🧪 TESTING COMPLETE!")
        self.get_logger().info("=" * 70)
        
        # Save results
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        try:
            with open(f'/app/logs/test_results_{timestamp}.json', 'w') as f:
                json.dump(self.test_results, f, indent=2)
            self.get_logger().info(f"Saved: /app/logs/test_results_{timestamp}.json")
        except Exception as e:
            self.get_logger().error(f"Save error: {e}")
        
        # Print summary
        successes = [ep for ep in self.test_results if ep['success']]
        
        self.get_logger().info(f"\n📊 TEST SUMMARY:")
        self.get_logger().info(f"  Success rate: {len(successes)}/{len(self.test_results)} ({100*len(successes)/len(self.test_results):.1f}%)")
        
        if successes:
            self.get_logger().info(f"  Avg handovers (successful): {np.mean([ep['handovers'] for ep in successes]):.1f}")
            self.get_logger().info(f"  Avg ping-pongs (successful): {np.mean([ep['ping_pong'] for ep in successes]):.1f}")
            self.get_logger().info(f"  Avg RSRP (successful): {np.mean([ep['avg_rsrp'] for ep in successes]):.1f} dBm")
            self.get_logger().info(f"  Avg time (successful): {np.mean([ep['time'] for ep in successes]):.1f}s")
        
        self.get_logger().info(f"\n  Overall avg handovers: {np.mean([ep['handovers'] for ep in self.test_results]):.1f}")
        self.get_logger().info(f"  Overall avg RSRP: {np.mean([ep['avg_rsrp'] for ep in self.test_results]):.1f} dBm")
        
        self.get_logger().info("=" * 70)
        
        self.get_logger().info("Shutting down in 5 seconds...")
        import time
        time.sleep(5)
        rclpy.shutdown()


def main(args=None):
    parser = argparse.ArgumentParser(description='RL Test Pilot - Evaluate trained Q-table')
    parser.add_argument('--episodes', type=int, default=20, 
                       help='Number of test episodes (default: 20)')
    parser.add_argument('--qtable', type=str, default=None,
                       help='Path to Q-table file (default: auto-detect latest in /app/models/)')
    parsed_args, _ = parser.parse_known_args()
    
    rclpy.init(args=args)
    node = RLTestPilot(test_episodes=parsed_args.episodes, q_table_path=parsed_args.qtable)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("\n⚠️ Interrupted by user")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
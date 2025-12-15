#!/usr/bin/env python3
"""
IMPROVED RL PILOT - Learns Curved Path for Good Connectivity
(40, 25, 25) -> (130, 130, 25) in 50 episodes

Key Features:
1. Rich state space: (grid_position, serving_cell, distance_band, signal_quality)
2. Ping-pong handover detection & heavy penalties
3. Connectivity-aware rewards (staying in good coverage)
4. Progressive timeout (checks actual progress, not just steps)
5. Handover efficiency bonus at goal
6. **UPDATED: Subscribes to /current_cell_id for handover notifications**
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
from ros_gz_interfaces.msg import Float32Array
from std_msgs.msg import UInt32  # NEW: For handover notifications
import numpy as np
import pickle
import json
import os
from datetime import datetime
from collections import deque
import argparse


class ConnectivityAwareRLPilot(Node):
    
    def __init__(self, episodes=50):
        super().__init__('rl_pilot')
        
        # ============ SCENARIO (L-shaped map with dead zone) ============
        self.source = np.array([0.0, 140.0, 25.0])
        self.destination = np.array([140.0, 10.0, 25.0])
        
        # ============ GRID & MAP CONFIGURATION ============
        self.GRID_SIZE = 15.0  # 15m cells - balance between granularity & state space
        self.MAP_MIN = 0.0
        self.MAP_MAX = 150.0
        
        self.current_pos = self.source.copy()
        self.last_pos = self.source.copy()
        
        # ============ EPISODE TRACKING ============
        self.num_episodes = episodes
        self.current_episode = 0
        self.current_step = 0
        self.episode_reward = 0.0
        self.episode_handovers = 0
        self.episode_ping_pongs = 0
        self.position_received = False
        self.episode_active = False
        self.episode_waypoints = []

        # ============ CONNECTIVITY TRACKING ============
        self.rsrp_values = []
        self.serving_cell = None  # Current serving cell ID (0, 1, or 2)
        self.previous_cell = None  # Track previous cell for handover detection
        self.handover_history = deque(maxlen=5)  # Track recent handovers for ping-pong detection
        self.good_signal_steps = 0  # Count steps with good RSRP
        self.poor_signal_steps = 0  # Count steps with poor RSRP
        self.rsrp_history = []  # Track RSRP of serving cell over episode
        self.best_avg_rsrp_successful = -100.0  # Best average RSRP from SUCCESSFUL episodes only
        
        # NEW: Handover event tracking
        self.pending_handover_cell = None  # Store cell ID from handover event
        self.handover_just_occurred = False  # Flag to process handover in control loop
        
        # ============ Q-LEARNING PARAMETERS ============
        self.q_table = {}
        self.actions = ['east', 'northeast', 'north', 'northwest', 
                       'west', 'southwest', 'south', 'southeast']
        
        # Epsilon decay with floor
        self.epsilon_start = 0.3
        self.epsilon_min = 0.1  # Keep 10% exploration always
        self.epsilon_decay = 0.96  # Slower decay to reach 0.1 around episode 30
        self.epsilon = self.epsilon_start
        
        # Learning parameters
        self.alpha = 0.25  # Learning rate
        self.gamma = 0.95  # Discount factor
        
        self.last_state = None
        self.last_action = None
        self.GOAL_THRESHOLD = 12.0
        
        # ============ PROGRESS TRACKING (for timeout) ============
        self.progress_window = deque(maxlen=40)  # 8 seconds at 5Hz
        self.last_progress_check = 0
        
        # ============ HISTORY & LOGGING ============
        self.training_history = []
        self.episode_start_time = None
        
        # ============ ROS SETUP ============
        self.cmd_pub = self.create_publisher(Twist, '/model_movement_commands', 10)
        self.pose_sub = self.create_subscription(TFMessage, '/model/X3/pose', 
                                                 self.pose_callback, 10)
        self.rsrp_sub = self.create_subscription(Float32Array, '/rsrp_values',
                                                 self.rsrp_callback, 10)
        
        # NEW: Subscribe to handover notifications from NS-3
        self.handover_sub = self.create_subscription(
            UInt32,
            '/current_cell_id',
            self.handover_callback,
            10
        )
        
        # Timer at 5 Hz
        self.control_timer = self.create_timer(0.2, self.control_loop)
        
        os.makedirs('/app/models', exist_ok=True)
        os.makedirs('/app/logs', exist_ok=True)
        
        self._print_startup()
    
    def _print_startup(self):
        """Print configuration"""
        self.get_logger().info("=" * 70)
        self.get_logger().info("CONNECTIVITY-AWARE RL PILOT (with NS-3 Handover Events)")
        self.get_logger().info("=" * 70)
        self.get_logger().info(f"Start: ({self.source[0]}, {self.source[1]}, {self.source[2]})")
        self.get_logger().info(f"Goal: ({self.destination[0]}, {self.destination[1]}, {self.destination[2]})")
        distance = np.linalg.norm(self.destination - self.source)
        self.get_logger().info(f"Distance: {distance:.1f}m")
        self.get_logger().info(f"Episodes: {self.num_episodes}")
        self.get_logger().info("=" * 70)
        self.get_logger().info("LEARNING OBJECTIVES:")
        self.get_logger().info("  ✓ Reach destination efficiently")
        self.get_logger().info("  ✓ Maintain good signal quality (RSRP > -90 dBm)")
        self.get_logger().info("  ✓ Avoid ping-pong handovers")
        self.get_logger().info("  ✓ Avoid dead zone (center area)")
        self.get_logger().info("  ✓ Prefer edge path (L-shape) over diagonal")
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
        
        # Make sure serving_cell is valid index
        if self.serving_cell >= len(self.rsrp_values):
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
        """
        RICH STATE: (grid_x, grid_y, cell, distance_band, signal_quality)
        
        This captures:
        - Spatial position (which part of map)
        - Coverage (which tower serving)
        - Progress (how far to goal)
        - Connectivity (signal quality)
        """
        grid_x, grid_y = self.position_to_grid(self.current_pos[0], self.current_pos[1])
        cell = int(self.serving_cell) if self.serving_cell is not None else 0
        
        distance = np.linalg.norm(self.current_pos[:2] - self.destination[:2])
        dist_band = self.get_distance_band(distance)
        
        signal_qual = self.get_signal_quality()
        
        return (grid_x, grid_y, cell, dist_band, signal_qual)
    
    # ============ ACTION SELECTION ============
    
    def choose_action(self, state):
        """Epsilon-greedy with goal-directed initialization"""
        if state not in self.q_table:
            # Initialize with heuristic toward goal
            self.q_table[state] = {}
            
            dx = self.destination[0] - self.current_pos[0]
            dy = self.destination[1] - self.current_pos[1]
            
            for action in self.actions:
                # Give initial bias toward goal direction
                if action == 'northeast' and dx > 0 and dy > 0:
                    self.q_table[state][action] = 10.0
                elif action == 'southeast' and dx > 0 and dy <= 0:
                    self.q_table[state][action] = 10.0
                elif action == 'northwest' and dx <= 0 and dy > 0:
                    self.q_table[state][action] = 10.0
                elif action == 'southwest' and dx <= 0 and dy <= 0:
                    self.q_table[state][action] = 10.0
                elif action in ['north', 'south', 'east', 'west']:
                    self.q_table[state][action] = 5.0
                else:
                    self.q_table[state][action] = 0.0
        
        # Epsilon-greedy
        if np.random.random() < self.epsilon:
            return np.random.choice(self.actions)
        else:
            return max(self.q_table[state], key=self.q_table[state].get)
    
    # ============ HANDOVER DETECTION ============
    
    def detect_ping_pong(self, new_cell):
        """
        Detect ping-pong handovers:
        Pattern: A -> B -> A (back and forth between same two cells)
        """
        if len(self.handover_history) < 2:
            return False
        
        # Check if we're going back to the cell before last
        if len(self.handover_history) >= 2:
            prev_cell = self.handover_history[-1]
            prev_prev_cell = self.handover_history[-2]
            
            if new_cell == prev_prev_cell and new_cell != prev_cell:
                return True
        
        return False
    
    def process_handover(self, new_cell_id):
        """
        Process handover event from NS-3
        Called when /current_cell_id message is received
        """
        # Check if this is actually a handover (cell changed)
        if self.serving_cell is not None and new_cell_id != self.serving_cell:
            # Detect ping-pong
            is_ping_pong = self.detect_ping_pong(new_cell_id)
            
            # Log handover
            if is_ping_pong:
                self.episode_ping_pongs += 1
                self.get_logger().info(
                    f"PING-PONG HO#{self.episode_handovers + 1}: "
                    f"eNB{self.serving_cell} ↔ eNB{new_cell_id} "
                    f"at ({self.current_pos[0]:.0f},{self.current_pos[1]:.0f})"
                )
            else:
                self.get_logger().info(
                    f"HANDOVER #{self.episode_handovers + 1}: "
                    f"eNB{self.serving_cell} → eNB{new_cell_id} "
                    f"at ({self.current_pos[0]:.0f},{self.current_pos[1]:.0f})"
                )
            
            # Update history
            self.handover_history.append(self.serving_cell)
            self.episode_handovers += 1
            
            # Return penalty for reward calculation
            return is_ping_pong
        
        return False
    
    # ============ REWARD FUNCTION ============
    
    def calculate_reward(self):
        """
        COMPREHENSIVE REWARD FUNCTION:
        1. Progress toward goal (potential-based shaping)
        2. Connectivity quality bonus/penalty
        3. Handover penalties (only for ping-pong)
        4. Altitude tracking
        5. Heading alignment
        6. Goal achievement bonus
        """
        reward = 0.0
        
        # ===== 1. PROGRESS TOWARD GOAL =====
        old_dist = np.linalg.norm(self.last_pos[:2] - self.destination[:2])
        new_dist = np.linalg.norm(self.current_pos[:2] - self.destination[:2])
        delta_dist = old_dist - new_dist
        
        # Scale bonus based on distance (encourage early progress)
        if new_dist > 80:
            reward += 30.0 * delta_dist
        elif new_dist > 50:
            reward += 25.0 * delta_dist
        elif new_dist > 20:
            reward += 20.0 * delta_dist
        else:
            reward += 15.0 * delta_dist
        
        # Penalty for moving away
        if delta_dist < -0.5:
            reward -= 8.0
        
        # ===== 2. CONNECTIVITY QUALITY =====
        if self.rsrp_values and self.serving_cell is not None and self.serving_cell < len(self.rsrp_values):
            rsrp = self.rsrp_values[int(self.serving_cell)]
            
            # Track RSRP for episode average
            self.rsrp_history.append(rsrp)
            
            if rsrp > -85:  # Excellent
                reward += 2.0
                self.good_signal_steps += 1
            elif rsrp > -90:  # Good
                reward += 1.0
                self.good_signal_steps += 1
            elif rsrp > -95:  # Fair
                reward += 0.0
            else:  # Poor (< -95 dBm)
                reward -= 3.0  # Penalty for poor signal
                self.poor_signal_steps += 1
        
        # ===== 3. HANDOVER PENALTIES (handled via event, applied here) =====
        # Ping-pong penalty is applied when handover is processed
        # No penalty for normal handovers
        
        # ===== 4. ALTITUDE TRACKING =====
        target_alt = 25.0
        old_alt_err = abs(self.last_pos[2] - target_alt)
        new_alt_err = abs(self.current_pos[2] - target_alt)
        reward += 2.0 * (old_alt_err - new_alt_err)
        reward -= 0.02 * new_alt_err
        
        # ===== 5. HEADING ALIGNMENT =====
        step_vec = self.current_pos[:2] - self.last_pos[:2]
        goal_vec = self.destination[:2] - self.last_pos[:2]
        ns = np.linalg.norm(step_vec)
        ng = np.linalg.norm(goal_vec)
        
        if ns > 1e-6 and ng > 1e-6:
            cos_heading = float(np.dot(step_vec, goal_vec) / (ns * ng))
            reward += 2.0 * cos_heading
        
        # ===== 6. GOAL ACHIEVEMENT =====
        if new_dist < self.GOAL_THRESHOLD:
            # Base goal bonus
            reward += 500.0
            
            # EFFICIENCY BONUS: Fewer handovers = higher bonus
            if self.episode_handovers == 0:
                reward += 200.0  # Perfect connectivity!
            elif self.episode_handovers <= 2:
                reward += 100.0  # Excellent
            elif self.episode_handovers <= 5:
                reward += 50.0   # Good
            # No bonus if > 5 handovers
            
            # Penalty for ping-pongs even at goal
            reward -= 30.0 * self.episode_ping_pongs
            
            self.get_logger().info(f"GOAL! HO:{self.episode_handovers} PP:{self.episode_ping_pongs}")
        
        # ===== 7. TIME PENALTY =====
        reward -= 0.15
        
        return float(reward)
    
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
        
        # Altitude control
        target_alt = 25.0
        alt_error = target_alt - self.current_pos[2]
        cmd.linear.z = float(np.clip(alt_error * 1.0, -3.0, 3.0))
        
        self.cmd_pub.publish(cmd)
    
    # ============ ROS CALLBACKS ============
    
    def pose_callback(self, msg):
        """Position update"""
        for transform in msg.transforms:
            if 'X3' in transform.child_frame_id:
                self.current_pos[0] = transform.transform.translation.x
                self.current_pos[1] = transform.transform.translation.y
                self.current_pos[2] = transform.transform.translation.z
                
                if not self.position_received:
                    self.position_received = True
                    self.episode_start_time = self.get_clock().now()
                    self.episode_active = True
                    
                    distance = np.linalg.norm(self.current_pos[:2] - self.destination[:2])
                    self.get_logger().info(f"Episode {self.current_episode+1} started | Distance: {distance:.0f}m")
    
    def rsrp_callback(self, msg):
        """RSRP update - store values for all cells"""
        self.rsrp_values = list(msg.data)
        
        # If we don't have a serving cell yet (first callback), use strongest
        if self.serving_cell is None and len(self.rsrp_values) > 0:
            self.serving_cell = int(np.argmax(self.rsrp_values))
            self.get_logger().info(f"Initial cell: eNB{self.serving_cell}")
    
    def handover_callback(self, msg):
        """
        NEW: Handover notification from NS-3
        Called when UE connects to a new cell
        """
        new_cell_id = int(msg.data)
        
        # Only process if episode is active
        if not self.episode_active:
            return
        
        # Process handover (detect ping-pong, update counters)
        is_ping_pong = self.process_handover(new_cell_id)
        
        # Apply ping-pong penalty immediately
        if is_ping_pong:
            self.episode_reward -= 50.0
        
        # Update serving cell
        self.serving_cell = new_cell_id
    
    # ============ MAIN CONTROL LOOP ============
    
    def control_loop(self):
        """Main loop at 5 Hz"""
        if not self.position_received or not self.episode_active:
            return
        
        distance = float(np.linalg.norm(self.current_pos[:2] - self.destination[:2]))
        
        # ===== CHECK TERMINATION CONDITIONS =====
        
        # 1. Goal reached
        if distance < self.GOAL_THRESHOLD:
            self.end_episode(reached_goal=True)
            return
        
        # 2. Progress-based timeout (only check when window is full)
        self.progress_window.append(distance)
        
        if len(self.progress_window) == self.progress_window.maxlen:
            # Check if making progress over last 8 seconds (40 samples at 5Hz)
            progress = self.progress_window[0] - self.progress_window[-1]
            
            # Only timeout if virtually NO progress AND we've been running a while
            if progress < 0.5 and self.current_step > 100:
                self.get_logger().info(f"No progress timeout (moved {progress:.1f}m in 8s)")
                self.end_episode(reached_goal=False)
                return
        
        # 3. Hard step limit - 100 seconds
        if self.current_step >= 500:
            self.get_logger().info("Max steps timeout")
            self.end_episode(reached_goal=False)
            return
        
        # ===== EXECUTE RL STEP =====
        
        state = self.get_state()
        action = self.choose_action(state)
        self.execute_action(action)

        # Store max 10 waypoints evenly distributed throughout episode
        # Calculate interval based on max expected steps (500) / 10 waypoints = every 50 steps
        if len(self.episode_waypoints) < 10 and self.current_step % 50 == 0:
            self.episode_waypoints.append([float(self.current_pos[0]), float(self.current_pos[1]), float(self.current_pos[2])])
        
        reward = self.calculate_reward()
        self.episode_reward += reward
        
        # Q-learning update
        if self.last_state is not None and self.last_action is not None:
            next_state = state
            if next_state not in self.q_table:
                self.q_table[next_state] = {a: 0.0 for a in self.actions}
            
            current_q = self.q_table[self.last_state][self.last_action]
            max_next_q = max(self.q_table[next_state].values())
            new_q = current_q + self.alpha * (reward + self.gamma * max_next_q - current_q)
            self.q_table[self.last_state][self.last_action] = new_q
        
        self.last_state = state
        self.last_action = action
        self.last_pos = self.current_pos.copy()
        self.current_step += 1
        
        # ===== LOGGING =====
        if self.current_step % 25 == 0:
            progress_pct = 100 * (1 - distance / np.linalg.norm(self.destination - self.source))
            
            # RSRP info
            if self.rsrp_values and self.serving_cell is not None and self.serving_cell < len(self.rsrp_values):
                rsrp = self.rsrp_values[int(self.serving_cell)]
                qual = self.get_signal_quality().upper()
                rsrp_str = f"RSRP:{rsrp:.1f}[{qual}] T{self.serving_cell}"
            else:
                rsrp_str = "RSRP:N/A"
            
            self.get_logger().info(
                f"[Ep{self.current_episode+1} S{self.current_step}] "
                f"Pos:({self.current_pos[0]:.0f},{self.current_pos[1]:.0f}) | "
                f"Dist:{distance:.0f}m ({progress_pct:.0f}%) | "
                f"Act:{action.upper()} | "
                f"HO:{self.episode_handovers} PP:{self.episode_ping_pongs} | "
                f"{rsrp_str}"
            )
    
    
    def reset_uav_in_gazebo(self):
        """Reset UAV position in Gazebo"""
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
        """End current episode"""
        self.episode_active = False
        self.cmd_pub.publish(Twist())  # Stop
        
        elapsed = (self.get_clock().now() - self.episode_start_time).nanoseconds / 1e9
        distance = np.linalg.norm(self.current_pos[:2] - self.destination[:2])
        
        # Calculate metrics
        total_distance = np.linalg.norm(self.destination - self.source)
        progress = total_distance - distance
        progress_pct = 100 * progress / total_distance
        
        # Calculate average RSRP for this episode
        avg_episode_rsrp = np.mean(self.rsrp_history) if self.rsrp_history else -100.0
        
        # BONUS: Only compare against successful episodes' RSRP
        connectivity_bonus = 0.0
        if reached_goal:  # Only for successful episodes
            if avg_episode_rsrp > self.best_avg_rsrp_successful:
                # Scale bonus based on improvement (max 100 points)
                improvement = avg_episode_rsrp - self.best_avg_rsrp_successful
                connectivity_bonus = min(100.0, improvement * 10.0)
                self.episode_reward += connectivity_bonus
                self.best_avg_rsrp_successful = avg_episode_rsrp
                self.get_logger().info(f"New best avg RSRP (successful): {avg_episode_rsrp:.1f} dBm (+{connectivity_bonus:.0f} bonus)")
        
        # PENALTY: Multiple handovers (more than 1)
        handover_penalty = 0.0
        if self.episode_handovers > 1:
            handover_penalty = (self.episode_handovers - 1) * 15.0  # 15 points per extra handover
            self.episode_reward -= handover_penalty
            self.get_logger().info(f"Multiple handovers penalty: -{handover_penalty:.0f} ({self.episode_handovers} HOs)")
        
        # Print summary
        self.get_logger().info("\n" + "=" * 60)
        self.get_logger().info(f"Episode {self.current_episode+1}/{self.num_episodes}: {'SUCCESS!' if reached_goal else 'TIMEOUT'}")
        self.get_logger().info(f"  Time: {elapsed:.1f}s | Steps: {self.current_step}")
        self.get_logger().info(f"  Reward: {int(self.episode_reward)}")
        self.get_logger().info(f"  Progress: {progress:.1f}m ({progress_pct:.0f}%)")
        self.get_logger().info(f"  Final dist: {distance:.1f}m")
        self.get_logger().info(f"  Handovers: {self.episode_handovers} (Ping-pong: {self.episode_ping_pongs})")
        
        if self.rsrp_history:
            self.get_logger().info(f"  Avg RSRP: {avg_episode_rsrp:.1f} dBm (best successful: {self.best_avg_rsrp_successful:.1f})")
            if connectivity_bonus > 0:
                self.get_logger().info(f"  Connectivity bonus: +{connectivity_bonus:.0f}")
        
        if handover_penalty > 0:
            self.get_logger().info(f"  Handover penalty: -{handover_penalty:.0f}")
        
        self.get_logger().info(f"  Good signal: {self.good_signal_steps} steps | Poor: {self.poor_signal_steps} steps")
        self.get_logger().info(f"  Q-table: {len(self.q_table)} states | ε: {self.epsilon:.3f}")
        self.get_logger().info("=" * 60)
        
        # Save history
        self.training_history.append({
            'episode': int(self.current_episode + 1),
            'success': bool(reached_goal),
            'reward': float(self.episode_reward),
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
            'poor_signal_steps': int(self.poor_signal_steps),
            'connectivity_bonus': float(connectivity_bonus),
            'handover_penalty': float(handover_penalty),
            'epsilon': float(self.epsilon),
            'q_states': int(len(self.q_table)),
            'waypoints': self.episode_waypoints
        })

        self.current_episode += 1
    
        if self.current_episode < self.num_episodes:
            # Reset for next episode
            self.current_step = 0
            self.episode_reward = 0.0
            self.episode_handovers = 0
            self.episode_ping_pongs = 0
            self.good_signal_steps = 0
            self.poor_signal_steps = 0
            self.position_received = False
            self.last_state = None
            self.last_action = None
            self.current_pos = self.source.copy()
            self.last_pos = self.source.copy()
            self.progress_window.clear()
            self.handover_history.clear()
            self.rsrp_history = []
            self.episode_waypoints = []
            
            # Reset cell tracking
            self.serving_cell = None
            self.previous_cell = None
            
            # Decay epsilon
            self.epsilon = max(self.epsilon_min, self.epsilon * self.epsilon_decay)
            
            # Reset UAV
            self.get_logger().info(f"\nResetting for Episode {self.current_episode+1}...")
            reset_success = self.reset_uav_in_gazebo()
            
            if reset_success:
                self.get_logger().info("Reset successful!")
            else:
                self.get_logger().warn("Reset failed, waiting...")
            
            import time
            time.sleep(3.0)
            self.get_logger().info(f"Starting Episode {self.current_episode+1}...\n")
        else:
            self.finish_training()

    def finish_training(self):
        """Training complete"""
        self.get_logger().info("\n" + "=" * 70)
        self.get_logger().info("TRAINING COMPLETE!")
        self.get_logger().info("=" * 70)
        
        # Save results
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        try:
            with open(f'/app/models/q_table_{timestamp}.pkl', 'wb') as f:
                pickle.dump(self.q_table, f)
            self.get_logger().info(f"Saved: /app/models/q_table_{timestamp}.pkl")
            
            with open(f'/app/logs/history_{timestamp}.json', 'w') as f:
                json.dump(self.training_history, f, indent=2)
            self.get_logger().info(f"Saved: /app/logs/history_{timestamp}.json")
        except Exception as e:
            self.get_logger().error(f"Save error: {e}")
        
        # Print summary
        successes = [ep for ep in self.training_history if ep['success']]
        
        self.get_logger().info(f"\nSUMMARY:")
        self.get_logger().info(f"  Total success rate: {len(successes)}/{len(self.training_history)} ({100*len(successes)/len(self.training_history):.1f}%)")
        self.get_logger().info(f"  Avg handovers: {np.mean([ep['handovers'] for ep in self.training_history]):.1f}")
        self.get_logger().info(f"  Avg ping-pongs: {np.mean([ep['ping_pong'] for ep in self.training_history]):.1f}")
        self.get_logger().info(f"  Avg RSRP: {np.mean([ep['avg_rsrp'] for ep in self.training_history]):.1f} dBm")
        self.get_logger().info(f"  Q-table size: {len(self.q_table)} states")
        
        # First 10 vs Last 10
        if len(self.training_history) >= 20:
            first_10 = self.training_history[:10]
            last_10 = self.training_history[-10:]
            
            self.get_logger().info(f"\nLEARNING PROGRESS (first 10 vs last 10):")
            self.get_logger().info(f"  Success rate: {sum(1 for ep in first_10 if ep['success'])}/10 -> {sum(1 for ep in last_10 if ep['success'])}/10")
            self.get_logger().info(f"  Handovers: {np.mean([ep['handovers'] for ep in first_10]):.1f} -> {np.mean([ep['handovers'] for ep in last_10]):.1f}")
            self.get_logger().info(f"  Ping-pongs: {np.mean([ep['ping_pong'] for ep in first_10]):.1f} -> {np.mean([ep['ping_pong'] for ep in last_10]):.1f}")
            self.get_logger().info(f"  Avg RSRP: {np.mean([ep['avg_rsrp'] for ep in first_10]):.1f} -> {np.mean([ep['avg_rsrp'] for ep in last_10]):.1f} dBm")
        
        self.get_logger().info("=" * 70)
        
        self.get_logger().info("Shutting down in 5 seconds...")
        import time
        time.sleep(5)
        rclpy.shutdown()
    
def main(args=None):
    parser = argparse.ArgumentParser(description='Connectivity-Aware RL Pilot')
    parser.add_argument('--episodes', type=int, default=50, help='Number of training episodes')
    parsed_args, _ = parser.parse_known_args()
    rclpy.init(args=args)
    node = ConnectivityAwareRLPilot(episodes=parsed_args.episodes)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("\nInterrupted by user")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == 'main':
    main()

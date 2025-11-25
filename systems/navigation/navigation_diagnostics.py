#!/usr/bin/env python3
"""
Navigation Diagnostics Logger
Captures detailed navigation data for debugging and analysis
Logs person detection, force calculations, movement commands, and decision rationale
"""
import csv
import time
import math
import os
from collections import deque
from typing import Dict, Any, List, Tuple, Optional

class NavigationDiagnosticsLogger:
    """
    Comprehensive navigation diagnostics logger
    Creates detailed CSV logs for analyzing navigation behavior
    """
    
    def __init__(self, log_filename="NAVIGATION_DIAGNOSTICS.csv"):
        self.log_filename = log_filename
        self.log_initialized = False
        self.start_time = time.time()
        
        # Diagnostic data buffers
        self.recent_decisions = deque(maxlen=100)
        self.anomaly_log = []
        
        # Tracking for anomaly detection
        self.last_person_angle = None
        self.last_movement_direction = None
        self.direction_change_count = 0
        self.consecutive_same_direction = 0
        
        # Initialize the log file
        self._initialize_log()
    
    def _initialize_log(self):
        """Initialize CSV log with headers"""
        try:
            with open(self.log_filename, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([
                    # Timestamp
                    'timestamp',
                    'elapsed_seconds',
                    
                    # Person Detection (raw from camera/LiDAR transform)
                    'person_detected',
                    'person_lidar_angle_deg',
                    'person_lidar_distance_mm',
                    'person_confidence',
                    
                    # Robot State
                    'robot_pos_x',
                    'robot_pos_y', 
                    'robot_imu_orientation_deg',
                    
                    # Person Target (calculated world position)
                    'target_x',
                    'target_y',
                    
                    # Force Calculations
                    'attractive_force_x',
                    'attractive_force_y',
                    'attractive_magnitude',
                    'repulsive_force_x',
                    'repulsive_force_y',
                    'repulsive_magnitude',
                    'total_force_x',
                    'total_force_y',
                    'total_force_magnitude',
                    
                    # Force Direction Analysis
                    'force_direction_deg',
                    'force_direction_relative_to_robot',
                    
                    # Movement Command Output
                    'movement_direction',
                    'movement_speed',
                    
                    # Decision Analysis
                    'expected_direction',
                    'direction_matches_expected',
                    'decision_rationale',
                    
                    # Obstacle Context
                    'num_obstacles_total',
                    'num_obstacles_filtered',
                    'closest_obstacle_angle',
                    'closest_obstacle_distance',
                    
                    # Navigation State
                    'nav_state',
                    'goal_reached',
                    
                    # Anomaly Flags
                    'anomaly_detected',
                    'anomaly_type'
                ])
            self.log_initialized = True
        except Exception as e:
            print(f"Failed to initialize navigation diagnostics log: {e}")
            self.log_initialized = False
    
    def calculate_expected_direction(self, person_lidar_angle: float) -> str:
        """
        Calculate what direction the robot SHOULD move based on person angle
        LiDAR angle convention: 0 = directly in front, positive = right, negative = left
        """
        if person_lidar_angle is None:
            return "NONE"
        
        # Normalize angle to -180 to 180
        angle = person_lidar_angle
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        
        abs_angle = abs(angle)
        
        # Determine expected direction based on where person is
        if abs_angle <= 30:
            return "FORWARDS"
        elif abs_angle >= 150:
            return "BACKWARDS"
        elif angle > 30 and angle < 150:
            # Person is to the right
            if abs_angle <= 60:
                return "FORWARDS_RIGHT"
            else:
                return "RIGHT"
        elif angle < -30 and angle > -150:
            # Person is to the left
            if abs_angle <= 60:
                return "FORWARDS_LEFT"
            else:
                return "LEFT"
        else:
            return "UNKNOWN"
    
    def analyze_decision(self, person_angle: float, actual_direction: str, 
                        forces: Dict[str, Tuple[float, float]]) -> Tuple[str, bool, str]:
        """
        Analyze if the navigation decision makes sense
        Returns: (expected_direction, matches, rationale)
        """
        expected = self.calculate_expected_direction(person_angle)
        
        # Check if actual matches expected
        matches = False
        rationale = ""
        
        if actual_direction == "NONE":
            if forces['total'][0] == 0 and forces['total'][1] == 0:
                rationale = "No force calculated - no movement"
                matches = True
            else:
                rationale = f"Force exists but no movement commanded"
        elif expected == actual_direction:
            matches = True
            rationale = "Direction matches expected based on person angle"
        elif expected == "FORWARDS" and actual_direction in ["FORWARDS_LEFT", "FORWARDS_RIGHT"]:
            matches = True
            rationale = "Slight deviation from forward - likely obstacle avoidance"
        elif expected in ["FORWARDS_LEFT", "FORWARDS_RIGHT"] and actual_direction == "FORWARDS":
            matches = True
            rationale = "Moving forward instead of diagonal - acceptable"
        else:
            # Mismatch - analyze why
            total_fx, total_fy = forces['total']
            attr_fx, attr_fy = forces['attractive']
            rep_fx, rep_fy = forces['repulsive']
            
            attr_mag = math.sqrt(attr_fx**2 + attr_fy**2)
            rep_mag = math.sqrt(rep_fx**2 + rep_fy**2)
            
            if rep_mag > attr_mag:
                rationale = f"Repulsive force ({rep_mag:.2f}) > Attractive ({attr_mag:.2f}) - obstacle avoidance dominates"
            elif attr_mag < 0.1:
                rationale = f"Attractive force too weak ({attr_mag:.2f}) - person too far or at goal?"
            else:
                # Calculate force direction
                force_angle = math.degrees(math.atan2(total_fy, total_fx))
                rationale = f"MISMATCH: Person at {person_angle:.1f}deg, force points {force_angle:.1f}deg, got {actual_direction}"
        
        return expected, matches, rationale
    
    def detect_anomalies(self, person_angle: float, actual_direction: str,
                        expected_direction: str, matches: bool) -> Tuple[bool, str]:
        """Detect navigation anomalies"""
        anomaly = False
        anomaly_type = ""
        
        # Track direction changes
        if self.last_movement_direction and actual_direction != "NONE":
            if actual_direction != self.last_movement_direction:
                self.direction_change_count += 1
                self.consecutive_same_direction = 0
                
                # Rapid direction changes are suspicious
                if self.direction_change_count > 5:
                    anomaly = True
                    anomaly_type = "RAPID_DIRECTION_CHANGES"
            else:
                self.consecutive_same_direction += 1
                self.direction_change_count = max(0, self.direction_change_count - 1)
        
        # Wrong direction anomaly
        if not matches and actual_direction != "NONE" and expected_direction != "UNKNOWN":
            # Check if completely opposite
            opposite_pairs = [
                ("FORWARDS", "BACKWARDS"),
                ("LEFT", "RIGHT"),
                ("FORWARDS_LEFT", "BACKWARDS_RIGHT"),
                ("FORWARDS_RIGHT", "BACKWARDS_LEFT")
            ]
            for pair in opposite_pairs:
                if (expected_direction == pair[0] and actual_direction == pair[1]) or \
                   (expected_direction == pair[1] and actual_direction == pair[0]):
                    anomaly = True
                    anomaly_type = f"OPPOSITE_DIRECTION: expected {expected_direction}, got {actual_direction}"
                    break
        
        # Person angle jump anomaly
        if self.last_person_angle is not None and person_angle is not None:
            angle_change = abs(person_angle - self.last_person_angle)
            if angle_change > 180:
                angle_change = 360 - angle_change
            if angle_change > 45:  # Large sudden jump
                anomaly = True
                anomaly_type = f"LARGE_ANGLE_JUMP: {self.last_person_angle:.1f} to {person_angle:.1f}"
        
        self.last_person_angle = person_angle
        self.last_movement_direction = actual_direction
        
        return anomaly, anomaly_type
    
    def log_navigation_cycle(self, 
                            person_data: Optional[Dict[str, Any]],
                            person_lidar_angle: float,
                            person_lidar_distance: float,
                            robot_state: Dict[str, Any],
                            person_target: Optional[Dict[str, Any]],
                            forces: Dict[str, Tuple[float, float]],
                            movement_command: Optional[Dict[str, str]],
                            obstacles: List[Tuple[float, float]],
                            obstacles_filtered_count: int,
                            nav_state: str,
                            goal_reached: bool):
        """Log a complete navigation decision cycle"""
        
        if not self.log_initialized:
            return
        
        try:
            current_time = time.time()
            elapsed = current_time - self.start_time
            
            # Extract person detection info
            person_detected = person_data is not None and person_lidar_distance > 0
            person_confidence = person_data.get('confidence', 0.0) if person_data else 0.0
            
            # Extract robot state
            robot_x = robot_state.get('position_x', 0.0)
            robot_y = robot_state.get('position_y', 0.0)
            robot_orientation = robot_state.get('orientation', 0.0)
            
            # Extract target position
            target_x = person_target['x'] if person_target else 0.0
            target_y = person_target['y'] if person_target else 0.0
            
            # Extract forces
            attr_fx, attr_fy = forces.get('attractive', (0.0, 0.0))
            rep_fx, rep_fy = forces.get('repulsive', (0.0, 0.0))
            total_fx, total_fy = forces.get('total', (0.0, 0.0))
            
            attr_mag = math.sqrt(attr_fx**2 + attr_fy**2)
            rep_mag = math.sqrt(rep_fx**2 + rep_fy**2)
            total_mag = math.sqrt(total_fx**2 + total_fy**2)
            
            # Calculate force direction
            force_dir_deg = math.degrees(math.atan2(total_fy, total_fx)) if total_mag > 0 else 0.0
            force_dir_relative = force_dir_deg - robot_orientation
            while force_dir_relative > 180:
                force_dir_relative -= 360
            while force_dir_relative < -180:
                force_dir_relative += 360
            
            # Extract movement command
            move_direction = movement_command.get('direction', 'NONE') if movement_command else 'NONE'
            move_speed = movement_command.get('speed', 0.0) if movement_command else 0.0
            
            # Analyze decision
            expected_dir, matches, rationale = self.analyze_decision(
                person_lidar_angle if person_detected else None,
                move_direction,
                forces
            )
            
            # Detect anomalies
            anomaly, anomaly_type = self.detect_anomalies(
                person_lidar_angle if person_detected else None,
                move_direction,
                expected_dir,
                matches
            )
            
            # Find closest obstacle
            closest_obs_angle = 0.0
            closest_obs_dist = 0.0
            if obstacles:
                closest = min(obstacles, key=lambda o: o[1])
                closest_obs_angle = closest[0]
                closest_obs_dist = closest[1]
            
            # Write to CSV
            with open(self.log_filename, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([
                    current_time,
                    f"{elapsed:.3f}",
                    
                    person_detected,
                    f"{person_lidar_angle:.2f}" if person_detected else "N/A",
                    f"{person_lidar_distance:.1f}" if person_detected else "N/A",
                    f"{person_confidence:.3f}",
                    
                    f"{robot_x:.1f}",
                    f"{robot_y:.1f}",
                    f"{robot_orientation:.2f}",
                    
                    f"{target_x:.1f}",
                    f"{target_y:.1f}",
                    
                    f"{attr_fx:.4f}",
                    f"{attr_fy:.4f}",
                    f"{attr_mag:.4f}",
                    f"{rep_fx:.4f}",
                    f"{rep_fy:.4f}",
                    f"{rep_mag:.4f}",
                    f"{total_fx:.4f}",
                    f"{total_fy:.4f}",
                    f"{total_mag:.4f}",
                    
                    f"{force_dir_deg:.2f}",
                    f"{force_dir_relative:.2f}",
                    
                    move_direction,
                    f"{move_speed:.3f}",
                    
                    expected_dir,
                    matches,
                    rationale,
                    
                    len(obstacles),
                    obstacles_filtered_count,
                    f"{closest_obs_angle:.1f}" if obstacles else "N/A",
                    f"{closest_obs_dist:.1f}" if obstacles else "N/A",
                    
                    nav_state,
                    goal_reached,
                    
                    anomaly,
                    anomaly_type if anomaly else ""
                ])
            
            # Store recent decision for analysis
            self.recent_decisions.append({
                'time': elapsed,
                'person_angle': person_lidar_angle if person_detected else None,
                'expected': expected_dir,
                'actual': move_direction,
                'matches': matches,
                'anomaly': anomaly
            })
            
            # Log anomalies separately for quick review
            if anomaly:
                self.anomaly_log.append({
                    'time': elapsed,
                    'type': anomaly_type,
                    'person_angle': person_lidar_angle,
                    'expected': expected_dir,
                    'actual': move_direction,
                    'forces': forces
                })
                
        except Exception as e:
            pass  # Don't let logging errors affect navigation
    
    def get_summary(self) -> Dict[str, Any]:
        """Get summary of navigation diagnostics"""
        if not self.recent_decisions:
            return {'total_decisions': 0}
        
        decisions = list(self.recent_decisions)
        total = len(decisions)
        matches = sum(1 for d in decisions if d['matches'])
        anomalies = sum(1 for d in decisions if d['anomaly'])
        
        return {
            'total_decisions': total,
            'correct_decisions': matches,
            'accuracy_pct': (matches / total * 100) if total > 0 else 0,
            'anomaly_count': anomalies,
            'anomaly_rate_pct': (anomalies / total * 100) if total > 0 else 0,
            'recent_anomalies': self.anomaly_log[-5:] if self.anomaly_log else []
        }
    
    def write_anomaly_report(self, filename="NAVIGATION_ANOMALIES.txt"):
        """Write a human-readable anomaly report"""
        try:
            with open(filename, 'w') as f:
                f.write("NAVIGATION ANOMALY REPORT\n")
                f.write("=" * 60 + "\n\n")
                
                summary = self.get_summary()
                f.write(f"Total Navigation Decisions: {summary['total_decisions']}\n")
                f.write(f"Correct Decisions: {summary['correct_decisions']} ({summary['accuracy_pct']:.1f}%)\n")
                f.write(f"Anomalies Detected: {summary['anomaly_count']} ({summary['anomaly_rate_pct']:.1f}%)\n\n")
                
                if self.anomaly_log:
                    f.write("ANOMALY DETAILS:\n")
                    f.write("-" * 60 + "\n")
                    for i, anomaly in enumerate(self.anomaly_log):
                        f.write(f"\nAnomaly #{i+1} at {anomaly['time']:.2f}s:\n")
                        f.write(f"  Type: {anomaly['type']}\n")
                        f.write(f"  Person Angle: {anomaly['person_angle']}\n")
                        f.write(f"  Expected Direction: {anomaly['expected']}\n")
                        f.write(f"  Actual Direction: {anomaly['actual']}\n")
                        if 'forces' in anomaly:
                            forces = anomaly['forces']
                            f.write(f"  Attractive Force: ({forces['attractive'][0]:.3f}, {forces['attractive'][1]:.3f})\n")
                            f.write(f"  Repulsive Force: ({forces['repulsive'][0]:.3f}, {forces['repulsive'][1]:.3f})\n")
                            f.write(f"  Total Force: ({forces['total'][0]:.3f}, {forces['total'][1]:.3f})\n")
                else:
                    f.write("No anomalies detected.\n")
                    
        except Exception as e:
            print(f"Failed to write anomaly report: {e}")
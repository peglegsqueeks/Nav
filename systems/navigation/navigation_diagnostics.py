#!/usr/bin/env python3
"""
Navigation Diagnostics Logger - Comprehensive Feedback System
Provides detailed CSV logging for debugging Potential Field Navigation

PURPOSE: Enable efficient Develop/Test/Feedback/Develop loop by capturing
all critical data needed to understand navigation behavior without verbal descriptions.

OUTPUT FILES:
1. nav_diagnostics_TIMESTAMP.csv - High-frequency navigation math logging
2. nav_events_TIMESTAMP.csv - State transitions and anomalies
3. nav_summary_TIMESTAMP.txt - End-of-session analysis report

KEY METRICS CAPTURED:
- Force calculations (attractive, repulsive, total) with full vector data
- Coordinate verification (person angle vs force direction)
- IMU state and potential drift detection
- Movement command validation
- Anomaly detection and flagging
"""
import os
import csv
import time
import math
from datetime import datetime
from collections import deque
from typing import Dict, Any, List, Tuple, Optional


class NavigationDiagnosticsLogger:
    """
    Comprehensive diagnostics logger for Potential Field Navigation debugging.
    Captures all data needed to understand and fix navigation issues.
    
    Test Modes:
    - TEST1: Direct forward target, no obstacles
    - TEST2: Forward target with obstacle bypass
    """
    
    # Test mode definitions
    TEST_MODES = {
        0: "GENERAL",
        1: "TEST1_DIRECT_FORWARD",
        2: "TEST2_OBSTACLE_BYPASS"
    }
    
    def __init__(self, output_dir: str = "nav_logs"):
        # Create output directory
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)
        
        # Test mode (can be set before navigation starts)
        self.test_mode = 0
        self.test_mode_name = "GENERAL"
        
        # Generate timestamp for this session
        self.session_timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # File paths (will be updated when test mode is set)
        self.diagnostics_file = os.path.join(output_dir, f"nav_diagnostics_{self.session_timestamp}.csv")
        self.events_file = os.path.join(output_dir, f"nav_events_{self.session_timestamp}.csv")
        self.summary_file = os.path.join(output_dir, f"nav_summary_{self.session_timestamp}.txt")
        
        # Initialize CSV files
        self._init_diagnostics_csv()
        self._init_events_csv()
        
        # Session tracking
        self.session_start_time = time.time()
        self.log_count = 0
        self.event_count = 0
        
        # Anomaly tracking
        self.anomalies = []
        self.direction_mismatches = 0
        self.stuck_events = 0
        self.force_zero_events = 0
        
        # Historical data for analysis
        self.force_history = deque(maxlen=1000)
        self.direction_history = deque(maxlen=1000)
        self.imu_history = deque(maxlen=1000)
        self.position_history = deque(maxlen=1000)
        
        # Previous state for change detection
        self.prev_nav_state = None
        self.prev_direction = None
        self.prev_person_detected = False
        
        # IMU drift detection
        self.imu_baseline_yaw = None
        self.imu_drift_samples = deque(maxlen=100)
        
        print(f"Navigation Diagnostics initialized:")
        print(f"  Diagnostics: {self.diagnostics_file}")
        print(f"  Events: {self.events_file}")
        print(f"  Summary: {self.summary_file}")
        print(f"  Test Mode: {self.test_mode_name}")
        print(f"  Press 1 or 2 to set test mode before enabling navigation")
    
    def set_test_mode(self, mode: int):
        """Set the test mode and update file names accordingly"""
        if mode in self.TEST_MODES:
            self.test_mode = mode
            self.test_mode_name = self.TEST_MODES[mode]
            
            # Update file names to include test mode
            self.diagnostics_file = os.path.join(
                self.output_dir, 
                f"nav_diagnostics_{self.test_mode_name}_{self.session_timestamp}.csv"
            )
            self.events_file = os.path.join(
                self.output_dir, 
                f"nav_events_{self.test_mode_name}_{self.session_timestamp}.csv"
            )
            self.summary_file = os.path.join(
                self.output_dir, 
                f"nav_summary_{self.test_mode_name}_{self.session_timestamp}.txt"
            )
            
            # Re-initialize CSV files with new names
            self._init_diagnostics_csv()
            self._init_events_csv()
            
            # Log the mode change as an event
            self._log_event('TEST_MODE_SET', f'Test mode set to {self.test_mode_name}',
                           False, 0, 0, 'NONE', 0)
            
            print(f"\n{'='*50}")
            print(f"TEST MODE SET: {self.test_mode_name}")
            print(f"Files will be saved as:")
            print(f"  {self.diagnostics_file}")
            print(f"  {self.summary_file}")
            print(f"{'='*50}\n")
            
            return True
        return False
    
    def get_test_mode_name(self) -> str:
        """Get current test mode name"""
        return self.test_mode_name
    
    def _init_diagnostics_csv(self):
        """Initialize the main diagnostics CSV with comprehensive headers"""
        headers = [
            # Timestamp
            'timestamp',
            'elapsed_time',
            'log_number',
            'test_mode',
            
            # Person Detection Input
            'person_detected',
            'person_x_mm',
            'person_y_mm',
            'person_distance_mm',
            'person_angle_deg',
            'person_confidence',
            'time_since_person_update_ms',
            
            # Robot State Input
            'robot_x_mm',
            'robot_y_mm',
            'robot_orientation_deg',
            
            # IMU State (Critical for debugging)
            'imu_yaw_raw',
            'imu_yaw_smoothed',
            'imu_roll',
            'imu_pitch',
            'imu_angular_vel_z',
            'imu_update_rate',
            'imu_drift_estimate',
            
            # Attractive Force Calculation
            'attr_force_x',
            'attr_force_y',
            'attr_force_magnitude',
            'attr_force_direction_deg',
            
            # Repulsive Force Calculation
            'repul_force_x',
            'repul_force_y',
            'repul_force_magnitude',
            'repul_force_direction_deg',
            'num_obstacles_in_range',
            'closest_obstacle_dist_mm',
            'closest_obstacle_angle_deg',
            
            # Total Force Result
            'total_force_x',
            'total_force_y',
            'total_force_magnitude',
            'total_force_direction_deg',
            
            # Direction Validation (CRITICAL)
            'expected_direction_to_person_deg',
            'force_direction_deg',
            'direction_error_deg',
            'direction_mismatch_flag',
            
            # Movement Command Output
            'commanded_direction',
            'commanded_speed',
            'motor_1_speed',
            'motor_2_speed',
            
            # Navigation State
            'nav_state',
            'nav_state_changed',
            'goal_reached',
            'stuck_detected',
            'time_in_current_state_s',
            
            # Anomaly Flags
            'anomaly_detected',
            'anomaly_type',
            
            # Performance
            'calculation_time_ms',
            'obstacles_filtered_for_person'
        ]
        
        with open(self.diagnostics_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(headers)
    
    def _init_events_csv(self):
        """Initialize events CSV for state transitions and anomalies"""
        headers = [
            'timestamp',
            'elapsed_time',
            'event_type',
            'event_description',
            'prev_state',
            'new_state',
            'person_detected',
            'person_distance_mm',
            'force_magnitude',
            'commanded_direction',
            'imu_yaw',
            'additional_data'
        ]
        
        with open(self.events_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(headers)
    
    def log_navigation_cycle(self, 
                             person_data: Optional[Dict],
                             robot_state: Dict,
                             imu_data: Dict,
                             attractive_force: Tuple[float, float],
                             repulsive_force: Tuple[float, float],
                             total_force: Tuple[float, float],
                             obstacles: List[Tuple[float, float]],
                             movement_command: Optional[Dict],
                             nav_state: str,
                             goal_reached: bool,
                             stuck_detected: bool,
                             obstacles_filtered: int = 0,
                             calculation_time_ms: float = 0.0):
        """
        Log a complete navigation cycle with all inputs, calculations, and outputs.
        This is the main logging function called every navigation update.
        """
        current_time = time.time()
        elapsed = current_time - self.session_start_time
        self.log_count += 1
        
        # Extract person data
        person_detected = person_data is not None and person_data.get('detected', False)
        person_x = person_data.get('lidar_x_mm', 0) if person_data else 0
        person_y = person_data.get('lidar_y_mm', 0) if person_data else 0
        person_distance = person_data.get('distance_mm', 0) if person_data else 0
        person_angle = person_data.get('lidar_angle_deg', 0) if person_data else 0
        person_confidence = person_data.get('confidence', 0) if person_data else 0
        time_since_person = person_data.get('age_ms', 0) if person_data else 0
        
        # Robot state
        robot_x = robot_state.get('position_x', 0)
        robot_y = robot_state.get('position_y', 0)
        robot_orientation = robot_state.get('orientation', 0)
        
        # IMU data
        imu_yaw_raw = imu_data.get('imu_yaw', 0)
        imu_yaw_smoothed = imu_data.get('imu_smoothed_yaw', 0)
        imu_roll = imu_data.get('imu_roll', 0)
        imu_pitch = imu_data.get('imu_pitch', 0)
        imu_angular_vel_z = imu_data.get('imu_angular_vel_z', 0)
        imu_update_rate = imu_data.get('imu_update_rate', 0)
        
        # Calculate IMU drift estimate
        imu_drift = self._estimate_imu_drift(imu_yaw_smoothed)
        
        # Force calculations
        # COORDINATE SYSTEM: 0° = Forward (+Y), 90° = Right (+X)
        # Use atan2(x, y) to get angle from forward axis
        attr_mag = math.sqrt(attractive_force[0]**2 + attractive_force[1]**2)
        attr_dir = math.degrees(math.atan2(attractive_force[0], attractive_force[1])) if attr_mag > 0.01 else 0
        
        repul_mag = math.sqrt(repulsive_force[0]**2 + repulsive_force[1]**2)
        repul_dir = math.degrees(math.atan2(repulsive_force[0], repulsive_force[1])) if repul_mag > 0.01 else 0
        
        total_mag = math.sqrt(total_force[0]**2 + total_force[1]**2)
        total_dir = math.degrees(math.atan2(total_force[0], total_force[1])) if total_mag > 0.01 else 0
        
        # Obstacle analysis
        obstacles_in_range = len([o for o in obstacles if o[1] < 800]) if obstacles else 0
        closest_dist = min([o[1] for o in obstacles], default=9999) if obstacles else 9999
        closest_angle = obstacles[0][0] if obstacles and len(obstacles) > 0 else 0
        
        # Direction validation (CRITICAL FOR DEBUGGING)
        # Convert person_angle from 0-360 to -180 to 180 for proper comparison
        person_angle_normalized = self._normalize_angle(person_angle)
        expected_dir_to_person = person_angle_normalized if person_detected else 0
        direction_error = self._normalize_angle(total_dir - expected_dir_to_person) if person_detected and total_mag > 0.1 else 0
        
        # Flag direction mismatch (force should generally point toward person)
        direction_mismatch = False
        if person_detected and total_mag > 0.1 and abs(direction_error) > 90:
            direction_mismatch = True
            self.direction_mismatches += 1
        
        # Movement command
        cmd_direction = movement_command.get('direction', 'NONE') if movement_command else 'NONE'
        cmd_speed = movement_command.get('speed', 0) if movement_command else 0
        motor_1 = movement_command.get('motor_1', 0) if movement_command else 0
        motor_2 = movement_command.get('motor_2', 0) if movement_command else 0
        
        # State change detection
        nav_state_changed = nav_state != self.prev_nav_state
        if nav_state_changed:
            self._log_event('STATE_CHANGE', f'{self.prev_nav_state} -> {nav_state}',
                           person_detected, person_distance, total_mag, cmd_direction, imu_yaw_smoothed)
            self.prev_nav_state = nav_state
        
        # Anomaly detection
        anomaly_detected, anomaly_type = self._detect_anomalies(
            person_detected, person_angle, total_dir, total_mag, 
            cmd_direction, stuck_detected, direction_mismatch
        )
        
        # Track for analysis
        self.force_history.append({
            'time': current_time,
            'attr_mag': attr_mag,
            'repul_mag': repul_mag,
            'total_mag': total_mag,
            'total_dir': total_dir
        })
        
        self.direction_history.append({
            'time': current_time,
            'person_angle': person_angle,
            'force_dir': total_dir,
            'cmd_dir': cmd_direction,
            'error': direction_error
        })
        
        self.imu_history.append({
            'time': current_time,
            'yaw': imu_yaw_smoothed,
            'drift': imu_drift
        })
        
        # Write to CSV
        row = [
            current_time,
            f"{elapsed:.3f}",
            self.log_count,
            self.test_mode_name,
            
            person_detected,
            f"{person_x:.1f}",
            f"{person_y:.1f}",
            f"{person_distance:.1f}",
            f"{person_angle:.2f}",
            f"{person_confidence:.3f}",
            f"{time_since_person:.1f}",
            
            f"{robot_x:.1f}",
            f"{robot_y:.1f}",
            f"{robot_orientation:.2f}",
            
            f"{imu_yaw_raw:.2f}",
            f"{imu_yaw_smoothed:.2f}",
            f"{imu_roll:.2f}",
            f"{imu_pitch:.2f}",
            f"{imu_angular_vel_z:.4f}",
            f"{imu_update_rate:.1f}",
            f"{imu_drift:.3f}",
            
            f"{attractive_force[0]:.4f}",
            f"{attractive_force[1]:.4f}",
            f"{attr_mag:.4f}",
            f"{attr_dir:.2f}",
            
            f"{repulsive_force[0]:.4f}",
            f"{repulsive_force[1]:.4f}",
            f"{repul_mag:.4f}",
            f"{repul_dir:.2f}",
            obstacles_in_range,
            f"{closest_dist:.1f}",
            f"{closest_angle:.1f}",
            
            f"{total_force[0]:.4f}",
            f"{total_force[1]:.4f}",
            f"{total_mag:.4f}",
            f"{total_dir:.2f}",
            
            f"{expected_dir_to_person:.2f}",
            f"{total_dir:.2f}",
            f"{direction_error:.2f}",
            direction_mismatch,
            
            cmd_direction,
            f"{cmd_speed:.3f}",
            motor_1,
            motor_2,
            
            nav_state,
            nav_state_changed,
            goal_reached,
            stuck_detected,
            0,  # time_in_current_state placeholder
            
            anomaly_detected,
            anomaly_type,
            
            f"{calculation_time_ms:.2f}",
            obstacles_filtered
        ]
        
        with open(self.diagnostics_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(row)
    
    def _estimate_imu_drift(self, current_yaw: float) -> float:
        """Estimate IMU drift by tracking yaw changes when robot should be stationary"""
        if self.imu_baseline_yaw is None:
            self.imu_baseline_yaw = current_yaw
            return 0.0
        
        self.imu_drift_samples.append(current_yaw - self.imu_baseline_yaw)
        
        if len(self.imu_drift_samples) > 10:
            return sum(self.imu_drift_samples) / len(self.imu_drift_samples)
        return 0.0
    
    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to -180 to 180 range"""
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle
    
    def _detect_anomalies(self, person_detected: bool, person_angle: float,
                          force_dir: float, force_mag: float, cmd_direction: str,
                          stuck: bool, direction_mismatch: bool) -> Tuple[bool, str]:
        """Detect navigation anomalies that indicate problems"""
        anomalies = []
        
        # Normalize person_angle to -180 to 180 for proper comparison
        person_angle_norm = self._normalize_angle(person_angle)
        
        # Anomaly 1: Force pointing away from person
        if direction_mismatch:
            anomalies.append("FORCE_AWAY_FROM_PERSON")
        
        # Anomaly 2: Person detected but no force generated
        if person_detected and force_mag < 0.1:
            anomalies.append("NO_FORCE_WITH_PERSON")
            self.force_zero_events += 1
        
        # Anomaly 3: Moving backward when person is in front (within ±45° of forward)
        if person_detected and abs(person_angle_norm) < 45 and cmd_direction == "BACKWARDS":
            anomalies.append("BACKWARD_TOWARD_FRONT_PERSON")
        
        # Anomaly 4: Turning wrong way (person on right but turning left, or vice versa)
        # Positive angle = person to the right, Negative angle = person to the left
        if person_detected and person_angle_norm > 30 and cmd_direction == "LEFT":
            anomalies.append("TURNING_WRONG_WAY")
        if person_detected and person_angle_norm < -30 and cmd_direction == "RIGHT":
            anomalies.append("TURNING_WRONG_WAY")
        
        # Anomaly 5: Stuck condition
        if stuck:
            anomalies.append("STUCK_DETECTED")
            self.stuck_events += 1
        
        if anomalies:
            anomaly_str = "|".join(anomalies)
            self.anomalies.append({
                'time': time.time(),
                'types': anomalies,
                'person_angle': person_angle,
                'force_dir': force_dir,
                'cmd_dir': cmd_direction
            })
            return True, anomaly_str
        
        return False, ""
    
    def _log_event(self, event_type: str, description: str,
                   person_detected: bool, person_distance: float,
                   force_mag: float, cmd_direction: str, imu_yaw: float,
                   additional_data: str = ""):
        """Log a significant event"""
        current_time = time.time()
        elapsed = current_time - self.session_start_time
        self.event_count += 1
        
        row = [
            current_time,
            f"{elapsed:.3f}",
            event_type,
            description,
            self.prev_nav_state or "NONE",
            "",  # new_state filled by caller context
            person_detected,
            f"{person_distance:.1f}",
            f"{force_mag:.4f}",
            cmd_direction,
            f"{imu_yaw:.2f}",
            additional_data
        ]
        
        with open(self.events_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(row)
    
    def log_anomaly_event(self, anomaly_type: str, details: str):
        """Log a detected anomaly as an event"""
        self._log_event('ANOMALY', f"{anomaly_type}: {details}", 
                       False, 0, 0, "NONE", 0, details)
    
    def write_summary_report(self):
        """Write end-of-session summary report with analysis"""
        elapsed = time.time() - self.session_start_time
        
        with open(self.summary_file, 'w') as f:
            f.write("=" * 70 + "\n")
            f.write("NAVIGATION DIAGNOSTICS SUMMARY REPORT\n")
            f.write(f"Session: {self.session_timestamp}\n")
            f.write(f"Test Mode: {self.test_mode_name}\n")
            f.write(f"Duration: {elapsed:.1f} seconds ({elapsed/60:.1f} minutes)\n")
            f.write("=" * 70 + "\n\n")
            
            # Test mode specific expectations
            if self.test_mode == 1:
                f.write("TEST 1: DIRECT FORWARD TARGET (No Obstacles)\n")
                f.write("-" * 40 + "\n")
                f.write("Expected: Robot moves FORWARD, direction error < 30°\n")
                f.write("Expected: No repulsive forces, no direction mismatches\n\n")
            elif self.test_mode == 2:
                f.write("TEST 2: OBSTACLE BYPASS\n")
                f.write("-" * 40 + "\n")
                f.write("Expected: Robot navigates AROUND obstacle\n")
                f.write("Expected: Repulsive forces active near obstacle\n")
                f.write("Expected: No stuck detection, successful bypass\n\n")
            
            # Basic Statistics
            f.write("BASIC STATISTICS\n")
            f.write("-" * 40 + "\n")
            f.write(f"Total log entries: {self.log_count}\n")
            f.write(f"Total events logged: {self.event_count}\n")
            f.write(f"Log rate: {self.log_count/max(1,elapsed):.1f} Hz\n\n")
            
            # Anomaly Summary
            f.write("ANOMALY SUMMARY\n")
            f.write("-" * 40 + "\n")
            f.write(f"Total anomalies detected: {len(self.anomalies)}\n")
            f.write(f"Direction mismatches: {self.direction_mismatches}\n")
            f.write(f"Stuck events: {self.stuck_events}\n")
            f.write(f"Zero force with person: {self.force_zero_events}\n")
            
            # Categorize anomalies
            anomaly_types = {}
            for a in self.anomalies:
                for t in a['types']:
                    anomaly_types[t] = anomaly_types.get(t, 0) + 1
            
            if anomaly_types:
                f.write("\nAnomaly breakdown:\n")
                for atype, count in sorted(anomaly_types.items(), key=lambda x: -x[1]):
                    f.write(f"  {atype}: {count}\n")
            f.write("\n")
            
            # Force Analysis
            f.write("FORCE ANALYSIS\n")
            f.write("-" * 40 + "\n")
            if self.force_history:
                attr_mags = [fh['attr_mag'] for fh in self.force_history]
                repul_mags = [fh['repul_mag'] for fh in self.force_history]
                total_mags = [fh['total_mag'] for fh in self.force_history]
                
                f.write(f"Attractive force - Avg: {sum(attr_mags)/len(attr_mags):.3f}, "
                       f"Max: {max(attr_mags):.3f}, Min: {min(attr_mags):.3f}\n")
                f.write(f"Repulsive force  - Avg: {sum(repul_mags)/len(repul_mags):.3f}, "
                       f"Max: {max(repul_mags):.3f}, Min: {min(repul_mags):.3f}\n")
                f.write(f"Total force      - Avg: {sum(total_mags)/len(total_mags):.3f}, "
                       f"Max: {max(total_mags):.3f}, Min: {min(total_mags):.3f}\n")
                
                zero_force_pct = len([m for m in total_mags if m < 0.1]) / len(total_mags) * 100
                f.write(f"Zero force percentage: {zero_force_pct:.1f}%\n")
            f.write("\n")
            
            # Direction Error Analysis
            f.write("DIRECTION ERROR ANALYSIS\n")
            f.write("-" * 40 + "\n")
            if self.direction_history:
                errors = [abs(dh['error']) for dh in self.direction_history if dh['person_angle'] != 0]
                if errors:
                    f.write(f"Direction error - Avg: {sum(errors)/len(errors):.1f}°, "
                           f"Max: {max(errors):.1f}°, Min: {min(errors):.1f}°\n")
                    
                    large_errors = len([e for e in errors if e > 45])
                    f.write(f"Large errors (>45°): {large_errors} ({large_errors/len(errors)*100:.1f}%)\n")
                    
                    very_large_errors = len([e for e in errors if e > 90])
                    f.write(f"Very large errors (>90°): {very_large_errors} ({very_large_errors/len(errors)*100:.1f}%)\n")
            f.write("\n")
            
            # IMU Analysis
            f.write("IMU ANALYSIS\n")
            f.write("-" * 40 + "\n")
            if self.imu_history:
                yaws = [ih['yaw'] for ih in self.imu_history]
                drifts = [ih['drift'] for ih in self.imu_history]
                
                f.write(f"Yaw range: {min(yaws):.1f}° to {max(yaws):.1f}° (span: {max(yaws)-min(yaws):.1f}°)\n")
                f.write(f"Estimated drift: {drifts[-1] if drifts else 0:.2f}°\n")
                
                # Check for IMU issues
                yaw_variance = sum((y - sum(yaws)/len(yaws))**2 for y in yaws) / len(yaws)
                f.write(f"Yaw variance: {yaw_variance:.2f}\n")
                
                if abs(drifts[-1] if drifts else 0) > 5:
                    f.write("WARNING: Significant IMU drift detected!\n")
            f.write("\n")
            
            # Key Issues Identified
            f.write("KEY ISSUES IDENTIFIED\n")
            f.write("-" * 40 + "\n")
            
            issues = []
            
            if self.direction_mismatches > 10:
                issues.append("CRITICAL: Frequent direction mismatches - force pointing away from person")
            
            if self.force_zero_events > 10:
                issues.append("CRITICAL: Force calculations returning zero when person detected")
            
            if self.stuck_events > 5:
                issues.append("WARNING: Multiple stuck events detected")
            
            if self.imu_history and abs(self.imu_history[-1]['drift']) > 10:
                issues.append("WARNING: Significant IMU drift may affect navigation")
            
            if self.direction_history:
                errors = [abs(dh['error']) for dh in self.direction_history if dh['person_angle'] != 0]
                if errors and sum(errors)/len(errors) > 45:
                    issues.append("CRITICAL: Average direction error > 45° - coordinate system issue likely")
            
            if issues:
                for issue in issues:
                    f.write(f"• {issue}\n")
            else:
                f.write("No critical issues identified.\n")
            
            f.write("\n")
            
            # Test-specific pass/fail analysis
            f.write("TEST RESULT ANALYSIS\n")
            f.write("-" * 40 + "\n")
            
            test_passed = True
            test_failures = []
            
            if self.test_mode == 1:
                # Test 1: Direct forward - should have minimal direction errors
                if self.direction_history:
                    errors = [abs(dh['error']) for dh in self.direction_history if dh['person_angle'] != 0]
                    if errors:
                        avg_error = sum(errors) / len(errors)
                        if avg_error > 30:
                            test_passed = False
                            test_failures.append(f"Average direction error {avg_error:.1f}° exceeds 30° limit")
                
                if self.direction_mismatches > 5:
                    test_passed = False
                    test_failures.append(f"{self.direction_mismatches} direction mismatches detected")
                
                # Check if robot moved forward
                if self.direction_history:
                    forward_cmds = len([d for d in self.direction_history if d['cmd_dir'] == 'FORWARDS'])
                    backward_cmds = len([d for d in self.direction_history if d['cmd_dir'] == 'BACKWARDS'])
                    if backward_cmds > forward_cmds:
                        test_passed = False
                        test_failures.append(f"Robot moved BACKWARD more than FORWARD ({backward_cmds} vs {forward_cmds})")
                
            elif self.test_mode == 2:
                # Test 2: Obstacle bypass - should have repulsive forces and no stuck
                if self.force_history:
                    max_repulsive = max([fh['repul_mag'] for fh in self.force_history])
                    if max_repulsive < 0.1:
                        test_passed = False
                        test_failures.append("No significant repulsive forces detected - obstacle avoidance may not be working")
                
                if self.stuck_events > 3:
                    test_passed = False
                    test_failures.append(f"{self.stuck_events} stuck events - robot may be in local minimum")
            
            if test_passed:
                f.write(f"✓ TEST {self.test_mode} ({self.test_mode_name}): PASSED\n")
            else:
                f.write(f"✗ TEST {self.test_mode} ({self.test_mode_name}): FAILED\n")
                for failure in test_failures:
                    f.write(f"  - {failure}\n")
            
            f.write("\n")
            f.write("=" * 70 + "\n")
            f.write("END OF REPORT\n")
            f.write("=" * 70 + "\n")
        
        print(f"\nSummary report written to: {self.summary_file}")
    
    def write_anomaly_report(self):
        """Write detailed anomaly report for debugging specific issues"""
        if not self.anomalies:
            return
        
        anomaly_file = os.path.join(self.output_dir, f"nav_anomalies_{self.session_timestamp}.txt")
        
        with open(anomaly_file, 'w') as f:
            f.write("DETAILED ANOMALY REPORT\n")
            f.write("=" * 60 + "\n\n")
            
            for i, anomaly in enumerate(self.anomalies[:100]):  # Limit to first 100
                f.write(f"Anomaly #{i+1}\n")
                f.write(f"  Time: {anomaly['time'] - self.session_start_time:.2f}s\n")
                f.write(f"  Types: {', '.join(anomaly['types'])}\n")
                f.write(f"  Person angle: {anomaly['person_angle']:.1f}°\n")
                f.write(f"  Force direction: {anomaly['force_dir']:.1f}°\n")
                f.write(f"  Command: {anomaly['cmd_dir']}\n")
                f.write("\n")
        
        print(f"Anomaly report written to: {anomaly_file}")


class QuickDiagnostics:
    """
    Lightweight diagnostics for quick status checks during development.
    Prints periodic summaries to console without full CSV overhead.
    """
    
    def __init__(self, report_interval: float = 5.0):
        self.report_interval = report_interval
        self.last_report_time = time.time()
        
        # Counters
        self.cycles = 0
        self.person_detections = 0
        self.movements_forward = 0
        self.movements_backward = 0
        self.movements_left = 0
        self.movements_right = 0
        self.movements_none = 0
        self.anomaly_count = 0
        
        # Recent values
        self.recent_force_mags = deque(maxlen=50)
        self.recent_direction_errors = deque(maxlen=50)
    
    def update(self, person_detected: bool, force_mag: float, 
               direction_error: float, cmd_direction: str, anomaly: bool):
        """Update quick diagnostics"""
        self.cycles += 1
        
        if person_detected:
            self.person_detections += 1
        
        if cmd_direction == "FORWARDS":
            self.movements_forward += 1
        elif cmd_direction == "BACKWARDS":
            self.movements_backward += 1
        elif cmd_direction == "LEFT":
            self.movements_left += 1
        elif cmd_direction == "RIGHT":
            self.movements_right += 1
        else:
            self.movements_none += 1
        
        if anomaly:
            self.anomaly_count += 1
        
        self.recent_force_mags.append(force_mag)
        self.recent_direction_errors.append(abs(direction_error))
        
        # Periodic report
        current_time = time.time()
        if current_time - self.last_report_time >= self.report_interval:
            self._print_report()
            self.last_report_time = current_time
    
    def _print_report(self):
        """Print quick status report to console"""
        avg_force = sum(self.recent_force_mags) / max(1, len(self.recent_force_mags))
        avg_error = sum(self.recent_direction_errors) / max(1, len(self.recent_direction_errors))
        
        print("\n" + "=" * 50)
        print("NAVIGATION QUICK STATUS")
        print("=" * 50)
        print(f"Cycles: {self.cycles} | Person detections: {self.person_detections}")
        print(f"Movements - F:{self.movements_forward} B:{self.movements_backward} "
              f"L:{self.movements_left} R:{self.movements_right} None:{self.movements_none}")
        print(f"Avg force: {avg_force:.3f} | Avg direction error: {avg_error:.1f}°")
        print(f"Anomalies: {self.anomaly_count}")
        print("=" * 50 + "\n")
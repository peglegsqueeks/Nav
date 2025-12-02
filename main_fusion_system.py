#!/usr/bin/env python3
"""
Main Coordinate Sensor Fusion System with Movement Integration
Enhanced Coordinate Sensor Fusion System with robot movement infrastructure

VERSION HISTORY:
v2.0.1 (2025-12-02 09:00): REVERTED SCREEN_X CHANGES
- REVERTED: Removed screen_x_normalized calculation and parameter passing
- Person detection restored to previous working state
- Reacquisition head tracking defaults to 'center' (no screen position)

v2.0 (2025-12-01 16:00): STOP AND REACQUIRE METHODOLOGY
- Integrated reacquisition control: head tracking + body rotation
- Head turns toward last screen position (left/right/center)
- Body rotates toward pre-avoidance orientation
- Passes screen X position to navigator for head direction
- Robot STOPS completely during reacquisition (no forward motion)

v1.1 (2025-12-01 15:45): FASTER MANUAL CONTROL
- Arrow key speed: 0.5 → 0.85 (70% faster manual navigation)

FEATURES:
- Critical person obstacle filtering for navigation
- Navigation diagnostics logging for debugging
- Test mode support (Press 1 or 2 to set test mode)
- Manual arrow key control with release detection
- Automatic reacquisition via head tracking and body rotation
"""
import pygame
import math
import time
import csv
import os
import statistics
import numpy as np
from collections import deque

# Import system modules
from systems.movement.movement_types import MovementDirection, VelocityConfig
from systems.movement.velocity_manager import EnhancedDirectVelocityManager
from systems.movement.robot_state_tracker import RobotStateTracker
from systems.sensors.detection_system import EnhancedRgbReuseDetectionSystem
from systems.sensors.lidar_system import OptimizedLidarSystem
from systems.control.head_controller import OptimizedNoValidationHeadController
from systems.navigation.potential_field_navigator import PotentialFieldNavigator
from systems.navigation.navigation_diagnostics import NavigationDiagnosticsLogger


class EnhancedCoordinateSensorFusionWithMovement:
    """ENHANCED Coordinate Sensor Fusion System with robot movement infrastructure and navigation"""
    
    def __init__(self):
        # Display setup
        self.screen = None
        self.center_x = 0
        self.center_y = 0
        self.scale = 0
        self.initialized = False
        
        # Systems
        self.detection_system = None
        self.lidar_system = None
        self.head_controller = None
        self.velocity_manager = None
        self.robot_state_tracker = None
        self.navigator = None
        
        # Navigation diagnostics logger
        self.nav_diagnostics = NavigationDiagnosticsLogger()
        
        # Tracking data
        self.detection_count = 0
        self.valid_x_pixels = deque(maxlen=100)
        self.current_variance_pixels = 0.0
        self.current_std_dev_pixels = 0.0
        self.current_mean_pixels = 0.0
        self.all_valid_x_pixels = deque(maxlen=1000)
        self.average_variance_pixels = 0.0
        
        # Detection performance tracking
        self.detection_success_count = 0
        self.detection_failure_count = 0
        self.total_detections_attempted = 0
        self.detection_rate = 0.0
        
        # Stable display data
        self.stable_person_x = 0
        self.last_valid_person_x = 0
        
        # FPS monitoring
        self.fps_counter = deque(maxlen=30)
        self.last_frame_time = time.time()
        self.current_fps = 0.0
        
        # Display control
        self.update_counter = 0
        self.display_update_rate = 3
        self.running = True
        
        # CSV logging
        self.csv_log_filename = "ENHANCED_MOVEMENT_TRACKING_LOG.csv"
        self.csv_initialized = False
        self.mode_start_time = time.time()
        
        # Calibration system
        self.calibration_mode = False
        self.calibration_csv_filename = "CALIBRATION_DATA.csv"
        self.calibration_samples = []
        self.calibration_marker_x = 0
        self.calibration_marker_y = 0
        self.current_dynamic_offset_x = 0.0
        self.current_dynamic_offset_y = 0.0
        
        # Sample feedback
        self.sample_taken_feedback = False
        self.sample_feedback_timer = 0.0
        self.sample_feedback_duration = 2.0
        
        # Control modes
        self.movement_enabled = False
        self.navigation_enabled = False
        self.show_navigation_forces = False
        
        # Person filtering tracking
        self.person_lidar_angle = 0.0
        self.person_lidar_distance = 0.0
        self.obstacles_filtered_count = 0
        
        # Initialize pygame
        if not pygame.get_init():
            pygame.init()
        pygame.font.init()
    
    def filter_person_obstacles(self, obstacles, person_lidar_angle, person_lidar_distance, exclusion_radius=200.0):
        """
        CRITICAL: Remove LiDAR obstacles within exclusion_radius of detected person position
        This prevents the person from being treated as an obstacle to avoid
        """
        if not obstacles or person_lidar_distance <= 0:
            return obstacles
            
        filtered_obstacles = []
        filtered_count = 0
        
        for obs_angle, obs_distance in obstacles:
            angle_diff = abs(obs_angle - person_lidar_angle)
            if angle_diff > 180:
                angle_diff = 360 - angle_diff
            
            try:
                angle_diff_rad = math.radians(angle_diff)
                distance_between = math.sqrt(
                    obs_distance**2 + person_lidar_distance**2 - 
                    2 * obs_distance * person_lidar_distance * math.cos(angle_diff_rad)
                )
                
                if distance_between > exclusion_radius:
                    filtered_obstacles.append((obs_angle, obs_distance))
                else:
                    filtered_count += 1
                    
            except Exception:
                filtered_obstacles.append((obs_angle, obs_distance))
        
        self.obstacles_filtered_count = filtered_count
        return filtered_obstacles
    
    def load_calibration_data(self):
        """Load calibration data from CSV file"""
        try:
            if os.path.exists(self.calibration_csv_filename):
                with open(self.calibration_csv_filename, 'r', newline='') as csvfile:
                    reader = csv.DictReader(csvfile)
                    self.calibration_samples = []
                    for row in reader:
                        try:
                            sample = {
                                'distance': float(row['distance']),
                                'lateral_angle': float(row['lateral_angle']),
                                'person_x': float(row['person_x']),
                                'person_y': float(row['person_y']),
                                'offset_x': float(row['offset_x']),
                                'offset_y': float(row['offset_y'])
                            }
                            self.calibration_samples.append(sample)
                        except Exception:
                            pass
        except Exception:
            self.calibration_samples = []
    
    def calculate_dynamic_offset(self, person_distance, person_angle, base_x=0, base_y=0):
        """Calculate dynamic offset based on calibration samples"""
        try:
            if not self.calibration_samples:
                return 0.0, 0.0
            
            weighted_offset_x = 0.0
            weighted_offset_y = 0.0
            total_weight = 0.0
            
            for sample in self.calibration_samples:
                dist_diff = abs(sample['distance'] - person_distance)
                angle_diff = abs(sample['lateral_angle'] - person_angle)
                weight = 1.0 / (1.0 + (dist_diff / 1000.0) + (angle_diff / 10.0))
                weighted_offset_x += sample['offset_x'] * weight
                weighted_offset_y += sample['offset_y'] * weight
                total_weight += weight
            
            if total_weight > 0:
                calculated_x = weighted_offset_x / total_weight
                calculated_y = weighted_offset_y / total_weight
                return calculated_x, calculated_y
            else:
                return 0.0, 0.0
        except Exception:
            return 0.0, 0.0

    def calculate_fps(self):
        """Calculate FPS"""
        current_time = time.time()
        frame_time = current_time - self.last_frame_time
        self.last_frame_time = current_time
        
        if frame_time > 0:
            fps = 1.0 / frame_time
            self.fps_counter.append(fps)
            if len(self.fps_counter) > 0:
                self.current_fps = sum(self.fps_counter) / len(self.fps_counter)
    
    def initialize_all_systems(self):
        """Initialize all systems"""
        if self.initialized:
            return True
        
        try:
            # Initialize display
            display_info = pygame.display.Info()
            self.screen = pygame.display.set_mode((display_info.current_w, display_info.current_h), pygame.FULLSCREEN)
            pygame.display.set_caption("ENHANCED MOVEMENT TRACKING - Robot Base Control Integration")
            pygame.mouse.set_visible(False)
            
            self.center_x = display_info.current_w // 2
            self.center_y = display_info.current_h // 2
            self.scale = min(display_info.current_w, display_info.current_h) // 7
            
            self.calibration_marker_x = self.center_x
            self.calibration_marker_y = self.center_y
            
            self.draw_stable_interface()
            pygame.display.flip()
            
            # Initialize head controller
            self.head_controller = OptimizedNoValidationHeadController()
            servo_init_success = self.head_controller.initialize_servo()
            
            if servo_init_success:
                self.head_controller.center_servo()
                time.sleep(0.5)
            
            # Initialize movement systems
            self.velocity_manager = EnhancedDirectVelocityManager(simulate=False)
            self.robot_state_tracker = RobotStateTracker()
            
            # Initialize navigation system with error handling
            try:
                self.navigator = PotentialFieldNavigator()
                self.navigator.set_debug_mode(False)
            except Exception as e:
                print(f"Navigation system initialization failed: {e}")
                self.navigator = None
            
            # Initialize detection system
            self.detection_system = EnhancedRgbReuseDetectionSystem()
            if not self.detection_system.initialize():
                self.detection_system = None
            
            # Initialize LiDAR system
            try:
                self.lidar_system = OptimizedLidarSystem()
                if not self.lidar_system.start():
                    self.lidar_system = None
                else:
                    time.sleep(3)
            except Exception:
                self.lidar_system = None
            
            # Initialize CSV logging
            self.initialize_enhanced_csv_log()
            
            # Load calibration data
            self.load_calibration_data()
            
            self.initialized = True
            return True
            
        except Exception as e:
            print(f"Initialization error: {e}")
            return False
    
    def initialize_enhanced_csv_log(self):
        """Initialize CSV logging"""
        try:
            with open(self.csv_log_filename, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([
                    'timestamp', 'mode_time_elapsed', 'fps',
                    'person_detected', 'person_x_pixels', 'person_z_camera', 'person_confidence',
                    'servo_position', 'servo_is_moving', 'person_centered',
                    'pixel_error', 'total_moves', 'successful_centers', 'false_positive_rejections',
                    'detection_success_count', 'detection_failure_count', 'detection_rate',
                    'consecutive_successes', 'consecutive_failures',
                    'rolling_variance_pixels', 'rolling_std_dev_pixels', 'average_variance_pixels',
                    'reused_detection', 'last_good_age_ms',
                    'smoothing_applied', 'smoothed_position_x', 'raw_position_x',
                    'dynamic_offset_x', 'dynamic_offset_y', 'calibration_samples_count',
                    'movement_enabled', 'base_is_moving', 'base_direction', 'base_speed',
                    'movement_success_rate', 'total_movement_commands', 'avg_movement_duration',
                    'robot_pos_x', 'robot_pos_y', 'robot_orientation', 'robot_velocity_x', 'robot_velocity_y',
                    'total_distance_traveled', 'total_rotation', 'position_confidence', 'orientation_confidence',
                    'imu_initialized', 'imu_roll', 'imu_pitch', 'imu_yaw', 'imu_smoothed_yaw',
                    'imu_angular_vel_x', 'imu_angular_vel_y', 'imu_angular_vel_z',
                    'imu_accel_x', 'imu_accel_y', 'imu_accel_z', 'imu_update_rate',
                    'navigation_enabled', 'navigation_state', 'has_person_target', 'goal_reached',
                    'person_distance_mm', 'person_angle_deg', 'time_since_person_update',
                    'attractive_force_magnitude', 'repulsive_force_magnitude', 'total_force_magnitude',
                    'nav_movement_direction', 'nav_movement_speed', 'navigation_success_rate',
                    'goals_reached', 'force_calculations_count', 'obstacles_filtered_count'
                ])
            self.csv_initialized = True
        except Exception:
            self.csv_initialized = False
    
    def get_person_detection(self):
        """Get person detection"""
        if not self.detection_system:
            self.detection_failure_count += 1
            return None
        
        self.total_detections_attempted += 1
        
        detection = self.detection_system.get_detection()
        if detection:
            screen_width = self.screen.get_width()
            screen_height = self.screen.get_height()
            x_pixels = int(detection['bbox_center']['x_normalized'] * screen_width)
            y_pixels = int(detection['bbox_center']['y_normalized'] * screen_height)
            
            detection['bbox_center']['x_pixels'] = x_pixels
            detection['bbox_center']['y_pixels'] = y_pixels
            
            self.stable_person_x = x_pixels
            self.last_valid_person_x = x_pixels
            self.detection_success_count += 1
            
            if self.total_detections_attempted > 0:
                self.detection_rate = (self.detection_success_count / self.total_detections_attempted) * 100
            
            return detection
        else:
            self.detection_failure_count += 1
            if self.total_detections_attempted > 0:
                self.detection_rate = (self.detection_success_count / self.total_detections_attempted) * 100
            return None
    
    def calculate_x_midpoint_variance(self, person_detected, x_pixels=None):
        """Calculate variance for valid detections only"""
        try:
            if person_detected and x_pixels is not None:
                self.valid_x_pixels.append(x_pixels)
                self.all_valid_x_pixels.append(x_pixels)
            
            if len(self.valid_x_pixels) >= 2:
                window_data = list(self.valid_x_pixels)
                mean_val = statistics.mean(window_data)
                variance_val = statistics.variance(window_data)
                std_dev_val = math.sqrt(variance_val)
                self.current_variance_pixels = variance_val
                self.current_std_dev_pixels = std_dev_val
                self.current_mean_pixels = mean_val
            else:
                self.current_variance_pixels = 0.0
                self.current_std_dev_pixels = 0.0
                self.current_mean_pixels = x_pixels if x_pixels else 0.0
            
            if len(self.all_valid_x_pixels) >= 2:
                overall_variance = statistics.variance(list(self.all_valid_x_pixels))
                self.average_variance_pixels = overall_variance
            else:
                self.average_variance_pixels = 0.0
            
            return self.current_variance_pixels, self.current_std_dev_pixels, self.current_mean_pixels
        except Exception:
            return 0.0, 0.0, 0.0
    
    def update_all_systems(self, person_data=None, obstacles=None):
        """Update all systems with navigation and diagnostics"""
        if self.detection_system:
            self.detection_system.update_imu_data()
        
        movement_status = self.velocity_manager.get_movement_status() if self.velocity_manager else {
            'is_moving': False, 'current_direction': 'NONE', 'current_speed': 0.0,
            'motor_initialized': False, 'total_movement_commands': 0, 'movement_success_rate': 0.0,
            'avg_movement_duration': 0.0
        }
        
        imu_status = self.detection_system.get_imu_status() if self.detection_system else {
            'imu_initialized': False, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0, 'smoothed_yaw': 0.0,
            'angular_vel_x': 0.0, 'angular_vel_y': 0.0, 'angular_vel_z': 0.0,
            'accel_x': 0.0, 'accel_y': 0.0, 'accel_z': 0.0, 'imu_update_rate': 0.0
        }
        
        if self.robot_state_tracker:
            head_status = self.head_controller.get_status() if self.head_controller else {}
            self.robot_state_tracker.update_state(movement_status, imu_status, head_status)
            robot_state = self.robot_state_tracker.get_robot_state()
        else:
            robot_state = {
                'position_x': 0.0, 'position_y': 0.0, 'orientation': 0.0,
                'velocity_x': 0.0, 'velocity_y': 0.0, 'total_distance_traveled': 0.0,
                'total_rotation': 0.0, 'position_confidence': 1.0, 'orientation_confidence': 1.0
            }
        
        # Default navigation status
        navigation_status = {'navigation_enabled': False, 'current_state': 'IDLE', 'has_person_target': False, 
                            'goal_reached': False, 'person_distance': 0.0, 'person_angle': 0.0, 
                            'time_since_person_update': 0, 'attractive_force_magnitude': 0.0,
                            'repulsive_force_magnitude': 0.0, 'total_force_magnitude': 0.0,
                            'current_movement_direction': 'NONE', 'current_movement_speed': 0.0,
                            'navigation_success_rate': 0.0, 'goals_reached': 0, 'force_calculations_count': 0}
        
        # Variables for diagnostics logging
        nav_command = None
        forces = {'attractive': (0.0, 0.0), 'repulsive': (0.0, 0.0), 'total': (0.0, 0.0)}
        filtered_obstacles = obstacles if obstacles else []
        
        if self.navigator:
            try:
                self.navigator.update_robot_state(robot_state)
                
                # Transform person coordinates and filter obstacles
                if person_data:
                    x_camera = person_data['x_camera']
                    z_camera = person_data['z_camera']
                    
                    servo_position = self.head_controller.current_position if self.head_controller else 0.0
                    lidar_angle, lidar_distance = self.transform_camera_to_lidar_coords(
                        x_camera, z_camera, servo_position
                    )
                    
                    # Store person coordinates for filtering
                    self.person_lidar_angle = lidar_angle
                    self.person_lidar_distance = lidar_distance
                    
                    # Filter out obstacles near the person (reduced radius to prevent excluding real obstacles)
                    if obstacles:
                        filtered_obstacles = self.filter_person_obstacles(
                            obstacles, lidar_angle, lidar_distance, exclusion_radius=200.0
                        )
                    
                    # Update navigator with person target
                    self.navigator.update_person_target(person_data, lidar_distance, lidar_angle)
                else:
                    self.person_lidar_angle = 0.0
                    self.person_lidar_distance = 0.0
                    self.obstacles_filtered_count = 0
                    self.navigator.update_person_target(None, 0, 0)
                
                # Calculate forces using filtered obstacles
                forces = self.navigator.calculate_navigation_forces(filtered_obstacles)
                self.navigator.update_navigation_state(person_data, filtered_obstacles)
                
                navigation_status = self.navigator.get_navigation_status()
                
                if self.navigation_enabled and self.movement_enabled and self.velocity_manager:
                    nav_command = self.navigator.generate_movement_command()
                    if nav_command:
                        direction_str = nav_command['direction']
                        speed = nav_command['speed']
                        
                        direction_map = {
                            'NONE': MovementDirection.NONE,
                            'FORWARDS': MovementDirection.FORWARDS,
                            'BACKWARDS': MovementDirection.BACKWARDS,
                            'LEFT': MovementDirection.LEFT,
                            'RIGHT': MovementDirection.RIGHT,
                            'FORWARDS_LEFT': MovementDirection.FORWARDS_LEFT,
                            'FORWARDS_RIGHT': MovementDirection.FORWARDS_RIGHT,
                            'BACKWARDS_LEFT': MovementDirection.BACKWARDS_LEFT,
                            'BACKWARDS_RIGHT': MovementDirection.BACKWARDS_RIGHT,
                        }
                        
                        direction_enum = direction_map.get(direction_str, MovementDirection.NONE)
                        velocity_config = VelocityConfig(direction_enum, speed)
                        self.velocity_manager.perform_action(velocity_config)
                    
                    # REACQUISITION CONTROL: Handle head tracking and body rotation
                    reacq_data = self.navigator.get_reacquisition_control_data()
                    if reacq_data and self.head_controller:
                        # HEAD TRACKING: Turn head toward last known screen position
                        head_dir = reacq_data['head_direction']
                        if head_dir == 'left':
                            target_servo = -45.0  # Turn head left
                        elif head_dir == 'right':
                            target_servo = 45.0  # Turn head right
                        else:
                            target_servo = 0.0  # Center
                        
                        self.head_controller.move_to_position(target_servo)
                        
                        # BODY ROTATION: Turn toward pre-avoidance orientation
                        bearing_error = reacq_data['bearing_error']
                        if abs(bearing_error) > 5.0:  # Need to rotate body
                            if bearing_error > 0:
                                body_direction = MovementDirection.RIGHT
                            else:
                                body_direction = MovementDirection.LEFT
                            
                            # Slow rotation speed for precision
                            rotation_speed = min(0.3, abs(bearing_error) / 100.0)
                            body_config = VelocityConfig(body_direction, rotation_speed)
                            self.velocity_manager.perform_action(body_config)
            except Exception as e:
                navigation_status['current_state'] = 'ERROR'
        
        # Log navigation diagnostics
        if self.navigation_enabled and self.navigator:
            try:
                # Prepare person data for diagnostics
                diag_person_data = None
                if person_data:
                    diag_person_data = {
                        'detected': True,
                        'lidar_x_mm': self.person_lidar_distance * math.sin(math.radians(self.person_lidar_angle)),
                        'lidar_y_mm': self.person_lidar_distance * math.cos(math.radians(self.person_lidar_angle)),
                        'distance_mm': self.person_lidar_distance,
                        'lidar_angle_deg': self.person_lidar_angle,
                        'confidence': person_data.get('confidence', 0),
                        'age_ms': 0
                    }
                
                # Prepare IMU data for diagnostics
                diag_imu_data = {
                    'imu_yaw': imu_status.get('yaw', 0),
                    'imu_smoothed_yaw': imu_status.get('smoothed_yaw', 0),
                    'imu_roll': imu_status.get('roll', 0),
                    'imu_pitch': imu_status.get('pitch', 0),
                    'imu_angular_vel_z': imu_status.get('angular_vel_z', 0),
                    'imu_update_rate': imu_status.get('imu_update_rate', 0)
                }
                
                # Prepare movement command for diagnostics
                diag_movement_cmd = None
                if nav_command:
                    diag_movement_cmd = {
                        'direction': nav_command.get('direction', 'NONE'),
                        'speed': nav_command.get('speed', 0),
                        'motor_1': 0,
                        'motor_2': 0
                    }
                
                # Log the navigation cycle
                self.nav_diagnostics.log_navigation_cycle(
                    person_data=diag_person_data,
                    robot_state=robot_state,
                    imu_data=diag_imu_data,
                    attractive_force=forces.get('attractive', (0.0, 0.0)),
                    repulsive_force=forces.get('repulsive', (0.0, 0.0)),
                    total_force=forces.get('total', (0.0, 0.0)),
                    obstacles=filtered_obstacles,
                    movement_command=diag_movement_cmd,
                    nav_state=navigation_status.get('current_state', 'UNKNOWN'),
                    goal_reached=navigation_status.get('goal_reached', False),
                    stuck_detected=navigation_status.get('robot_stuck', False),
                    obstacles_filtered=self.obstacles_filtered_count,
                    calculation_time_ms=0.0
                )
            except Exception:
                pass
        
        return movement_status, imu_status, navigation_status
    
    def log_enhanced_detection_to_csv(self, person_data, head_status, movement_status, robot_state, imu_status, navigation_status):
        """Log detection data to CSV"""
        try:
            if not self.csv_initialized:
                return
            
            reused_detection = 0
            last_good_age_ms = 0.0
            raw_position_x = 0
            smoothed_position_x = 0
            
            if person_data:
                bbox_center = person_data['bbox_center']
                x_pixels = bbox_center['x_pixels']
                raw_position_x = x_pixels
                self.detection_count += 1
                
                self.current_variance_pixels, self.current_std_dev_pixels, self.current_mean_pixels = self.calculate_x_midpoint_variance(True, x_pixels)
                
                pixel_error = x_pixels - 910
                person_detected = True
                person_x = x_pixels
                person_z = person_data['z_camera']
                person_conf = person_data.get('confidence', 0.0)
                
                if 'meta' in person_data:
                    reused_detection = person_data['meta'].get('reused_last_good', 0)
                    last_good_age_ms = person_data['meta'].get('last_good_age_ms', 0.0)
            else:
                self.calculate_x_midpoint_variance(False)
                pixel_error = 0
                person_detected = False
                person_x = 0
                person_z = 0
                person_conf = 0.0
            
            smoothed_position_x = head_status.get('smoothed_position', 0) or 0
            
            current_time = time.time()
            mode_elapsed = current_time - self.mode_start_time
            
            with open(self.csv_log_filename, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([
                    current_time, mode_elapsed, self.current_fps,
                    person_detected, person_x, person_z, person_conf,
                    head_status['position'], head_status['is_moving'], head_status['person_centered'],
                    pixel_error, head_status['total_moves'], head_status['successful_centers'], head_status['false_positive_rejections'],
                    self.detection_success_count, self.detection_failure_count, self.detection_rate,
                    head_status['consecutive_successes'], head_status['consecutive_failures'],
                    self.current_variance_pixels, self.current_std_dev_pixels, self.average_variance_pixels,
                    reused_detection, last_good_age_ms,
                    head_status.get('smoothing_applied', 0),
                    smoothed_position_x, raw_position_x,
                    self.current_dynamic_offset_x, self.current_dynamic_offset_y, len(self.calibration_samples),
                    self.movement_enabled, movement_status['is_moving'], movement_status['current_direction'], movement_status['current_speed'],
                    movement_status['movement_success_rate'], movement_status['total_movement_commands'], movement_status['avg_movement_duration'],
                    robot_state['position_x'], robot_state['position_y'], robot_state['orientation'], 
                    robot_state['velocity_x'], robot_state['velocity_y'], robot_state['total_distance_traveled'], 
                    robot_state['total_rotation'], robot_state['position_confidence'], robot_state['orientation_confidence'],
                    imu_status['imu_initialized'], imu_status['roll'], imu_status['pitch'], imu_status['yaw'], imu_status['smoothed_yaw'],
                    imu_status['angular_vel_x'], imu_status['angular_vel_y'], imu_status['angular_vel_z'],
                    imu_status['accel_x'], imu_status['accel_y'], imu_status['accel_z'], imu_status['imu_update_rate'],
                    navigation_status['navigation_enabled'], navigation_status['current_state'], navigation_status['has_person_target'], navigation_status['goal_reached'],
                    navigation_status['person_distance'], navigation_status['person_angle'], navigation_status['time_since_person_update'],
                    navigation_status['attractive_force_magnitude'], navigation_status['repulsive_force_magnitude'], navigation_status['total_force_magnitude'],
                    navigation_status['current_movement_direction'], navigation_status['current_movement_speed'], navigation_status['navigation_success_rate'],
                    navigation_status['goals_reached'], navigation_status['force_calculations_count'], self.obstacles_filtered_count
                ])
        except Exception:
            pass
    
    def main_loop(self):
        """Main application loop"""
        if not self.initialize_all_systems():
            return False
        
        clock = pygame.time.Clock()
        target_fps = 25
        
        try:
            while self.running:
                self.calculate_fps()
                self.update_counter += 1
                
                if self.detection_system:
                    self.detection_system.update_camera_frame()
                
                person_data = self.get_person_detection()
                
                obstacles = []
                if self.lidar_system:
                    obstacles = self.lidar_system.get_display_obstacles()
                
                movement_status, imu_status, navigation_status = self.update_all_systems(person_data, obstacles)
                
                self.handle_enhanced_events(person_data)
                
                if self.head_controller:
                    self.head_controller.update_person_detection(person_data)
                    self.head_controller.execute_head_tracking()
                    head_status = self.head_controller.get_status()
                else:
                    head_status = {
                        'initialized': False, 'position': 0.0, 'is_moving': False,
                        'person_detected': False, 'person_x': None, 'person_centered': False,
                        'total_moves': 0, 'successful_centers': 0, 'false_positive_rejections': 0,
                        'can_move': False, 'time_since_last_move': 0,
                        'consecutive_successes': 0, 'consecutive_failures': 0
                    }
                
                robot_state = self.robot_state_tracker.get_robot_state() if self.robot_state_tracker else {
                    'position_x': 0.0, 'position_y': 0.0, 'orientation': 0.0,
                    'velocity_x': 0.0, 'velocity_y': 0.0, 'total_distance_traveled': 0.0,
                    'total_rotation': 0.0, 'position_confidence': 1.0, 'orientation_confidence': 1.0
                }
                
                self.log_enhanced_detection_to_csv(person_data, head_status, movement_status, robot_state, imu_status, navigation_status)
                
                if self.update_counter % self.display_update_rate == 0:
                    try:
                        self.screen.fill((0, 0, 0))
                        self.draw_radar_grid()
                        self.draw_robot()
                        
                        obstacle_count = 0
                        if obstacles:
                            obstacle_count = self.draw_lidar_data(obstacles)
                        
                        if person_data:
                            self.draw_person_detection(person_data)
                        
                        self.draw_stable_status_display()
                        pygame.display.flip()
                    except Exception:
                        pass
                
                clock.tick(target_fps)
                
        except KeyboardInterrupt:
            pass
        except Exception:
            pass
        finally:
            self.cleanup()
        
        return True
    
    def cleanup(self):
        """Cleanup all systems"""
        # Write navigation diagnostics reports
        try:
            self.nav_diagnostics.write_summary_report()
            self.nav_diagnostics.write_anomaly_report()
            print("\n" + "=" * 60)
            print("NAVIGATION DIAGNOSTICS REPORTS WRITTEN")
            print(f"Check nav_logs/ directory for detailed analysis")
            print("=" * 60 + "\n")
        except Exception:
            pass
        
        try:
            if self.velocity_manager:
                stop_config = VelocityConfig(MovementDirection.NONE, 0.0)
                self.velocity_manager.perform_action(stop_config)
                time.sleep(0.2)
                self.velocity_manager.shutdown()
        except Exception:
            pass
        
        try:
            if self.head_controller:
                self.head_controller.center_servo()
                time.sleep(0.5)
        except Exception:
            pass
        
        try:
            if self.detection_system:
                self.detection_system.shutdown()
        except Exception:
            pass
        
        try:
            if self.lidar_system:
                self.lidar_system.stop()
        except Exception:
            pass
        
        try:
            if pygame.get_init():
                pygame.quit()
        except Exception:
            pass
    
    # Display functions
    def draw_stable_interface(self):
        """Draw stable interface"""
        self.screen.fill((0, 0, 0))
        self.draw_radar_grid()
        self.draw_robot()
    
    def draw_radar_grid(self):
        """Draw radar grid"""
        for distance in [1000, 2000, 3000, 4000, 5000, 6000]:
            radius = distance * self.scale // 1000
            if radius < min(self.center_x, self.center_y) - 50:
                line_width = 3 if distance == 6000 else 2
                color = (0, 150, 0) if distance < 6000 else (255, 255, 0)
                pygame.draw.circle(self.screen, color, (self.center_x, self.center_y), radius, line_width)
        
        for angle in [0, 45, 90, 135, 180, 225, 270, 315]:
            display_angle_rad = math.radians(90 - angle)
            line_length = min(self.center_x, self.center_y) - 80
            end_x = self.center_x + int(line_length * math.cos(display_angle_rad))
            end_y = self.center_y - int(line_length * math.sin(display_angle_rad))
            line_width = 3 if angle % 90 == 0 else 1
            pygame.draw.line(self.screen, (0, 150, 0), (self.center_x, self.center_y), (end_x, end_y), line_width)
    
    def draw_robot(self):
        """Draw robot at center"""
        pygame.draw.circle(self.screen, (0, 255, 0), (self.center_x, self.center_y), 15, 3)
        
        if self.robot_state_tracker:
            robot_state = self.robot_state_tracker.get_robot_state()
            orientation_rad = math.radians(robot_state['orientation'])
            arrow_end_x = self.center_x + int(30 * math.cos(orientation_rad + math.pi/2))
            arrow_end_y = self.center_y - int(30 * math.sin(orientation_rad + math.pi/2))
            pygame.draw.line(self.screen, (0, 255, 0), (self.center_x, self.center_y), (arrow_end_x, arrow_end_y), 5)
        else:
            arrow_end_x = self.center_x
            arrow_end_y = self.center_y - 30
            pygame.draw.line(self.screen, (0, 255, 0), (self.center_x, self.center_y), (arrow_end_x, arrow_end_y), 5)
    
    def draw_lidar_data(self, obstacles):
        """Draw LiDAR data"""
        if not obstacles:
            return 0
        
        obstacle_count = 0
        for angle, distance in obstacles:
            corrected_angle_rad = math.radians(90 - angle)
            x = self.center_x + math.cos(corrected_angle_rad) * (distance * self.scale // 1000)
            y = self.center_y - math.sin(corrected_angle_rad) * (distance * self.scale // 1000)
            if (0 <= x < self.screen.get_width() and 0 <= y < self.screen.get_height()):
                pygame.draw.circle(self.screen, (255, 255, 255), (int(x), int(y)), 2)
                obstacle_count += 1
        return obstacle_count
    
    def transform_camera_to_lidar_coords(self, x_camera, z_camera, servo_position):
        """Transform camera coordinates to LiDAR reference frame"""
        try:
            if z_camera <= 0:
                return 0, 0
                
            max_servo_angle_deg = 45
            clamped_servo_position = max(-0.8, min(0.8, servo_position))
            servo_angle_rad = (clamped_servo_position / 0.8) * math.radians(max_servo_angle_deg)
            
            adjusted_z = z_camera + 130
            
            world_x = x_camera * math.cos(servo_angle_rad) - adjusted_z * math.sin(servo_angle_rad)
            world_z = x_camera * math.sin(servo_angle_rad) + adjusted_z * math.cos(servo_angle_rad)
            
            lidar_distance = math.sqrt(world_x**2 + world_z**2)
            
            if lidar_distance <= 0:
                return 0, 0
                
            lidar_angle = math.atan2(world_x, world_z) * 180 / math.pi
            
            if lidar_angle < 0:
                lidar_angle += 360
                
            return lidar_angle, lidar_distance
        except Exception:
            try:
                person_angle_rad = math.atan2(x_camera, z_camera)
                person_angle_deg = math.degrees(person_angle_rad)
                display_angle_deg = 360 - person_angle_deg
                while display_angle_deg < 0:
                    display_angle_deg += 360
                while display_angle_deg >= 360:
                    display_angle_deg -= 360
                return display_angle_deg, z_camera
            except:
                return 0, 0

    def draw_person_detection(self, person_data):
        """Draw person detection with dynamic offset"""
        if not person_data:
            return None
        
        try:
            x_camera = person_data['x_camera']
            z_camera = person_data['z_camera']
            
            if z_camera <= 0:
                return None
            
            if self.head_controller:
                servo_position = self.head_controller.current_position
            else:
                servo_position = 0.0
            
            lidar_angle, lidar_distance = self.transform_camera_to_lidar_coords(
                x_camera, z_camera, servo_position
            )
            
            distance_m = lidar_distance / 1000.0
            display_angle_rad = math.radians(90 - lidar_angle)
            base_x = self.center_x + int(distance_m * self.scale * math.cos(display_angle_rad))
            base_y = self.center_y - int(distance_m * self.scale * math.sin(display_angle_rad))
            
            self.current_dynamic_offset_x, self.current_dynamic_offset_y = self.calculate_dynamic_offset(
                lidar_distance, lidar_angle, base_x, base_y
            )
            
            x = base_x + int(self.current_dynamic_offset_x)
            y = base_y + int(self.current_dynamic_offset_y)
            
            if 0 <= x < self.screen.get_width() and 0 <= y < self.screen.get_height():
                circle_radius_m = 0.3
                circle_radius_pixels = int(circle_radius_m * self.scale)
                pygame.draw.circle(self.screen, (200, 200, 200), (x, y), circle_radius_pixels, 2)
                
                if self.head_controller:
                    head_status = self.head_controller.get_status()
                    
                    if 'meta' in person_data and person_data['meta'].get('reused_last_good', 0) == 1:
                        color = (0, 255, 255)
                    elif head_status['person_centered']:
                        color = (0, 255, 0)
                    elif head_status['is_moving']:
                        color = (255, 0, 255)
                    else:
                        color = (255, 255, 0)
                else:
                    color = (255, 255, 255)
                
                pygame.draw.circle(self.screen, color, (x, y), 8)
                pygame.draw.circle(self.screen, (255, 255, 255), (x, y), 9, 2)
                
                if self.calibration_mode:
                    pygame.draw.line(self.screen, (0, 255, 0), (base_x, base_y), (x, y), 2)
            
            return person_data
        except Exception:
            return None

    def draw_stable_status_display(self):
        """Draw status display"""
        try:
            huge_font = pygame.font.Font(None, 120)
            large_font = pygame.font.Font(None, 72)
            medium_font = pygame.font.Font(None, 48)
            
            display_x = self.stable_person_x if self.stable_person_x > 0 else self.last_valid_person_x
            
            person_x_text = huge_font.render(f"PERSON X-CENTER: {display_x}px", True, (0, 255, 255))
            person_x_rect = person_x_text.get_rect(center=(self.screen.get_width() // 2, 80))
            self.screen.blit(person_x_text, person_x_rect)
            
            rate_text = large_font.render(f"Detection Rate: {self.detection_rate:.1f}%", True, (255, 255, 255))
            rate_rect = rate_text.get_rect(center=(self.screen.get_width() // 2, 200))
            self.screen.blit(rate_text, rate_rect)
            
            # Movement status
            if self.velocity_manager:
                movement_status = self.velocity_manager.get_movement_status()
                movement_color = (0, 255, 0) if movement_status['is_moving'] else (255, 128, 0)
                movement_text = medium_font.render(f"Movement: {'ENABLED' if self.movement_enabled else 'DISABLED'} | {movement_status['current_direction']}", True, movement_color)
                movement_rect = movement_text.get_rect(center=(self.screen.get_width() // 2, 270))
                self.screen.blit(movement_text, movement_rect)
            
            # Navigation status
            nav_state_text = "UNKNOWN"
            nav_color = (255, 0, 0)
            
            if self.navigator is None:
                nav_state_text = "NO_NAVIGATOR"
                nav_color = (128, 128, 128)
            else:
                try:
                    nav_status = self.navigator.get_navigation_status()
                    nav_state_text = nav_status.get('current_state', 'MISSING_STATE')
                    nav_color = (0, 255, 0) if self.navigation_enabled else (255, 128, 0)
                except Exception:
                    nav_state_text = "ERROR"
                    nav_color = (255, 0, 0)
            
            nav_text = medium_font.render(f"Navigation: {'ENABLED' if self.navigation_enabled else 'DISABLED'} | {nav_state_text}", True, nav_color)
            nav_rect = nav_text.get_rect(center=(self.screen.get_width() // 2, 320))
            self.screen.blit(nav_text, nav_rect)
            
            # Test mode display
            if self.nav_diagnostics:
                test_mode_name = self.nav_diagnostics.get_test_mode_name()
                test_color = (0, 255, 255) if test_mode_name != "GENERAL" else (128, 128, 128)
                test_text = medium_font.render(f"Test Mode: {test_mode_name} (Press 1 or 2)", True, test_color)
                test_rect = test_text.get_rect(center=(self.screen.get_width() // 2, 370))
                self.screen.blit(test_text, test_rect)
            
        except Exception:
            pass

    def handle_enhanced_events(self, person_data=None):
        """Handle pygame events"""
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    self.running = False
                
                # Test mode selection
                elif event.key == pygame.K_1:
                    if self.nav_diagnostics:
                        self.nav_diagnostics.set_test_mode(1)
                
                elif event.key == pygame.K_2:
                    if self.nav_diagnostics:
                        self.nav_diagnostics.set_test_mode(2)
                
                elif event.key == pygame.K_m:
                    self.movement_enabled = not self.movement_enabled
                    if not self.movement_enabled and self.velocity_manager:
                        stop_config = VelocityConfig(MovementDirection.NONE, 0.0)
                        self.velocity_manager.perform_action(stop_config)
                
                elif event.key == pygame.K_n:
                    if self.navigator:
                        try:
                            self.navigation_enabled = not self.navigation_enabled
                            if self.navigation_enabled:
                                if self.movement_enabled:
                                    self.navigator.enable_navigation()
                                else:
                                    self.navigation_enabled = False
                            else:
                                self.navigator.disable_navigation()
                        except Exception as e:
                            self.navigation_enabled = False
                
                elif event.key == pygame.K_f:
                    self.show_navigation_forces = not self.show_navigation_forces
                
                # Manual movement on key press (only when navigation is disabled)
                elif self.movement_enabled and not self.navigation_enabled and self.velocity_manager:
                    # MANUAL CONTROL: Increased speed for faster navigation
                    movement_speed = 0.85  # Increased from 0.5 for faster manual control
                    
                    if event.key == pygame.K_UP:
                        config = VelocityConfig(MovementDirection.FORWARDS, movement_speed)
                        self.velocity_manager.perform_action(config)
                    elif event.key == pygame.K_DOWN:
                        config = VelocityConfig(MovementDirection.BACKWARDS, movement_speed)
                        self.velocity_manager.perform_action(config)
                    elif event.key == pygame.K_LEFT:
                        config = VelocityConfig(MovementDirection.LEFT, movement_speed)
                        self.velocity_manager.perform_action(config)
                    elif event.key == pygame.K_RIGHT:
                        config = VelocityConfig(MovementDirection.RIGHT, movement_speed)
                        self.velocity_manager.perform_action(config)
                    elif event.key == pygame.K_SPACE:
                        config = VelocityConfig(MovementDirection.NONE, 0.0)
                        self.velocity_manager.perform_action(config)
            
            # Stop movement when arrow keys are released
            elif event.type == pygame.KEYUP:
                if event.key in [pygame.K_UP, pygame.K_DOWN, pygame.K_LEFT, pygame.K_RIGHT]:
                    if self.movement_enabled and not self.navigation_enabled and self.velocity_manager:
                        stop_config = VelocityConfig(MovementDirection.NONE, 0.0)
                        self.velocity_manager.perform_action(stop_config)
                        
            elif event.type == pygame.QUIT:
                self.running = False
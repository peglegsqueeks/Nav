#!/usr/bin/env python3
"""
Robot State Tracking System
Comprehensive robot state tracking for position, orientation, and movement
Integrates head position, base movement, and IMU data
"""
import math
import time
import statistics
from collections import deque
from .movement_types import MovementDirection

class RobotStateTracker:
    """
    Comprehensive robot state tracking for position, orientation, and movement
    Integrates head position, base movement, and IMU data
    """
    
    def __init__(self):
        # Robot position (relative to starting point)
        self.position_x = 0.0  # mm from start
        self.position_y = 0.0  # mm from start
        self.orientation = 0.0  # degrees from start orientation
        
        # Movement integration
        self.last_update_time = time.time()
        self.velocity_x = 0.0  # mm/s
        self.velocity_y = 0.0  # mm/s
        self.angular_velocity = 0.0  # degrees/s
        
        # Position history for smoothing
        self.position_history = deque(maxlen=20)
        self.orientation_history = deque(maxlen=20)
        
        # Performance tracking
        self.total_distance_traveled = 0.0  # mm
        self.total_rotation = 0.0  # degrees
        self.max_velocity_recorded = 0.0  # mm/s
        
        # State estimation confidence
        self.position_confidence = 1.0  # 0-1 scale
        self.orientation_confidence = 1.0  # 0-1 scale

    def update_state(self, movement_status, imu_status, head_status):
        """
        Update robot state based on movement commands and IMU data
        Uses dead reckoning with IMU correction
        """
        current_time = time.time()
        dt = current_time - self.last_update_time
        self.last_update_time = current_time
        
        if dt <= 0 or dt > 1.0:  # Skip invalid time deltas
            return
            
        try:
            # Update orientation from IMU if available
            if imu_status['imu_initialized']:
                # Use smoothed yaw from IMU as primary orientation source
                self.orientation = imu_status['smoothed_yaw']
                self.orientation_confidence = 0.9  # High confidence with IMU
                self.angular_velocity = imu_status['angular_vel_z']
            else:
                # Fallback: estimate rotation from movement commands
                if movement_status['is_moving']:
                    direction = movement_status['current_direction']
                    speed = movement_status['current_speed']
                    
                    # Rough angular velocity estimation for turning movements
                    if 'LEFT' in direction:
                        estimated_angular_vel = -30.0 * speed  # degrees/s
                    elif 'RIGHT' in direction:
                        estimated_angular_vel = 30.0 * speed  # degrees/s
                    else:
                        estimated_angular_vel = 0.0
                        
                    self.angular_velocity = estimated_angular_vel
                    self.orientation += self.angular_velocity * dt
                    self.orientation = self.normalize_angle(self.orientation)
                    self.orientation_confidence = 0.5  # Lower confidence without IMU
            
            # Update position from movement integration
            if movement_status['is_moving']:
                direction = movement_status['current_direction']
                speed = movement_status['current_speed']
                
                # Estimate linear velocity based on direction and speed
                forward_vel, turn_rate = self.direction_to_velocity(direction, speed)
                
                # Convert to world coordinates using current orientation
                orientation_rad = math.radians(self.orientation)
                self.velocity_x = forward_vel * math.cos(orientation_rad)
                self.velocity_y = forward_vel * math.sin(orientation_rad)
                
                # Update position
                self.position_x += self.velocity_x * dt
                self.position_y += self.velocity_y * dt
                
                # Track total distance
                distance_delta = math.sqrt((self.velocity_x * dt)**2 + (self.velocity_y * dt)**2)
                self.total_distance_traveled += distance_delta
                
                # Track maximum velocity
                current_vel_magnitude = math.sqrt(self.velocity_x**2 + self.velocity_y**2)
                self.max_velocity_recorded = max(self.max_velocity_recorded, current_vel_magnitude)
                
            else:
                self.velocity_x = 0.0
                self.velocity_y = 0.0
            
            # Add to history for smoothing
            self.position_history.append((self.position_x, self.position_y))
            self.orientation_history.append(self.orientation)
            
            # Track total rotation
            if len(self.orientation_history) > 1:
                angle_delta = abs(self.orientation_history[-1] - self.orientation_history[-2])
                if angle_delta > 180:  # Handle wraparound
                    angle_delta = 360 - angle_delta
                self.total_rotation += angle_delta
                
        except Exception:
            # If state update fails, maintain previous values
            pass

    def direction_to_velocity(self, direction, speed):
        """
        Convert MovementDirection and speed to forward velocity and turn rate
        Returns (forward_velocity_mm_s, turn_rate_deg_s)
        """
        # Base velocities (mm/s) - these are rough estimates and may need tuning
        base_forward_velocity = 300.0  # mm/s at speed=1.0
        base_turn_rate = 45.0  # degrees/s at speed=1.0
        
        if isinstance(direction, str):
            direction = MovementDirection(direction)
        
        direction_map = {
            MovementDirection.NONE: (0.0, 0.0),
            MovementDirection.FORWARDS: (base_forward_velocity, 0.0),
            MovementDirection.BACKWARDS: (-base_forward_velocity, 0.0),
            MovementDirection.LEFT: (0.0, base_turn_rate),
            MovementDirection.RIGHT: (0.0, -base_turn_rate),
            MovementDirection.FORWARDS_LEFT: (base_forward_velocity * 0.7, base_turn_rate * 0.5),
            MovementDirection.FORWARDS_RIGHT: (base_forward_velocity * 0.7, -base_turn_rate * 0.5),
            MovementDirection.BACKWARDS_LEFT: (-base_forward_velocity * 0.7, base_turn_rate * 0.5),
            MovementDirection.BACKWARDS_RIGHT: (-base_forward_velocity * 0.7, -base_turn_rate * 0.5),
        }
        
        base_forward, base_turn = direction_map.get(direction, (0.0, 0.0))
        return base_forward * speed, base_turn * speed

    def normalize_angle(self, angle):
        """Normalize angle to -180 to 180 range"""
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle

    def get_robot_state(self):
        """Get complete robot state for logging and navigation"""
        return {
            'position_x': self.position_x,
            'position_y': self.position_y,
            'orientation': self.orientation,
            'velocity_x': self.velocity_x,
            'velocity_y': self.velocity_y,
            'angular_velocity': self.angular_velocity,
            'total_distance_traveled': self.total_distance_traveled,
            'total_rotation': self.total_rotation,
            'max_velocity_recorded': self.max_velocity_recorded,
            'position_confidence': self.position_confidence,
            'orientation_confidence': self.orientation_confidence
        }
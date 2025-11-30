#!/usr/bin/env python3
"""
Potential Field Mathematics Engine - FIXED VERSION
Core mathematical calculations for attractive and repulsive force fields
Handles force vector summation and force-to-velocity conversion
"""
import math
from typing import List, Tuple, Dict, Any

class PotentialFieldCalculations:
    """
    Core mathematics engine for potential field navigation
    Implements attractive fields (towards person) and repulsive fields (from obstacles)
    """
    
    def __init__(self):
        # ATTRACTIVE FIELD PARAMETERS (towards person)
        self.attractive_strength = 2.0  # Base attractive force strength
        self.attractive_max_distance = 5000.0  # mm - beyond this, no attraction
        self.attractive_min_distance = 500.0  # mm - minimum safe distance to person
        
        # REPULSIVE FIELD PARAMETERS (from obstacles)
        self.repulsive_strength = 3.0  # Base repulsive force strength  
        self.repulsive_influence_radius = 800.0  # mm - obstacles influence within this radius
        self.repulsive_max_force_distance = 300.0  # mm - maximum repulsive force distance
        
        # FORCE LIMITS AND SCALING
        self.max_total_force = 5.0  # Maximum resultant force magnitude
        self.force_to_velocity_scale = 0.3  # Conversion factor from force to velocity
        self.min_movement_force = 0.2  # Minimum force required to generate movement
        
        # GOAL PARAMETERS
        self.goal_reached_threshold = 600.0  # mm - consider goal reached within this distance
        
        # PERFORMANCE TRACKING
        self.calculation_count = 0
        self.attractive_force_history = []
        self.repulsive_force_history = []
        self.total_force_history = []
    
    def calculate_attractive_field(self, robot_pos_x: float, robot_pos_y: float, 
                                 person_pos_x: float, person_pos_y: float) -> Tuple[float, float]:
        """Calculate attractive force field towards detected person"""
        try:
            # Calculate distance to person
            dx = person_pos_x - robot_pos_x
            dy = person_pos_y - robot_pos_y
            distance = math.sqrt(dx**2 + dy**2)
            
            # Check if person is within attraction range
            if distance > self.attractive_max_distance or distance < 1.0:
                return 0.0, 0.0
            
            # Calculate unit direction vector towards person
            unit_x = dx / distance
            unit_y = dy / distance
            
            # Apply different force profiles based on distance
            if distance <= self.attractive_min_distance:
                # Within minimum safe distance - no or reduced attraction
                force_magnitude = 0.1 * self.attractive_strength
            elif distance <= 1500.0:  # Close range - linear increase
                # Linear increase from min distance to mid range
                normalized_distance = (distance - self.attractive_min_distance) / 1000.0
                force_magnitude = self.attractive_strength * normalized_distance
            else:  # Long range - inverse square law
                # Inverse square law for long distance attraction
                force_magnitude = self.attractive_strength * (1500.0 / distance)**2
            
            # Apply directional force
            force_x = force_magnitude * unit_x
            force_y = force_magnitude * unit_y
            
            # Store for analysis
            self.attractive_force_history.append((force_x, force_y, distance))
            
            return force_x, force_y
            
        except Exception:
            return 0.0, 0.0
    
    def calculate_repulsive_field(self, robot_pos_x: float, robot_pos_y: float, 
                                obstacles: List[Tuple[float, float]]) -> Tuple[float, float]:
        """Calculate repulsive force field from LiDAR obstacles"""
        try:
            total_repulsive_x = 0.0
            total_repulsive_y = 0.0
            active_obstacles = 0
            
            for angle_deg, distance_mm in obstacles:
                # Convert polar to cartesian coordinates (LiDAR frame)
                angle_rad = math.radians(angle_deg)
                obstacle_x = robot_pos_x + distance_mm * math.cos(angle_rad)
                obstacle_y = robot_pos_y + distance_mm * math.sin(angle_rad)
                
                # Calculate distance from robot to obstacle
                dx = obstacle_x - robot_pos_x
                dy = obstacle_y - robot_pos_y
                distance_to_obstacle = math.sqrt(dx**2 + dy**2)
                
                # Only consider obstacles within influence radius
                if distance_to_obstacle > self.repulsive_influence_radius or distance_to_obstacle < 1.0:
                    continue
                
                active_obstacles += 1
                
                # Calculate unit vector pointing away from obstacle (towards robot)
                unit_x = -dx / distance_to_obstacle  # Negative because we want to move away
                unit_y = -dy / distance_to_obstacle
                
                # Calculate repulsive force magnitude (inverse relationship with distance)
                if distance_to_obstacle <= self.repulsive_max_force_distance:
                    # Very close - maximum repulsive force
                    force_magnitude = self.repulsive_strength
                else:
                    # Inverse square law for repulsive force
                    force_magnitude = self.repulsive_strength * (self.repulsive_max_force_distance / distance_to_obstacle)**2
                
                # Weight based on obstacle angle (obstacles directly ahead get more weight)
                angle_weight = 1.0 + 0.5 * math.cos(angle_rad)  # More weight for front obstacles
                
                # Apply weighted repulsive force
                repulsive_x = force_magnitude * unit_x * angle_weight
                repulsive_y = force_magnitude * unit_y * angle_weight
                
                total_repulsive_x += repulsive_x
                total_repulsive_y += repulsive_y
            
            # Store for analysis
            self.repulsive_force_history.append((total_repulsive_x, total_repulsive_y, active_obstacles))
            
            return total_repulsive_x, total_repulsive_y
            
        except Exception:
            return 0.0, 0.0
    
    def sum_force_vectors(self, attractive_force: Tuple[float, float], 
                         repulsive_force: Tuple[float, float]) -> Tuple[float, float]:
        """Sum attractive and repulsive force vectors with magnitude limiting"""
        try:
            # Sum the force vectors
            total_x = attractive_force[0] + repulsive_force[0]
            total_y = attractive_force[1] + repulsive_force[1]
            
            # Calculate magnitude
            magnitude = math.sqrt(total_x**2 + total_y**2)
            
            # Limit maximum force magnitude
            if magnitude > self.max_total_force:
                scale_factor = self.max_total_force / magnitude
                total_x *= scale_factor
                total_y *= scale_factor
                magnitude = self.max_total_force
            
            # Store for analysis
            self.total_force_history.append((total_x, total_y, magnitude))
            self.calculation_count += 1
            
            return total_x, total_y
            
        except Exception:
            return 0.0, 0.0
    
    def convert_force_to_velocity(self, force_x: float, force_y: float, 
                                 current_orientation: float) -> Tuple[str, float]:
        """Convert force vector to robot movement direction and speed - FIXED IMPORTS"""
        try:
            # Calculate force magnitude and direction
            force_magnitude = math.sqrt(force_x**2 + force_y**2)
            
            # Check if force is significant enough to move
            if force_magnitude < self.min_movement_force:
                return "NONE", 0.0
            
            # Calculate desired movement direction (global frame)
            desired_angle_rad = math.atan2(force_y, force_x)
            desired_angle_deg = math.degrees(desired_angle_rad)
            
            # Convert to robot-relative angle
            relative_angle = desired_angle_deg - current_orientation
            
            # Normalize angle to -180 to 180
            while relative_angle > 180:
                relative_angle -= 360
            while relative_angle < -180:
                relative_angle += 360
            
            # Calculate movement speed (scaled from force magnitude)
            speed = min(1.0, force_magnitude * self.force_to_velocity_scale)
            
            # Determine movement direction based on relative angle
            abs_angle = abs(relative_angle)
            
            if abs_angle <= 30:  # Within 30 degrees of forward
                return "FORWARDS", speed
            elif abs_angle >= 150:  # Within 30 degrees of backward  
                return "BACKWARDS", speed
            elif -120 <= relative_angle <= -30:  # Left side
                if abs_angle <= 60:
                    return "FORWARDS_LEFT", speed * 0.8
                else:
                    return "LEFT", speed * 0.6
            elif 30 <= relative_angle <= 120:  # Right side
                if abs_angle <= 60:
                    return "FORWARDS_RIGHT", speed * 0.8
                else:
                    return "RIGHT", speed * 0.6
            else:
                # Default to turning towards target
                if relative_angle > 0:
                    return "RIGHT", speed * 0.5
                else:
                    return "LEFT", speed * 0.5
            
        except Exception:
            return "NONE", 0.0
    
    def is_goal_reached(self, robot_pos_x: float, robot_pos_y: float, 
                       person_pos_x: float, person_pos_y: float) -> bool:
        """Check if robot has reached the goal (person) position"""
        try:
            dx = person_pos_x - robot_pos_x
            dy = person_pos_y - robot_pos_y
            distance = math.sqrt(dx**2 + dy**2)
            
            return distance <= self.goal_reached_threshold
            
        except Exception:
            return False
    
    def check_local_minimum(self, recent_positions: List[Tuple[float, float]], 
                           time_window: float = 5.0) -> bool:
        """Detect if robot is stuck in local minimum (oscillating or not moving)"""
        try:
            if len(recent_positions) < 10:  # Need sufficient history
                return False
            
            # Calculate position variance over recent history
            recent_x = [pos[0] for pos in recent_positions[-10:]]
            recent_y = [pos[1] for pos in recent_positions[-10:]]
            
            if len(set(recent_x)) == 1 and len(set(recent_y)) == 1:
                return True  # No movement at all
            
            # Calculate movement variance
            import statistics
            if len(recent_x) > 1:
                x_variance = statistics.variance(recent_x)
                y_variance = statistics.variance(recent_y)
                total_variance = x_variance + y_variance
                
                # If variance is very low, robot might be oscillating/stuck
                return total_variance < 1000.0  # mm^2 threshold
            
            return False
            
        except Exception:
            return False
    
    def get_field_parameters(self) -> Dict[str, Any]:
        """Get current field parameters for tuning and display"""
        return {
            'attractive_strength': self.attractive_strength,
            'attractive_max_distance': self.attractive_max_distance,
            'attractive_min_distance': self.attractive_min_distance,
            'repulsive_strength': self.repulsive_strength,
            'repulsive_influence_radius': self.repulsive_influence_radius,
            'repulsive_max_force_distance': self.repulsive_max_force_distance,
            'max_total_force': self.max_total_force,
            'force_to_velocity_scale': self.force_to_velocity_scale,
            'goal_reached_threshold': self.goal_reached_threshold,
            'calculation_count': self.calculation_count
        }
    
    def update_field_parameters(self, **kwargs):
        """Update field parameters for real-time tuning"""
        for key, value in kwargs.items():
            if hasattr(self, key):
                setattr(self, key, value)
    
    def get_force_statistics(self) -> Dict[str, Any]:
        """Get force calculation statistics for analysis"""
        try:
            recent_attractive = self.attractive_force_history[-100:] if self.attractive_force_history else []
            recent_repulsive = self.repulsive_force_history[-100:] if self.repulsive_force_history else []
            recent_total = self.total_force_history[-100:] if self.total_force_history else []
            
            stats = {
                'calculation_count': self.calculation_count,
                'avg_attractive_magnitude': 0.0,
                'avg_repulsive_magnitude': 0.0,
                'avg_total_magnitude': 0.0,
                'max_total_magnitude': 0.0
            }
            
            if recent_attractive:
                attractive_mags = [math.sqrt(fx**2 + fy**2) for fx, fy, _ in recent_attractive]
                stats['avg_attractive_magnitude'] = sum(attractive_mags) / len(attractive_mags)
            
            if recent_repulsive:
                repulsive_mags = [math.sqrt(fx**2 + fy**2) for fx, fy, _ in recent_repulsive]
                stats['avg_repulsive_magnitude'] = sum(repulsive_mags) / len(repulsive_mags)
            
            if recent_total:
                total_mags = [mag for _, _, mag in recent_total]
                stats['avg_total_magnitude'] = sum(total_mags) / len(total_mags)
                stats['max_total_magnitude'] = max(total_mags)
            
            return stats
            
        except Exception:
            return {
                'calculation_count': self.calculation_count,
                'avg_attractive_magnitude': 0.0,
                'avg_repulsive_magnitude': 0.0,
                'avg_total_magnitude': 0.0,
                'max_total_magnitude': 0.0
            }
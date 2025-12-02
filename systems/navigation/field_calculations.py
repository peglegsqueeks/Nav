#!/usr/bin/env python3
"""
Potential Field Mathematics Engine - VERSION 4.3
Core mathematical calculations for attractive and repulsive force fields
Handles force vector summation and force-to-velocity conversion

VERSION HISTORY:
v4.3 (2025-12-02 08:45): FORCE BALANCE - 3RD ITERATION
- CRITICAL: repulsive_strength: 1.2 → 0.8 (33% reduction - much gentler!)
- CRITICAL: max_repulsive_magnitude: 2.5 → 2.0 (20% reduction)
- Problem: Repulsive STILL maxing at 2.5 entire test, overwhelming attractive 3.0
- Target ratio: Attractive 6.0 vs Repulsive cap 2.0 (3:1 attractive dominant!)
- Expected forces: 800mm=0.8, 400mm=1.6, 200mm=2.0 (capped but manageable)

v4.2 (2025-12-02 08:30): CRITICAL FORCE BALANCE FIX
- CRITICAL: max_repulsive_magnitude: 8.0 → 3.5 (was causing circular motion!)
- Reduced repulsive_strength: 2.0 → 1.5 (prevent overwhelming attraction)
- Problem: Repulsive was maxing at 8.0, overwhelming attractive 3.6, causing LEFT turns

v4.1 (2025-12-01 15:50): SLIGHTLY GENTLER OBSTACLE AVOIDANCE TURNS
- Reduced angle_weight: 0.3 → 0.25 (slight 17% reduction, not 50%!)
- Result: Robot turns slightly less sharply when avoiding obstacles ahead

v4 (2025-12-01 15:30): EXPONENTIAL OBSTACLE AVOIDANCE
- Attractive strength: 6.0 (strong long-range pull)
- Repulsive strength: 2.0 (base strength)
- Repulsive influence radius: 1000mm (detect obstacles at 1m)
- Max repulsive magnitude: 8.0 (allow very strong avoidance)
- Inverse CUBE law for obstacles < 300mm (exponential close-range push!)
- Inverse SQUARE law for obstacles 300-600mm (strong push)
- Linear falloff for obstacles 600-1000mm (gentle push)
- Changed long-range attractive: inverse square → linear (maintain strength at distance)

v3 (2025-12-01 14:00): OBSTACLE DETECTION RANGE
- Increased repulsive_influence_radius: 500mm → 1000mm
- Increased attractive_strength: 4.0 → 5.0
- Balanced for Test 2 obstacle bypass

v2 (2025-12-01 12:00): FORCE BALANCE TUNING
- Increased attractive_strength: 2.0 → 3.5
- Decreased repulsive_strength: 3.0 → 1.5
- Decreased repulsive_influence_radius: 800mm → 600mm
- Added repulsive force cap to prevent overwhelming attraction

v1 (2025-12-01 10:00): COORDINATE SYSTEM FIX
- Fixed atan2 argument order for robot coordinate system
- Robot frame: 0° = Forward (+Y), 90° = Right (+X)
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
        self.attractive_strength = 6.0  # Strong long-range pull
        self.attractive_max_distance = 5000.0  # mm - beyond this, no attraction
        self.attractive_min_distance = 300.0  # mm - minimum safe distance to person
        
        # REPULSIVE FIELD PARAMETERS (from obstacles)
        self.repulsive_strength = 0.8  # REDUCED from 1.2 - much gentler base strength!
        self.repulsive_influence_radius = 1000.0  # mm - obstacle detection radius
        self.repulsive_max_force_distance = 600.0  # mm - stronger push zone
        self.max_repulsive_magnitude = 2.0  # CRITICAL: Reduced from 2.5 - prevent maxing out!
        
        # FORCE LIMITS AND SCALING
        self.max_total_force = 8.0  # Maximum resultant force magnitude
        self.force_to_velocity_scale = 0.3  # Conversion factor from force to velocity
        self.min_movement_force = 0.15  # Minimum force required to generate movement
        
        # GOAL PARAMETERS
        self.goal_reached_threshold = 400.0  # mm - consider goal reached within this distance
        
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
                # Within minimum safe distance - reduced attraction
                force_magnitude = 0.1 * self.attractive_strength
            elif distance <= 2000.0:  # Close to mid range - linear increase
                # Linear increase from min distance to mid range
                normalized_distance = (distance - self.attractive_min_distance) / 1700.0
                force_magnitude = self.attractive_strength * normalized_distance
            else:  # Long range - constant strong attraction (not inverse square)
                # CHANGED: Maintain strong force at long range for obstacle bypass
                # Use linear falloff instead of inverse square
                force_magnitude = self.attractive_strength * (2000.0 / distance)
                # Cap at full strength
                force_magnitude = min(force_magnitude, self.attractive_strength)
            
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
                
                # Calculate repulsive force magnitude
                # CRITICAL: Use inverse CUBE for close range to force sharp turns!
                if distance_to_obstacle <= 300.0:
                    # VERY close (< 300mm) - inverse cube law for exponential growth
                    # This creates VERY strong repulsion to force sharp turns
                    force_magnitude = self.repulsive_strength * (300.0 / distance_to_obstacle)**3
                elif distance_to_obstacle <= self.repulsive_max_force_distance:
                    # Close range (300-600mm) - inverse square law
                    force_magnitude = self.repulsive_strength * (self.repulsive_max_force_distance / distance_to_obstacle)**2
                else:
                    # Far range (600-1000mm) - linear falloff
                    distance_ratio = (self.repulsive_influence_radius - distance_to_obstacle) / (self.repulsive_influence_radius - self.repulsive_max_force_distance)
                    force_magnitude = self.repulsive_strength * distance_ratio
                
                # Weight based on obstacle angle (obstacles directly ahead get more weight)
                # TUNED: Slightly reduced angle weight to make turns less aggressive
                angle_weight = 1.0 + 0.25 * math.cos(angle_rad)  # Reduced from 0.3 to 0.25 (slight 17% reduction)
                
                # Apply weighted repulsive force
                repulsive_x = force_magnitude * unit_x * angle_weight
                repulsive_y = force_magnitude * unit_y * angle_weight
                
                total_repulsive_x += repulsive_x
                total_repulsive_y += repulsive_y
            
            # NEW: Cap total repulsive force to prevent overwhelming attraction
            repulsive_magnitude = math.sqrt(total_repulsive_x**2 + total_repulsive_y**2)
            if repulsive_magnitude > self.max_repulsive_magnitude:
                scale = self.max_repulsive_magnitude / repulsive_magnitude
                total_repulsive_x *= scale
                total_repulsive_y *= scale
            
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
            
            # Track calculation
            self.calculation_count += 1
            self.total_force_history.append((total_x, total_y, magnitude))
            
            return total_x, total_y
            
        except Exception:
            return 0.0, 0.0
    
    def convert_force_to_velocity(self, force_x: float, force_y: float, 
                         robot_orientation: float = 0.0) -> Tuple[str, float]:
        """Convert force vector to velocity command (direction and speed)
        
        COORDINATE SYSTEM:
        - Robot frame: 0° = Forward (+Y), 90° = Right (+X)
        - force_x = lateral force (positive = right)
        - force_y = forward force (positive = forward)
        - Use atan2(x, y) to get angle from forward (+Y) axis
        """
        try:
            magnitude = math.sqrt(force_x**2 + force_y**2)
            
            # Check if force is sufficient for movement
            if magnitude < self.min_movement_force:
                return "NONE", 0.0
            
            # Calculate force direction in robot frame
            # FIXED: atan2(x, y) gives angle from +Y axis (forward)
            # Positive angles = clockwise = right
            # Negative angles = counter-clockwise = left
            force_angle = math.atan2(force_x, force_y)
            force_angle_deg = math.degrees(force_angle)
            
            # Convert to robot-relative angle (already in robot frame, so just apply orientation offset)
            relative_angle = force_angle_deg - robot_orientation
            
            # Normalize to -180 to 180
            while relative_angle > 180:
                relative_angle -= 360
            while relative_angle < -180:
                relative_angle += 360
            
            # Calculate speed based on force magnitude
            speed = min(1.0, magnitude * self.force_to_velocity_scale)
            
            # Determine movement direction based on relative angle
            # 0° = forward, positive = right, negative = left
            if -22.5 <= relative_angle < 22.5:
                return "FORWARDS", speed
            elif 22.5 <= relative_angle < 67.5:
                return "FORWARDS_RIGHT", speed
            elif 67.5 <= relative_angle < 112.5:
                return "RIGHT", speed * 0.7
            elif 112.5 <= relative_angle < 157.5:
                return "BACKWARDS_RIGHT", speed * 0.5
            elif relative_angle >= 157.5 or relative_angle < -157.5:
                return "BACKWARDS", speed * 0.5
            elif -157.5 <= relative_angle < -112.5:
                return "BACKWARDS_LEFT", speed * 0.5
            elif -112.5 <= relative_angle < -67.5:
                return "LEFT", speed * 0.7
            elif -67.5 <= relative_angle < -22.5:
                return "FORWARDS_LEFT", speed
            else:
                # Default: turn towards target
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
            'max_repulsive_magnitude': self.max_repulsive_magnitude,
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
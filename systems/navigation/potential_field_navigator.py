#!/usr/bin/env python3
"""
Potential Field Navigation System with Target Reacquisition - VERSION 3.1
Main navigation controller that integrates person detection, obstacle avoidance,
and robot movement using potential field mathematics

VERSION HISTORY:
v3.1 (2025-12-02 09:00): REVERTED SCREEN_X CHANGES
- REVERTED: Removed screen_x parameter from update_person_target()
- REVERTED: Removed last_person_screen_x tracking
- Head direction in reacquisition now defaults to 'center'
- Person detection system restored to previous working state

v3 (2025-12-01 16:00): STOP AND REACQUIRE METHODOLOGY
- CRITICAL: person_lost_threshold: 0.5s → 0.25s (IMMEDIATE response)
- METHODOLOGY CHANGE: Robot STOPS completely when person lost > 0.25s
- Head tracking: Turns toward last screen position (left/right)
- Body rotation: Turns toward pre-avoidance orientation  
- NO forward motion until target reacquired
- Added get_reacquisition_control_data() for head/body control
- Detects obstacle avoidance start (stores pre-avoidance orientation)
- Tracks last person screen X position for head direction

v2 (2025-12-01 15:30): TARGET REACQUISITION - RAPID RESPONSE
- Reduced person_lost_threshold: 2.0s → 0.5s (4x faster detection)
- Reduced person_timeout: 3.0s → 1.0s (lose target faster)
- Added EMERGENCY STOP when person lost (prevents crash)
- Reacquisition now OVERRIDES all other forces (pure rotation)
- Added reacquisition_in_progress flag to block normal navigation
- Reduced reacquisition turn speeds for precision

v1 (2025-12-01 14:00): TARGET REACQUISITION - INITIAL
- Added target position memory system (IMU-based bearing storage)
- Added reacquisition mode state machine
- New methods: calculate_reacquisition_bearing(), generate_reacquisition_movement()
- Modified update_person_target() to store memory
- Modified generate_movement_command() to handle reacquisition

PREVIOUS FIXES (2025-12-01 11:00):
- Reset last_significant_movement when navigation is enabled
- Increased stuck_threshold_time from 8s to 10s
- Fixed deque slice indexing in get_navigation_status()
- Fixed coordinate system (atan2 argument order for robot frame)
"""
import time
import math
from collections import deque
from typing import Optional, Dict, Any, List, Tuple

from .field_calculations import PotentialFieldCalculations
from .navigation_state_machine import NavigationStateMachine


class PotentialFieldNavigator:
    """
    Main potential field navigation controller - FIXED VERSION
    Coordinates person detection, obstacle avoidance, and robot movement
    """
    
    def __init__(self):
        # Core systems
        self.field_calculator = PotentialFieldCalculations()
        self.state_machine = NavigationStateMachine()
        
        # Navigation state
        self.navigation_enabled = False
        self.person_target = None  # Current person target position
        self.last_person_update = 0.0
        self.person_timeout = 1.0  # CRITICAL: Reduced from 3.0s - lose target faster!
        
        # TARGET POSITION MEMORY SYSTEM
        self.last_known_person_bearing = None  # IMU bearing to person (degrees)
        self.last_known_person_position = None  # (x, y) in robot frame
        self.person_bearing_timestamp = 0.0  # When bearing was stored
        self.pre_avoidance_orientation = None  # IMU orientation BEFORE obstacle avoidance started
        self.in_obstacle_avoidance = False  # Flag: Currently avoiding obstacle
        self.person_lost_timestamp = None  # When person was lost
        self.reacquisition_mode = False  # Are we trying to reacquire the target?
        self.reacquisition_in_progress = False  # EMERGENCY: Block all movement except reacquisition
        self.reacquisition_timeout = 15.0  # seconds - max age of memory
        self.person_lost_threshold = 0.25  # CRITICAL: Reduced to 0.25s - STOP immediately!
        self.emergency_stop_on_loss = True  # CRITICAL: Stop robot when person lost
        
        # Robot state tracking
        self.robot_position = {'x': 0.0, 'y': 0.0, 'orientation': 0.0}
        self.position_history = deque(maxlen=50)
        
        # Navigation performance
        self.navigation_start_time = 0.0
        self.total_navigation_time = 0.0
        self.successful_navigations = 0
        self.failed_navigations = 0
        self.goals_reached = 0
        
        # Force tracking
        self.current_forces = {
            'attractive': (0.0, 0.0),
            'repulsive': (0.0, 0.0),
            'total': (0.0, 0.0)
        }
        
        # Movement command tracking
        self.current_movement_command = None
        self.last_movement_time = 0.0
        self.movement_command_history = deque(maxlen=20)
        
        # Safety and performance monitoring
        self.stuck_detection_enabled = True
        self.stuck_threshold_time = 10.0  # TUNED: Increased from 8.0 seconds
        self.last_significant_movement = time.time()
        self.min_movement_distance = 30.0  # mm
        
        # Debug and analysis
        self.debug_enabled = False
        self.navigation_log = []
    
    def update_robot_state(self, robot_state: Dict[str, Any]):
        """Update robot position and orientation from robot state tracker"""
        try:
            self.robot_position = {
                'x': robot_state.get('position_x', 0.0),
                'y': robot_state.get('position_y', 0.0), 
                'orientation': robot_state.get('orientation', 0.0)
            }
            
            # Add to position history for stuck detection
            current_time = time.time()
            self.position_history.append({
                'x': self.robot_position['x'],
                'y': self.robot_position['y'],
                'timestamp': current_time
            })
            
            # Check for significant movement
            if len(self.position_history) >= 2:
                prev_pos = self.position_history[-2]
                dx = self.robot_position['x'] - prev_pos['x']
                dy = self.robot_position['y'] - prev_pos['y']
                distance_moved = math.sqrt(dx**2 + dy**2)
                
                if distance_moved > self.min_movement_distance:
                    self.last_significant_movement = current_time
                    
        except Exception as e:
            if self.debug_enabled:
                print(f"Error updating robot state: {e}")
    
    def update_person_target(self, person_data: Optional[Dict[str, Any]], 
                           lidar_distance: float, lidar_angle: float):
        """Update person target from detection system with memory"""
        try:
            current_time = time.time()
            
            if person_data and lidar_distance > 0:
                # Convert LiDAR polar to Cartesian
                # In LiDAR frame: 0° = forward, positive angles = right
                angle_rad = math.radians(lidar_angle)
                
                self.person_target = {
                    'x': lidar_distance * math.sin(angle_rad),  # Lateral
                    'y': lidar_distance * math.cos(angle_rad),  # Forward
                    'distance': lidar_distance,
                    'angle': lidar_angle,
                    'confidence': person_data.get('confidence', 0.5)
                }
                self.last_person_update = current_time
                
                # TARGET MEMORY: Store bearing to person (IMU-based)
                # Calculate absolute bearing: robot orientation + relative angle
                current_orientation = self.robot_position.get('orientation', 0.0)
                absolute_bearing = self._normalize_angle(current_orientation + lidar_angle)
                self.last_known_person_bearing = absolute_bearing
                self.last_known_person_position = (self.person_target['x'], self.person_target['y'])
                self.person_bearing_timestamp = current_time
                
                # Store pre-avoidance orientation if not in avoidance yet
                if not self.in_obstacle_avoidance and self.pre_avoidance_orientation is None:
                    self.pre_avoidance_orientation = current_orientation
                    if self.debug_enabled:
                        print(f"📍 Stored pre-avoidance orientation: {self.pre_avoidance_orientation:.1f}°")
                
                # Person found - exit reacquisition mode and obstacle avoidance
                if self.reacquisition_mode:
                    self.reacquisition_mode = False
                    self.reacquisition_in_progress = False
                    self.in_obstacle_avoidance = False
                    self.pre_avoidance_orientation = None  # Reset for next avoidance
                    if self.debug_enabled:
                        print("✓ Target reacquired!")
                
                # Clear person lost tracking
                self.person_lost_timestamp = None
                
            else:
                # Person not detected - track how long we've been without target
                if self.person_target is not None:
                    # Just lost the person
                    if self.person_lost_timestamp is None:
                        self.person_lost_timestamp = current_time
                        if self.debug_enabled:
                            print(f"⚠ Person lost at {current_time:.1f}s")
                
                # Check for person timeout
                if self.person_target and (current_time - self.last_person_update) > self.person_timeout:
                    self.person_target = None
                    
                    # Enter reacquisition mode if we have a memory and been lost long enough
                    if (self.last_known_person_bearing is not None and 
                        self.person_lost_timestamp is not None and
                        (current_time - self.person_lost_timestamp) > self.person_lost_threshold):
                        
                        # Check if memory is still valid (not too old)
                        memory_age = current_time - self.person_bearing_timestamp
                        if memory_age < self.reacquisition_timeout:
                            if not self.reacquisition_mode:
                                self.reacquisition_mode = True
                                self.reacquisition_in_progress = True  # EMERGENCY: Block normal navigation
                                if self.debug_enabled:
                                    print(f"🛑 EMERGENCY: Person lost! Entering reacquisition mode.")
                                    print(f"→ Last bearing: {self.last_known_person_bearing:.1f}°")
                        else:
                            # Memory too old - give up
                            self.last_known_person_bearing = None
                            self.last_known_person_position = None
                            if self.debug_enabled:
                                print("⚠ Memory too old - giving up on reacquisition")
                    
        except Exception as e:
            if self.debug_enabled:
                print(f"Error updating person target: {e}")
    
    def calculate_navigation_forces(self, obstacles: List[Tuple[float, float]]) -> Dict[str, Tuple[float, float]]:
        """Calculate attractive and repulsive forces"""
        try:
            attractive_force = (0.0, 0.0)
            repulsive_force = (0.0, 0.0)
            
            robot_x = self.robot_position['x']
            robot_y = self.robot_position['y']
            
            # Calculate attractive force towards person
            if self.person_target:
                attractive_force = self.field_calculator.calculate_attractive_field(
                    robot_x, robot_y,
                    self.person_target['x'], self.person_target['y']
                )
            
            # Calculate repulsive forces from obstacles
            if obstacles:
                repulsive_force = self.field_calculator.calculate_repulsive_field(
                    robot_x, robot_y, obstacles
                )
            
            # Sum forces
            total_force = self.field_calculator.sum_force_vectors(
                attractive_force, repulsive_force
            )
            
            # Store current forces
            self.current_forces = {
                'attractive': attractive_force,
                'repulsive': repulsive_force,
                'total': total_force
            }
            
            # DETECT OBSTACLE AVOIDANCE: Set flag when repulsive force is significant
            repulsive_magnitude = math.sqrt(repulsive_force[0]**2 + repulsive_force[1]**2)
            if repulsive_magnitude > 2.0:  # Threshold for "significant" repulsion
                if not self.in_obstacle_avoidance:
                    self.in_obstacle_avoidance = True
                    if self.debug_enabled:
                        print(f"🚧 Obstacle avoidance STARTED (repulsive: {repulsive_magnitude:.1f})")
            elif repulsive_magnitude < 0.5:  # Clear when repulsion drops
                if self.in_obstacle_avoidance:
                    self.in_obstacle_avoidance = False
                    if self.debug_enabled:
                        print(f"✓ Obstacle cleared")
            
            return self.current_forces
            
        except Exception as e:
            if self.debug_enabled:
                print(f"Error calculating navigation forces: {e}")
            return {
                'attractive': (0.0, 0.0),
                'repulsive': (0.0, 0.0),
                'total': (0.0, 0.0)
            }
    
    def generate_movement_command(self):
        """Generate movement command from current forces or reacquisition mode"""
        try:
            if not self.navigation_enabled:
                return None
            
            # EMERGENCY STOP: If person lost and emergency mode enabled, STOP immediately
            if (self.emergency_stop_on_loss and 
                self.person_lost_timestamp is not None and 
                not self.reacquisition_mode):
                # Person just lost, not yet in reacquisition - STOP!
                current_time = time.time()
                time_since_loss = current_time - self.person_lost_timestamp
                if time_since_loss < self.person_lost_threshold:
                    if self.debug_enabled:
                        print(f"🛑 EMERGENCY STOP: Person lost {time_since_loss:.1f}s ago, waiting for reacquisition")
                    self.current_movement_command = None
                    return None
            
            # CRITICAL: REACQUISITION MODE - OVERRIDE ALL OTHER FORCES
            if self.reacquisition_in_progress:
                reacq_command = self.generate_reacquisition_movement()
                if reacq_command:
                    # Convert VelocityConfig to command dict
                    command = {
                        'direction': reacq_command.direction.name,
                        'speed': reacq_command.speed
                    }
                    
                    # Track command
                    current_time = time.time()
                    self.current_movement_command = {
                        'direction': reacq_command.direction.name,
                        'speed': reacq_command.speed,
                        'timestamp': current_time,
                        'force_magnitude': 0.0  # Not force-driven
                    }
                    
                    if self.debug_enabled:
                        print(f"↻ Reacquisition: {reacq_command.direction.name} at {reacq_command.speed:.2f}")
                    
                    return command
                else:
                    # Reacquisition failed - exit mode
                    self.reacquisition_mode = False
                    self.reacquisition_in_progress = False
            
            # NORMAL NAVIGATION MODE: Use potential fields
            # SAFETY: Stop if goal (person) is reached
            if self.check_goal_reached():
                if self.debug_enabled:
                    print("Goal reached - stopping movement")
                self.current_movement_command = None
                return None
            
            total_force = self.current_forces['total']
            
            # FIXED: Use orientation = 0.0 because forces are already calculated in 
            # robot-relative coordinates (based on LiDAR angle where 0 = directly in front).
            direction_str, speed = self.field_calculator.convert_force_to_velocity(
                total_force[0], total_force[1], 0.0
            )
            
            # Create movement command
            if direction_str != "NONE" and speed > 0:
                command = {
                    'direction': direction_str,
                    'speed': speed
                }
                
                # Track command
                current_time = time.time()
                self.current_movement_command = {
                    'direction': direction_str,
                    'speed': speed,
                    'timestamp': current_time,
                    'force_magnitude': math.sqrt(total_force[0]**2 + total_force[1]**2)
                }
                
                self.movement_command_history.append(self.current_movement_command)
                self.last_movement_time = current_time
                
                if self.debug_enabled:
                    print(f"Movement command: {direction_str} at {speed:.2f}")
                
                return command
            else:
                # No movement needed
                self.current_movement_command = None
                return None
                
        except Exception as e:
            if self.debug_enabled:
                print(f"Error generating movement command: {e}")
            return None
    
    def check_goal_reached(self) -> bool:
        """Check if navigation goal has been reached"""
        try:
            if not self.person_target:
                return False
            
            goal_reached = self.field_calculator.is_goal_reached(
                self.robot_position['x'], self.robot_position['y'],
                self.person_target['x'], self.person_target['y']
            )
            
            if goal_reached and self.navigation_enabled:
                self.goals_reached += 1
                if self.debug_enabled:
                    print("Navigation goal reached!")
                
            return goal_reached
            
        except Exception as e:
            if self.debug_enabled:
                print(f"Error checking goal reached: {e}")
            return False
    
    def check_stuck_condition(self) -> bool:
        """Check if robot is stuck in local minimum or not making progress"""
        try:
            if not self.stuck_detection_enabled:
                return False
            
            current_time = time.time()
            
            # Check if robot hasn't moved significantly in a while
            time_since_movement = current_time - self.last_significant_movement
            if time_since_movement > self.stuck_threshold_time:
                if self.debug_enabled:
                    print(f"Robot appears stuck - no significant movement for {time_since_movement:.1f}s")
                return True
            
            # Check position history for local minimum detection
            if len(self.position_history) >= 10:
                # FIXED: Convert deque to list before slicing
                recent_positions = [(pos['x'], pos['y']) for pos in list(self.position_history)[-10:]]
                if self.field_calculator.check_local_minimum(recent_positions):
                    if self.debug_enabled:
                        print("Robot detected in local minimum")
                    return True
            
            return False
            
        except Exception as e:
            if self.debug_enabled:
                print(f"Error checking stuck condition: {e}")
            return False
    
    def enable_navigation(self):
        """Enable autonomous navigation"""
        if not self.navigation_enabled:
            self.navigation_enabled = True
            self.navigation_start_time = time.time()
            
            # FIXED: Reset stuck detection timer when enabling navigation
            self.last_significant_movement = time.time()
            
            # Clear position history to start fresh
            self.position_history.clear()
            
            self.state_machine.start_navigation()
            
            if self.debug_enabled:
                print("Navigation ENABLED")
    
    def disable_navigation(self):
        """Disable autonomous navigation"""
        if self.navigation_enabled:
            self.navigation_enabled = False
            if self.navigation_start_time > 0:
                self.total_navigation_time += time.time() - self.navigation_start_time
                self.navigation_start_time = 0
            
            self.state_machine.stop_navigation()
            self.current_movement_command = None
            
            if self.debug_enabled:
                print("Navigation DISABLED")
    
    def get_navigation_status(self) -> Dict[str, Any]:
        """Get comprehensive navigation status - FIXED SLICE INDEXING"""
        try:
            # Check state machine
            try:
                current_state = self.state_machine.get_current_state()
            except Exception:
                current_state = "ERROR"
            
            # Check field calculator
            try:
                force_stats = self.field_calculator.get_force_statistics()
            except Exception:
                force_stats = {
                    'calculation_count': 0,
                    'avg_attractive_magnitude': 0.0,
                    'avg_repulsive_magnitude': 0.0,
                    'avg_total_magnitude': 0.0
                }
            
            # Check goal reached
            try:
                goal_reached = self.check_goal_reached()
            except Exception:
                goal_reached = False
            
            # Calculate navigation performance metrics
            try:
                total_navigations = self.successful_navigations + self.failed_navigations
                success_rate = (self.successful_navigations / max(1, total_navigations)) * 100
            except Exception:
                success_rate = 0.0
            
            # FIXED: Check movement command history with proper deque handling
            try:
                avg_command_interval = 0.0
                if len(self.movement_command_history) > 1:
                    # FIXED: Convert deque to list before slicing
                    recent_commands = list(self.movement_command_history)[-10:]
                    times = [cmd['timestamp'] for cmd in recent_commands]
                    if len(times) > 1:
                        intervals = [times[i+1] - times[i] for i in range(len(times)-1)]
                        avg_command_interval = sum(intervals) / len(intervals)
            except Exception:
                avg_command_interval = 0.0
            
            # Build status dict
            try:
                status = {
                    # Navigation state
                    'navigation_enabled': self.navigation_enabled,
                    'current_state': current_state,
                    'has_person_target': self.person_target is not None,
                    'goal_reached': goal_reached,
                    'robot_stuck': self.check_stuck_condition(),
                    
                    # Target information
                    'person_distance': self.person_target['distance'] if self.person_target else 0.0,
                    'person_angle': self.person_target['angle'] if self.person_target else 0.0,
                    'time_since_person_update': time.time() - self.last_person_update,
                    
                    # Force information
                    'attractive_force_magnitude': math.sqrt(sum(x**2 for x in self.current_forces['attractive'])),
                    'repulsive_force_magnitude': math.sqrt(sum(x**2 for x in self.current_forces['repulsive'])),
                    'total_force_magnitude': math.sqrt(sum(x**2 for x in self.current_forces['total'])),
                    
                    # Movement information
                    'current_movement_direction': self.current_movement_command['direction'] if self.current_movement_command else 'NONE',
                    'current_movement_speed': self.current_movement_command['speed'] if self.current_movement_command else 0.0,
                    'time_since_last_movement': time.time() - self.last_movement_time if self.last_movement_time > 0 else 0,
                    
                    # Performance metrics
                    'successful_navigations': self.successful_navigations,
                    'failed_navigations': self.failed_navigations,
                    'goals_reached': self.goals_reached,
                    'navigation_success_rate': success_rate,
                    'total_navigation_time': self.total_navigation_time,
                    'avg_movement_command_interval': avg_command_interval,
                    
                    # Force statistics
                    'force_calculations_count': force_stats['calculation_count'],
                    'avg_attractive_magnitude': force_stats['avg_attractive_magnitude'],
                    'avg_repulsive_magnitude': force_stats['avg_repulsive_magnitude'],
                    'avg_total_force_magnitude': force_stats['avg_total_magnitude'],
                    
                    # Target reacquisition status (NEW)
                    'reacquisition_mode': self.reacquisition_mode,
                    'has_target_memory': self.last_known_person_bearing is not None,
                    'last_known_bearing': self.last_known_person_bearing if self.last_known_person_bearing is not None else 0.0,
                    'target_bearing': self.calculate_reacquisition_bearing() if self.last_known_person_bearing is not None else 0.0,
                    'time_since_person_lost': time.time() - self.person_lost_timestamp if self.person_lost_timestamp else 0.0,
                    
                    # Field parameters
                    'attractive_strength': self.field_calculator.attractive_strength,
                    'repulsive_strength': self.field_calculator.repulsive_strength,
                    'goal_reached_threshold': self.field_calculator.goal_reached_threshold,
                }
                
                return status
                
            except Exception as e:
                # Return minimal error status if status building fails
                return {
                    'navigation_enabled': False, 
                    'current_state': 'ERROR',
                    'has_person_target': False,
                    'goal_reached': False,
                    'robot_stuck': True,
                    'person_distance': 0.0,
                    'person_angle': 0.0,
                    'time_since_person_update': 0,
                    'attractive_force_magnitude': 0.0,
                    'repulsive_force_magnitude': 0.0,
                    'total_force_magnitude': 0.0,
                    'current_movement_direction': 'NONE',
                    'current_movement_speed': 0.0,
                    'time_since_last_movement': 0,
                    'successful_navigations': 0,
                    'failed_navigations': 0,
                    'goals_reached': 0,
                    'navigation_success_rate': 0.0,
                    'total_navigation_time': 0.0,
                    'avg_movement_command_interval': 0.0,
                    'force_calculations_count': 0,
                    'avg_attractive_magnitude': 0.0,
                    'avg_repulsive_magnitude': 0.0,
                    'avg_total_force_magnitude': 0.0,
                    'attractive_strength': 0.0,
                    'repulsive_strength': 0.0,
                    'goal_reached_threshold': 0.0
                }
                
        except Exception as e:
            # Fallback minimal status
            return {'navigation_enabled': False, 'current_state': 'ERROR'}
    
    def update_navigation_state(self, person_data: Optional[Dict[str, Any]], 
                              obstacles: List[Tuple[float, float]]) -> str:
        """Update navigation state machine based on current conditions"""
        try:
            # Update state machine with current conditions
            has_person = person_data is not None and self.person_target is not None
            goal_reached = self.check_goal_reached() if has_person else False
            is_stuck = self.check_stuck_condition()
            has_obstacles = len(obstacles) > 0
            
            return self.state_machine.update_state(
                has_person_target=has_person,
                goal_reached=goal_reached,
                robot_stuck=is_stuck,
                obstacles_detected=has_obstacles,
                navigation_enabled=self.navigation_enabled
            )
            
        except Exception as e:
            if self.debug_enabled:
                print(f"Error updating navigation state: {e}")
            return "ERROR"
    
    def set_debug_mode(self, enabled: bool):
        """Enable/disable debug output"""
        self.debug_enabled = enabled
        
    def reset_navigation_statistics(self):
        """Reset navigation performance statistics"""
        self.successful_navigations = 0
        self.failed_navigations = 0
        self.goals_reached = 0
        self.total_navigation_time = 0.0
        self.navigation_log.clear()
        
        if self.debug_enabled:
            print("Navigation statistics reset")
    
    # ============================================================================
    # TARGET REACQUISITION METHODS (NEW)
    # ============================================================================
    
    def _normalize_angle(self, angle_deg: float) -> float:
        """Normalize angle to [-180, 180] range"""
        while angle_deg > 180:
            angle_deg -= 360
        while angle_deg < -180:
            angle_deg += 360
        return angle_deg
    
    def calculate_reacquisition_bearing(self) -> Optional[float]:
        """
        Calculate the bearing to turn toward to reacquire the person.
        Uses IMU to compensate for rotation during obstacle avoidance.
        
        Returns:
            Target bearing in degrees (robot-relative), or None if no memory
        """
        try:
            if self.last_known_person_bearing is None:
                return None
            
            current_orientation = self.robot_position.get('orientation', 0.0)
            
            # Calculate relative bearing to the last known person position
            # This accounts for robot rotation since person was last seen
            relative_bearing = self._normalize_angle(
                self.last_known_person_bearing - current_orientation
            )
            
            return relative_bearing
            
        except Exception as e:
            if self.debug_enabled:
                print(f"Error calculating reacquisition bearing: {e}")
            return None
    
    def generate_reacquisition_movement(self) -> Optional[Any]:
        """
        NEW METHODOLOGY: STOP completely during reacquisition.
        Head and body will rotate to find person, but NO forward/backward motion.
        
        Returns:
            NONE movement command (full stop)
        """
        try:
            from systems.movement.velocity_config import VelocityConfig, MovementDirection
            
            # CRITICAL: Complete STOP - no robot motion during reacquisition
            # Head tracking and body rotation handled separately by main system
            if self.debug_enabled:
                print(f"🛑 REACQUISITION MODE: Robot STOPPED (head/body will scan)")
            
            return VelocityConfig(MovementDirection.NONE, 0.0)
            
        except Exception as e:
            if self.debug_enabled:
                print(f"Error generating reacquisition movement: {e}")
            return None
    
    def get_reacquisition_control_data(self) -> Optional[Dict[str, Any]]:
        """
        Get data needed for reacquisition control (head tracking and body rotation).
        
        Returns:
            Dictionary with:
            - in_reacquisition: bool
            - head_direction: 'left', 'right', or 'center' (based on last screen position)
            - body_target_orientation: float (pre-avoidance orientation)
            - current_orientation: float (current robot orientation)
        """
        try:
            if not self.reacquisition_in_progress:
                return None
            
            # Default head direction to center (no screen position available)
            head_direction = 'center'
            
            current_orientation = self.robot_position.get('orientation', 0.0)
            
            return {
                'in_reacquisition': True,
                'head_direction': head_direction,
                'body_target_orientation': self.pre_avoidance_orientation,
                'current_orientation': current_orientation,
                'bearing_error': self._normalize_angle(
                    self.pre_avoidance_orientation - current_orientation
                ) if self.pre_avoidance_orientation is not None else 0.0
            }
            
        except Exception as e:
            if self.debug_enabled:
                print(f"Error getting reacquisition control data: {e}")
            return None
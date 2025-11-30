#!/usr/bin/env python3
"""
Optimized Head Controller System
OPTIMIZED NO VALIDATION head tracking controller with anti-oscillation smoothing
- MAINTAINED: ONLY confidence validation (65% initial, 40% continue)
- ENHANCED: Added position smoothing and enhanced dead zone to prevent oscillation
- OPTIMIZED: Removed redundant code while preserving functionality
"""
import time
from collections import deque

# Import UltraBorg directly for servo control
import UltraBorg

class OptimizedNoValidationHeadController:
    """
    OPTIMIZED NO VALIDATION head tracking controller with anti-oscillation smoothing
    - MAINTAINED: ONLY confidence validation (65% initial, 40% continue)
    - ENHANCED: Added position smoothing and enhanced dead zone to prevent oscillation
    - OPTIMIZED: Removed redundant code while preserving functionality
    """
    
    def __init__(self):
        # UltraBorg servo control
        self.board = UltraBorg.UltraBorg()
        self.board.i2cAddress = 11
        self.servo_initialized = False
        
        # ENHANCED: Movement parameters with anti-oscillation features
        self.step_size = 0.02
        self.target_center = 910
        # ENHANCED: Wider dead zone to reduce oscillation
        self.dead_zone_min = 800  # Was 850 - wider by 50px each side
        self.dead_zone_max = 1020  # Was 970 - wider by 50px each side
        self.current_position = 0.0
        
        # ENHANCED: Anti-oscillation parameters
        self.min_movement_threshold = 15  # Ignore movements smaller than 15px
        self.position_smoothing_window = 5  # Average last 5 positions
        
        # Person detection state
        self.person_detected = False
        self.last_person_x = None
        self.detection_timeout = 1.0
        self.last_detection_time = 0.0
        
        # ENHANCED: Position smoothing buffers
        self.raw_position_history = deque(maxlen=self.position_smoothing_window)
        self.smoothed_position = None
        self.last_smoothed_position = None
        
        # Movement control
        self.is_moving = False
        self.last_move_time = 0.0
        self.move_start_time = 0.0
        self.servo_settle_time = 0.2
        self.min_move_interval = 0.1
        
        # MAINTAINED: ONLY CONFIDENCE THRESHOLDS - NO OTHER VALIDATION
        self.initial_confidence_threshold = 0.65
        self.continue_confidence_threshold = 0.4
        
        # Temporal consistency
        self.detection_history = deque(maxlen=3)
        
        # Statistics
        self.total_moves = 0
        self.successful_centers = 0
        self.false_positive_rejections = 0
        self.consecutive_successes = 0
        self.consecutive_failures = 0
        # ENHANCED: Anti-oscillation statistics
        self.smoothing_applied = 0
    
    def initialize_servo(self):
        """Initialize UltraBorg servo system"""
        try:
            self.board.Init()
            self.board.SetWithRetry(self.board.SetServoMaximum3, self.board.GetServoMaximum3, 5085, 5)
            self.board.SetWithRetry(self.board.SetServoMinimum3, self.board.GetServoMinimum3, 1550, 5)
            self.board.SetWithRetry(self.board.SetServoStartup3, self.board.GetServoStartup3, 3565, 5)
            self.board.SetServoPosition3(0.0)
            self.current_position = 0.0
            self.servo_initialized = True
            return True
        except Exception:
            self.servo_initialized = False
            return False
    
    def is_person_centered(self, x_pixels):
        """Check if person is within acceptable center range"""
        return self.dead_zone_min <= x_pixels <= self.dead_zone_max
    
    def smooth_position(self, raw_x_pixels):
        """ENHANCED: Apply position smoothing to reduce oscillation (no outlier rejection)"""
        try:
            # Add to history
            self.raw_position_history.append(raw_x_pixels)
            
            # Calculate smoothed position
            if len(self.raw_position_history) >= 2:
                # Use weighted average with more weight on recent positions
                positions = list(self.raw_position_history)
                weights = [(i + 1) for i in range(len(positions))]  # More weight to recent
                weighted_sum = sum(pos * weight for pos, weight in zip(positions, weights))
                weight_sum = sum(weights)
                smoothed = weighted_sum / weight_sum
                self.smoothing_applied += 1
            else:
                smoothed = raw_x_pixels
            
            self.smoothed_position = smoothed
            self.last_smoothed_position = smoothed
            return smoothed
            
        except Exception:
            return raw_x_pixels
    
    def should_move_servo(self, smoothed_x_pixels):
        """ENHANCED: Determine if servo should move based on smoothed position and thresholds"""
        if smoothed_x_pixels is None:
            return False
        
        # Check if already centered
        if self.is_person_centered(smoothed_x_pixels):
            return False
        
        # FIXED: Check minimum movement threshold against TARGET CENTER, not previous position
        # This prevents tiny movements near center but allows movement when far from center
        distance_from_center = abs(smoothed_x_pixels - self.target_center)
        if distance_from_center < self.min_movement_threshold:
            return False
        
        return True
    
    def can_move_now(self):
        """Check if enough time has passed to allow movement"""
        current_time = time.time()
        if self.is_moving and (current_time - self.move_start_time) < self.servo_settle_time:
            return False
        if (current_time - self.last_move_time) < self.min_move_interval:
            return False
        return True
    
    def center_servo(self):
        """Move servo to center position"""
        if not self.servo_initialized:
            return False
        try:
            self.board.SetServoPosition3(0.0)
            self.current_position = 0.0
            return True
        except:
            return False
    
    def confidence_only_validation(self, detection_data):
        """MAINTAINED: CONFIDENCE ONLY VALIDATION - no size, position, or depth checks"""
        try:
            confidence = detection_data.get('confidence', 0)
            
            # Choose confidence threshold based on current detection state
            if self.person_detected:
                conf_thresh = self.continue_confidence_threshold
                mode = "CONTINUE"
            else:
                conf_thresh = self.initial_confidence_threshold
                mode = "INITIAL"
            
            # MAINTAINED: ONLY check confidence - accept EVERYTHING else
            if confidence < conf_thresh:
                return False, f"{mode}_confidence_too_low_{confidence:.3f}<{conf_thresh}"
            
            return True, f"{mode}_confidence_valid"
            
        except Exception as e:
            return False, f"validation_error_{str(e)[:20]}"
    
    def simple_temporal_consistency(self, is_valid_detection):
        """Simple temporal consistency - just need 1 valid detection"""
        self.detection_history.append(is_valid_detection)
        
        if not self.person_detected:
            recent_valid = sum(1 for d in list(self.detection_history)[-2:] if d)
            return recent_valid >= 1
        
        recent_valid = sum(1 for d in list(self.detection_history)[-3:] if d)
        return recent_valid >= 1
    
    def update_person_detection(self, person_data):
        """ENHANCED: Update person detection with confidence validation and position smoothing"""
        current_time = time.time()
        previous_detected = self.person_detected
        
        if person_data is not None:
            is_valid, reason = self.confidence_only_validation(person_data)
            
            if is_valid:
                is_temporally_consistent = self.simple_temporal_consistency(True)
                
                if is_temporally_consistent:
                    self.person_detected = True
                    # ENHANCED: Apply position smoothing to reduce oscillation
                    raw_x_pixels = person_data['bbox_center']['x_pixels']
                    smoothed_x_pixels = self.smooth_position(raw_x_pixels)
                    self.last_person_x = smoothed_x_pixels  # Use smoothed position
                    self.last_detection_time = current_time
                    self.consecutive_successes += 1
                    self.consecutive_failures = 0
                else:
                    self.simple_temporal_consistency(False)
                    self.consecutive_failures += 1
                    self.consecutive_successes = 0
                    if not previous_detected:
                        self.person_detected = False
                        self.last_person_x = None
            else:
                self.simple_temporal_consistency(False)
                self.consecutive_failures += 1
                self.consecutive_successes = 0
                if current_time - self.last_detection_time > self.detection_timeout:
                    self.person_detected = False
                    self.last_person_x = None
        else:
            self.simple_temporal_consistency(False)
            self.consecutive_failures += 1
            self.consecutive_successes = 0
            if current_time - self.last_detection_time > self.detection_timeout:
                self.person_detected = False
                self.last_person_x = None
    
    def update_movement_state(self):
        """Update internal movement state"""
        current_time = time.time()
        if self.is_moving and (current_time - self.move_start_time) >= self.servo_settle_time:
            self.is_moving = False
    
    def calculate_movement_direction(self, x_pixels):
        """Calculate which direction to move based on person position"""
        if x_pixels < self.dead_zone_min:
            return 1  # Move servo right (positive)
        elif x_pixels > self.dead_zone_max:
            return -1  # Move servo left (negative)
        else:
            return 0  # Centered
    
    def execute_head_tracking(self):
        """ENHANCED: Execute head tracking with anti-oscillation logic"""
        self.update_movement_state()
        
        if not self.person_detected or self.last_person_x is None:
            return
        
        # ENHANCED: Use smoothed position and enhanced movement logic
        if self.is_person_centered(self.last_person_x):
            self.successful_centers += 1
            return
        
        if not self.can_move_now():
            return
        
        # ENHANCED: Check if movement is actually needed (anti-oscillation)
        if not self.should_move_servo(self.last_person_x):
            return
        
        direction = self.calculate_movement_direction(self.last_person_x)
        
        if direction != 0:
            new_position = self.current_position + (direction * self.step_size)
            new_position = max(-0.8, min(0.8, new_position))
            
            try:
                self.board.SetServoPosition3(new_position)
                self.current_position = new_position
                current_time = time.time()
                self.last_move_time = current_time
                self.move_start_time = current_time
                self.is_moving = True
                self.total_moves += 1
            except Exception:
                pass
    
    def get_status(self):
        """Get current status for display including anti-oscillation stats"""
        return {
            'initialized': self.servo_initialized,
            'position': self.current_position,
            'is_moving': self.is_moving,
            'person_detected': self.person_detected,
            'person_x': self.last_person_x,
            'person_centered': self.is_person_centered(self.last_person_x) if self.last_person_x else False,
            'total_moves': self.total_moves,
            'successful_centers': self.successful_centers,
            'false_positive_rejections': self.false_positive_rejections,
            'can_move': self.can_move_now(),
            'time_since_last_move': time.time() - self.last_move_time if self.last_move_time > 0 else 0,
            'consecutive_successes': self.consecutive_successes,
            'consecutive_failures': self.consecutive_failures,
            # ENHANCED: Anti-oscillation statistics
            'smoothing_applied': self.smoothing_applied,
            'smoothed_position': self.smoothed_position
        }
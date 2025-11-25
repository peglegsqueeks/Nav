#!/usr/bin/env python3
"""
Navigation State Machine
Manages navigation states and state transitions for potential field navigation
Handles edge cases, stuck conditions, and recovery strategies
"""
import time
from enum import Enum
from typing import Dict, Any, List
from collections import deque

class NavigationState(Enum):
    """Navigation state enumeration"""
    IDLE = "IDLE"                           # Navigation disabled
    SEARCHING = "SEARCHING"                 # Looking for person target
    NAVIGATING = "NAVIGATING"               # Actively navigating to person
    GOAL_REACHED = "GOAL_REACHED"           # Successfully reached person
    AVOIDING_OBSTACLES = "AVOIDING_OBSTACLES" # Primarily avoiding obstacles
    STUCK = "STUCK"                         # Stuck in local minimum
    RECOVERING = "RECOVERING"               # Executing recovery strategy
    ERROR = "ERROR"                         # Error condition

class RecoveryStrategy(Enum):
    """Recovery strategy types"""
    NONE = "NONE"
    RANDOM_WALK = "RANDOM_WALK"
    SPIRAL_OUT = "SPIRAL_OUT"
    BACKUP_AND_TURN = "BACKUP_AND_TURN"

class NavigationStateMachine:
    """
    Navigation state machine for handling complex navigation scenarios
    Provides robust state transitions and recovery mechanisms
    """
    
    def __init__(self):
        # Current state
        self.current_state = NavigationState.IDLE
        self.previous_state = NavigationState.IDLE
        self.state_entry_time = time.time()
        self.state_duration = 0.0
        
        # State transition history
        self.state_history = deque(maxlen=20)
        self.transition_count = {}
        
        # State-specific timers and thresholds
        self.searching_timeout = 10.0  # seconds
        self.goal_reached_hold_time = 3.0  # seconds
        self.stuck_detection_time = 5.0  # seconds
        self.recovery_timeout = 8.0  # seconds
        self.obstacle_avoidance_time = 8.0  # seconds
        
        # Recovery system
        self.current_recovery_strategy = RecoveryStrategy.NONE
        self.recovery_start_time = 0.0
        self.recovery_attempts = 0
        self.max_recovery_attempts = 3
        
        # State conditions tracking
        self.last_person_seen_time = 0.0
        self.last_goal_reached_time = 0.0
        self.consecutive_stuck_detections = 0
        self.obstacle_avoidance_start_time = 0.0
        
        # Performance tracking
        self.total_state_changes = 0
        self.time_in_each_state = {state: 0.0 for state in NavigationState}
        self.successful_navigations = 0
        self.failed_navigations = 0
    
    def start_navigation(self):
        """Start navigation - transition from IDLE to SEARCHING"""
        if self.current_state == NavigationState.IDLE:
            self.transition_to_state(NavigationState.SEARCHING)
    
    def stop_navigation(self):
        """Stop navigation - transition to IDLE state"""
        self.transition_to_state(NavigationState.IDLE)
        self.current_recovery_strategy = RecoveryStrategy.NONE
        self.recovery_attempts = 0
    
    def transition_to_state(self, new_state: NavigationState):
        """
        Transition to a new state with proper cleanup and initialization
        
        Args:
            new_state: Target navigation state
        """
        if new_state == self.current_state:
            return  # No transition needed
        
        # Record time spent in current state
        current_time = time.time()
        self.state_duration = current_time - self.state_entry_time
        self.time_in_each_state[self.current_state] += self.state_duration
        
        # Record transition
        self.state_history.append({
            'from_state': self.current_state.value,
            'to_state': new_state.value,
            'timestamp': current_time,
            'duration_in_previous': self.state_duration
        })
        
        # Update transition count
        transition_key = f"{self.current_state.value}->{new_state.value}"
        self.transition_count[transition_key] = self.transition_count.get(transition_key, 0) + 1
        
        # State exit actions
        self.on_state_exit(self.current_state)
        
        # Update state
        self.previous_state = self.current_state
        self.current_state = new_state
        self.state_entry_time = current_time
        self.total_state_changes += 1
        
        # State entry actions
        self.on_state_entry(new_state)
    
    def on_state_entry(self, state: NavigationState):
        """Actions to perform when entering a state"""
        if state == NavigationState.SEARCHING:
            pass  # No special actions for searching
        elif state == NavigationState.NAVIGATING:
            pass  # No special actions for navigating
        elif state == NavigationState.GOAL_REACHED:
            self.last_goal_reached_time = time.time()
            self.successful_navigations += 1
        elif state == NavigationState.STUCK:
            self.consecutive_stuck_detections += 1
        elif state == NavigationState.RECOVERING:
            self.recovery_start_time = time.time()
            self.select_recovery_strategy()
        elif state == NavigationState.AVOIDING_OBSTACLES:
            self.obstacle_avoidance_start_time = time.time()
    
    def on_state_exit(self, state: NavigationState):
        """Actions to perform when exiting a state"""
        if state == NavigationState.STUCK:
            # Reset stuck detection counter when successfully leaving stuck state
            if self.current_state != NavigationState.RECOVERING:
                self.consecutive_stuck_detections = 0
        elif state == NavigationState.RECOVERING:
            if self.current_state not in [NavigationState.STUCK, NavigationState.ERROR]:
                # Recovery was successful
                self.recovery_attempts = 0
                self.current_recovery_strategy = RecoveryStrategy.NONE
    
    def select_recovery_strategy(self):
        """Select appropriate recovery strategy based on situation"""
        # Cycle through recovery strategies
        if self.recovery_attempts == 0:
            self.current_recovery_strategy = RecoveryStrategy.BACKUP_AND_TURN
        elif self.recovery_attempts == 1:
            self.current_recovery_strategy = RecoveryStrategy.RANDOM_WALK
        else:
            self.current_recovery_strategy = RecoveryStrategy.SPIRAL_OUT
        
        self.recovery_attempts += 1
    
    def update_state(self, has_person_target: bool, goal_reached: bool, 
                    robot_stuck: bool, obstacles_detected: bool, 
                    navigation_enabled: bool) -> str:
        """
        Update state machine based on current conditions
        
        Args:
            has_person_target: Whether person is currently detected
            goal_reached: Whether navigation goal has been reached
            robot_stuck: Whether robot appears to be stuck
            obstacles_detected: Whether obstacles are detected
            navigation_enabled: Whether navigation is enabled
            
        Returns:
            str: Current state name
        """
        current_time = time.time()
        
        # Track person detection
        if has_person_target:
            self.last_person_seen_time = current_time
        
        # Update state duration
        self.state_duration = current_time - self.state_entry_time
        
        # Handle navigation disabled
        if not navigation_enabled:
            if self.current_state != NavigationState.IDLE:
                self.transition_to_state(NavigationState.IDLE)
            return self.current_state.value
        
        # State-specific logic
        if self.current_state == NavigationState.IDLE:
            if navigation_enabled:
                self.transition_to_state(NavigationState.SEARCHING)
        
        elif self.current_state == NavigationState.SEARCHING:
            if has_person_target:
                self.transition_to_state(NavigationState.NAVIGATING)
            elif self.state_duration > self.searching_timeout:
                # Been searching too long - maybe implement search pattern
                pass  # Stay in searching for now
        
        elif self.current_state == NavigationState.NAVIGATING:
            if goal_reached:
                self.transition_to_state(NavigationState.GOAL_REACHED)
            elif not has_person_target:
                # Lost person target
                self.transition_to_state(NavigationState.SEARCHING)
            elif robot_stuck:
                self.transition_to_state(NavigationState.STUCK)
            elif obstacles_detected and self.should_prioritize_obstacle_avoidance():
                self.transition_to_state(NavigationState.AVOIDING_OBSTACLES)
        
        elif self.current_state == NavigationState.GOAL_REACHED:
            if self.state_duration > self.goal_reached_hold_time:
                if has_person_target and not goal_reached:
                    # Person moved away - start navigating again
                    self.transition_to_state(NavigationState.NAVIGATING)
                else:
                    # Stay at goal or return to searching
                    self.transition_to_state(NavigationState.SEARCHING)
        
        elif self.current_state == NavigationState.AVOIDING_OBSTACLES:
            if goal_reached:
                self.transition_to_state(NavigationState.GOAL_REACHED)
            elif not obstacles_detected or self.state_duration > self.obstacle_avoidance_time:
                # Obstacles cleared or timeout - return to navigation
                if has_person_target:
                    self.transition_to_state(NavigationState.NAVIGATING)
                else:
                    self.transition_to_state(NavigationState.SEARCHING)
            elif robot_stuck:
                self.transition_to_state(NavigationState.STUCK)
        
        elif self.current_state == NavigationState.STUCK:
            if not robot_stuck:
                # No longer stuck - resume navigation
                if has_person_target:
                    self.transition_to_state(NavigationState.NAVIGATING)
                else:
                    self.transition_to_state(NavigationState.SEARCHING)
            elif self.state_duration > self.stuck_detection_time:
                # Been stuck too long - start recovery
                self.transition_to_state(NavigationState.RECOVERING)
        
        elif self.current_state == NavigationState.RECOVERING:
            if goal_reached:
                self.transition_to_state(NavigationState.GOAL_REACHED)
            elif not robot_stuck and has_person_target:
                # Recovery successful - resume navigation
                self.transition_to_state(NavigationState.NAVIGATING)
            elif self.state_duration > self.recovery_timeout:
                if self.recovery_attempts >= self.max_recovery_attempts:
                    # Too many recovery attempts failed
                    self.failed_navigations += 1
                    self.transition_to_state(NavigationState.ERROR)
                else:
                    # Try again with different recovery strategy
                    self.transition_to_state(NavigationState.STUCK)
        
        elif self.current_state == NavigationState.ERROR:
            # Manual intervention required or reset navigation
            if self.state_duration > 10.0:  # Auto-recover after 10 seconds
                self.transition_to_state(NavigationState.SEARCHING)
                self.recovery_attempts = 0
        
        return self.current_state.value
    
    def should_prioritize_obstacle_avoidance(self) -> bool:
        """
        Determine if obstacle avoidance should take priority over person following
        
        Returns:
            bool: True if should prioritize obstacle avoidance
        """
        # This could be enhanced with more sophisticated logic
        # For now, simple heuristic based on recent state history
        recent_states = [entry['to_state'] for entry in list(self.state_history)[-5:]]
        stuck_count = recent_states.count('STUCK')
        
        # If robot has been getting stuck frequently, prioritize obstacle avoidance
        return stuck_count >= 2
    
    def get_current_state(self) -> str:
        """Get current navigation state as string"""
        return self.current_state.value
    
    def get_recovery_strategy(self) -> str:
        """Get current recovery strategy as string"""
        return self.current_recovery_strategy.value
    
    def get_state_duration(self) -> float:
        """Get time spent in current state (seconds)"""
        return time.time() - self.state_entry_time
    
    def get_state_statistics(self) -> Dict[str, Any]:
        """Get comprehensive state machine statistics"""
        current_time = time.time()
        current_state_duration = current_time - self.state_entry_time
        
        # Calculate state distribution
        total_time = sum(self.time_in_each_state.values()) + current_state_duration
        state_percentages = {}
        for state, time_spent in self.time_in_each_state.items():
            if state == self.current_state:
                time_spent += current_state_duration
            percentage = (time_spent / max(total_time, 1.0)) * 100
            state_percentages[state.value] = percentage
        
        return {
            'current_state': self.current_state.value,
            'previous_state': self.previous_state.value,
            'current_state_duration': current_state_duration,
            'total_state_changes': self.total_state_changes,
            'state_percentages': state_percentages,
            'transition_count': self.transition_count.copy(),
            'recovery_strategy': self.current_recovery_strategy.value,
            'recovery_attempts': self.recovery_attempts,
            'successful_navigations': self.successful_navigations,
            'failed_navigations': self.failed_navigations,
            'consecutive_stuck_detections': self.consecutive_stuck_detections,
            'time_since_person_seen': current_time - self.last_person_seen_time if self.last_person_seen_time > 0 else 0
        }
    
    def reset_statistics(self):
        """Reset all state machine statistics"""
        self.state_history.clear()
        self.transition_count.clear()
        self.time_in_each_state = {state: 0.0 for state in NavigationState}
        self.total_state_changes = 0
        self.successful_navigations = 0
        self.failed_navigations = 0
        self.recovery_attempts = 0
        self.consecutive_stuck_detections = 0
    
    def force_state(self, state_name: str):
        """
        Force transition to specific state (for debugging/testing)
        
        Args:
            state_name: Name of state to transition to
        """
        try:
            target_state = NavigationState(state_name.upper())
            self.transition_to_state(target_state)
        except ValueError:
            pass  # Invalid state name - ignore
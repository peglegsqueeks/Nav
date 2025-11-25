#!/usr/bin/env python3
"""
ENHANCED COORDINATE SENSOR FUSION WITH ROBOT MOVEMENT AND NAVIGATION - MAIN ENTRY POINT
- ENHANCED: Integrated robot base movement using motoron controller
- ENHANCED: IMU integration for robot orientation tracking
- ENHANCED: Movement state tracking and safety mechanisms
- ENHANCED: Extended CSV logging with movement and IMU metrics
- STEP 2 COMPLETE: Potential Field Navigation System
- MAINTAINED: All existing person detection, LiDAR, and head tracking functionality
- REFACTORED: Modular design with logical system separation

STEP 1: ROBOT BASE MOVEMENT INFRASTRUCTURE COMPLETE ✅
- DirectVelocityManager integration with motoron controller
- MovementDirection enum for directional control
- IMU data collection from Oak-D Pro for orientation
- Robot position and orientation state tracking
- Movement safety mechanisms and emergency stop handling
- Enhanced CSV logging with movement metrics

STEP 2: POTENTIAL FIELD NAVIGATION SYSTEM COMPLETE ✅
- PotentialFieldCalculations engine for force mathematics
- NavigationStateMachine for robust state management
- PotentialFieldNavigator for autonomous person following
- Real-time force visualization and navigation state display
- Comprehensive navigation metrics logging
- Safety mechanisms and stuck detection with recovery

NAVIGATION CONTROLS:
- N: Toggle navigation enable/disable (requires movement enabled)
- F: Toggle force vector visualization
- Navigation automatically follows detected person while avoiding LiDAR obstacles

MOVEMENT CONTROLS:
- M: Toggle movement enable/disable (starts disabled for safety)
- Arrow Keys: Manual movement (when movement enabled, navigation disabled)
  - UP: Forward
  - DOWN: Backward  
  - LEFT: Turn left
  - RIGHT: Turn right
- SPACE: Emergency stop

POTENTIAL FIELD FEATURES:
- Attractive fields towards detected person with distance-based scaling
- Repulsive fields from LiDAR obstacles with configurable influence radius
- Force vector summation with magnitude limiting
- Force-to-velocity conversion with directional movement mapping
- Goal reached detection and navigation state management
- Local minimum detection with recovery strategies
- Real-time force visualization (Green=Attractive, Red=Repulsive, Yellow=Total)

NAVIGATION STATES:
- IDLE: Navigation disabled
- SEARCHING: Looking for person target
- NAVIGATING: Actively moving towards person
- GOAL_REACHED: Successfully reached person
- AVOIDING_OBSTACLES: Prioritizing obstacle avoidance
- STUCK: Detected in local minimum
- RECOVERING: Executing recovery strategy
- ERROR: Error condition requiring intervention

SAFETY FEATURES:
- Movement starts disabled - must be manually enabled
- Navigation requires movement to be enabled
- Emergency stop functionality (Space key)
- Motor fault detection and automatic shutdown
- Brake engagement on shutdown or emergency
- Stuck detection and automatic recovery
- Multiple safety layers for obstacle avoidance

Usage: python3 Enhanced-Co-Ordinate-Sensor-Fusion-Movement.py
"""

# Import the main fusion system
from main_fusion_system import EnhancedCoordinateSensorFusionWithMovement

def main():
    """
    ENHANCED COORDINATE SENSOR FUSION WITH ROBOT MOVEMENT AND NAVIGATION - STEP 2 COMPLETE
    
    STEP 1 FEATURES (COMPLETE):
    - Enhanced DirectVelocityManager with motoron controller integration
    - Robot state tracking (position, orientation, velocity)
    - IMU integration from Oak-D Pro for accurate orientation
    - Movement safety mechanisms and emergency stop handling
    - Manual movement controls (Arrow keys + Space to stop)
    - Movement enable/disable toggle (M key)
    - Enhanced CSV logging with movement and IMU metrics
    
    STEP 2 FEATURES (COMPLETE):
    - PotentialFieldCalculations: Core mathematics for attractive/repulsive forces
    - NavigationStateMachine: Robust state management with recovery strategies
    - PotentialFieldNavigator: Autonomous person following with obstacle avoidance
    - Real-time navigation force visualization (F key to toggle)
    - Navigation enable/disable toggle (N key)
    - Comprehensive navigation metrics and logging
    
    AUTONOMOUS NAVIGATION:
    - Attractive forces guide robot towards detected person
    - Repulsive forces prevent collisions with LiDAR obstacles  
    - Real-time force calculation and vector summation
    - Force-to-velocity conversion for smooth movement
    - Navigation state machine handles complex scenarios
    - Stuck detection with automatic recovery strategies
    
    MODULAR ARCHITECTURE:
    - systems/movement/: Movement control and robot state tracking
    - systems/sensors/: Camera detection, LiDAR, and IMU systems
    - systems/control/: Head tracking servo control
    - systems/navigation/: Potential field navigation system
    - main_fusion_system.py: Main coordination and display logic
    
    CSV LOGGING ENHANCED:
    - All existing metrics preserved
    - Added movement metrics (direction, speed, success rate)
    - Added robot state metrics (position, orientation, velocity)
    - Added IMU metrics (orientation, angular velocity, acceleration)
    - Added navigation metrics (state, forces, targets, performance)
    - Complete system performance tracking for analysis
    
    FUNCTIONALITY MAINTAINED:
    - All existing person detection and head tracking
    - All existing LiDAR processing and display
    - All existing calibration system functionality
    - All existing coordinate alignment features
    - All existing anti-oscillation improvements
    
    POTENTIAL FIELD NAVIGATION READY:
    - Robot autonomously navigates to detected person
    - Avoids obstacles using LiDAR data
    - Handles complex navigation scenarios
    - Provides real-time force visualization
    - Comprehensive performance monitoring and logging
    
    This completes Step 2 of the navigation implementation with full autonomous 
    person-following capability using potential field mathematics!
    """
    fusion_system = EnhancedCoordinateSensorFusionWithMovement()
    
    try:
        success = fusion_system.main_loop()
    except Exception as e:
        print(f"System error: {e}")
    finally:
        fusion_system.cleanup()

if __name__ == "__main__":
    main()
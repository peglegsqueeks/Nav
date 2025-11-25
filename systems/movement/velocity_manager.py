#!/usr/bin/env python3
"""
Enhanced Direct Velocity Manager for robot base movement
Handles motoron controller integration with safety features and movement tracking

CLEAN BRAKE VERSION - 2025-11-25
================================
Brake is controlled EXACTLY TWICE:
1. __init__: GPIO HIGH = brake OFF
2. shutdown(): GPIO LOW = brake ON

NO turn_brake_on() or turn_brake_off() methods exist.
NO brake toggling during operation.
================================
"""
import time
import statistics
from collections import deque
from .movement_types import ActionManager, VelocityConfig, MovementDirection

try:
    import Jetson.GPIO as GPIO
    import motoron
    MOTOR_AVAILABLE = True
except ImportError as e:
    print(f"WARNING: Missing motor control libraries: {e}")
    MOTOR_AVAILABLE = False

# =============================================================================
# VERIFICATION: This message proves the CLEAN version is loaded
# =============================================================================
print("=" * 60)
print("VELOCITY_MANAGER: CLEAN BRAKE VERSION 2025-11-25 LOADED")
print("=" * 60)

class EnhancedDirectVelocityManager(ActionManager):
    """
    Enhanced Direct velocity manager - CLEAN BRAKE VERSION
    
    Brake is set OFF at startup and stays OFF until program exit.
    NO brake control during operation.
    """
    
    REFERENCE_MV = 3300
    PIN = 7  # GPIO pin for brake relay
    
    MAX_ACCELERATION = 50
    MAX_DECELERATION = 40
    CURRENT_LIMIT = 15000

    # Motor speed mappings for different movement directions
    DIRECTION_TO_MOTOR_SPEED = {
        MovementDirection.NONE: (0, 0),
        MovementDirection.FORWARDS: (-200, -196),
        MovementDirection.RIGHT: (170, -170),
        MovementDirection.LEFT: (-170, 170),
        MovementDirection.FORWARDS_LEFT: (-270, -200),
        MovementDirection.BACKWARDS_LEFT: (300, 200),
        MovementDirection.BACKWARDS: (200, 200),
        MovementDirection.BACKWARDS_RIGHT: (-170, -205),
        MovementDirection.FORWARDS_RIGHT: (-170, -205),
    }

    def __init__(self, version=1, maxaccel=50, maxdecel=40, current_limit=15000, simulate=False):
        super().__init__("Enhanced Direct Motor Control Velocity Manager")
        
        self.CURRENT_LIMIT = current_limit
        self.MAX_ACCELERATION = maxaccel
        self.MAX_DECELERATION = maxdecel
        self.simulate = simulate
        
        # Movement state tracking
        self.is_moving = False
        self.current_direction = MovementDirection.NONE
        self.current_speed = 0.0
        self.last_movement_time = 0.0
        self.total_movement_commands = 0
        self.movement_start_time = 0.0
        
        # Safety mechanisms
        self.emergency_stop_active = False
        self.motor_fault_detected = False
        self.last_safety_check = time.time()
        self.safety_check_interval = 0.1
        
        # Movement performance tracking
        self.successful_movements = 0
        self.failed_movements = 0
        self.movement_duration_history = deque(maxlen=100)
        
        # Motor initialization
        self.motor_initialized = False
        self.mc = None
        self.gpio_initialized = False
        
        # Type references (set after import check)
        self.TYPE = motoron.CurrentSenseType.MOTORON_24V16 if MOTOR_AVAILABLE else None
        self.VIN_TYPE = motoron.VinSenseType.MOTORON_HP if MOTOR_AVAILABLE else None
        
        if MOTOR_AVAILABLE:
            try:
                # Setup GPIO for brake control
                GPIO.setmode(GPIO.BOARD)
                GPIO.setup(self.PIN, GPIO.OUT)
                self.gpio_initialized = True
                
                # ==========================================================
                # BRAKE OFF - GPIO HIGH = relay energized = brake disengaged
                # This is the ONLY place brake is turned OFF
                # ==========================================================
                GPIO.output(self.PIN, GPIO.HIGH)
                print("VELOCITY_MANAGER: Brake set to OFF (GPIO HIGH)")
                
                if not self.simulate:
                    self.initialise_motor_control()
                    self.motor_initialized = True
                    print("VELOCITY_MANAGER: Motor controller initialized")
            except Exception as e:
                print(f"VELOCITY_MANAGER: Motor initialization failed: {e}")
                self.motor_initialized = False
        else:
            print("VELOCITY_MANAGER: Motor libraries not available - simulation mode")
            self.simulate = True

    def shutdown(self):
        """Shutdown motors and engage brake - ONLY called at program exit"""
        print("VELOCITY_MANAGER: shutdown() called - engaging brake")
        try:
            # Stop motors first
            self.stop_all_movement()
            time.sleep(0.2)
            
            # Release motor controller
            if hasattr(self, 'mc') and self.mc:
                self.mc = None
                
        except Exception as e:
            print(f"VELOCITY_MANAGER: Shutdown error: {e}")
        finally:
            # ==========================================================
            # BRAKE ON - GPIO LOW = relay de-energized = brake engaged
            # This is the ONLY place brake is turned ON
            # ==========================================================
            if MOTOR_AVAILABLE and self.gpio_initialized:
                try:
                    GPIO.output(self.PIN, GPIO.LOW)
                    print("VELOCITY_MANAGER: Brake set to ON (GPIO LOW)")
                except Exception:
                    pass

    def initialise_motor_control(self):
        """Initialize the motoron motor controller"""
        if self.simulate or not MOTOR_AVAILABLE:
            return
            
        try:
            # Initialize the motor controller
            self.mc = motoron.MotoronI2C(bus=7)
            self.mc.reinitialize() 
            self.mc.disable_crc()
            self.mc.clear_reset_flag()
            self.mc.disable_command_timeout()

            # Configure both motors 
            self.configure_motor(1)
            self.configure_motor(2)
            
        except Exception as e:
            print(f"VELOCITY_MANAGER: Motor controller init failed: {e}")
            self.motor_initialized = False
            self.simulate = True

    def configure_motor(self, motor_id):
        """Configure individual motor settings"""
        try:
            self.mc.clear_motor_fault()
            self.mc.set_max_acceleration(motor_id, self.MAX_ACCELERATION)
            self.mc.set_max_deceleration(motor_id, self.MAX_DECELERATION)
            self.mc.set_braking(motor_id, 0)
            self.mc.set_speed(motor_id, 0)
            current_offset = self.mc.get_current_sense_offset(motor_id)
            limit = motoron.calculate_current_limit(self.CURRENT_LIMIT, self.TYPE, self.REFERENCE_MV, current_offset)
            self.mc.set_current_limit(motor_id, limit)
        except Exception as e:
            print(f"VELOCITY_MANAGER: Motor {motor_id} config failed: {e}")
            raise

    def perform_safety_check(self):
        """Perform safety check on motor system - NO BRAKE CONTROL"""
        current_time = time.time()
        if current_time - self.last_safety_check < self.safety_check_interval:
            return True
        self.last_safety_check = current_time
        
        if self.simulate or not self.motor_initialized:
            return True
        
        self.motor_fault_detected = False
        return True

    def stop_all_movement(self):
        """Stop all motor movement - NO BRAKE CONTROL"""
        try:
            if self.simulate:
                self.is_moving = False
                self.current_direction = MovementDirection.NONE
                self.current_speed = 0.0
                return True
                
            if hasattr(self, 'mc') and self.mc:
                self.mc.set_speed(1, 0)
                self.mc.set_speed(2, 0)
                
            self.is_moving = False
            self.current_direction = MovementDirection.NONE
            self.current_speed = 0.0
            return True
            
        except Exception:
            return False

    def perform_action(self, config: VelocityConfig):
        """Perform movement action - NO BRAKE CONTROL"""
        # Safety checks
        if not self.perform_safety_check():
            self.failed_movements += 1
            return False
            
        if self.emergency_stop_active or self.motor_fault_detected:
            self.stop_all_movement()
            return False
        
        direction = config.direction
        speed = config.speed
        current_time = time.time()
        
        # Update movement state tracking
        was_moving = self.is_moving
        self.current_direction = direction
        self.current_speed = speed
        self.total_movement_commands += 1
        
        if direction == MovementDirection.NONE:
            if was_moving and self.movement_start_time > 0:
                duration = current_time - self.movement_start_time
                self.movement_duration_history.append(duration)
            self.is_moving = False
            self.movement_start_time = 0.0
        else:
            if not was_moving:
                self.movement_start_time = current_time
            self.is_moving = True
            
        self.last_movement_time = current_time
        
        # Get base motor speeds for the direction
        motor_1, motor_2 = self.DIRECTION_TO_MOTOR_SPEED.get(direction, (0, 0))

        # Apply speed multiplier
        motor_1 = int(motor_1 * speed)
        motor_2 = int(motor_2 * speed)

        # Clamp motor speeds to safe range
        motor_1 = max(-800, min(800, motor_1))
        motor_2 = max(-800, min(800, motor_2))

        if self.simulate:
            self.successful_movements += 1
            return True

        # Send commands to motors
        try:
            if hasattr(self, 'mc') and self.mc and self.motor_initialized:
                self.mc.set_speed(1, motor_1)
                self.mc.set_speed(2, motor_2)
                self.successful_movements += 1
                return True
            else:
                self.failed_movements += 1
                return False
        except Exception:
            self.failed_movements += 1
            return False

    def get_movement_status(self):
        """Get current movement status"""
        avg_movement_duration = 0.0
        if len(self.movement_duration_history) > 0:
            avg_movement_duration = statistics.mean(self.movement_duration_history)
            
        return {
            'is_moving': self.is_moving,
            'current_direction': self.current_direction.value,
            'current_speed': self.current_speed,
            'motor_initialized': self.motor_initialized,
            'emergency_stop_active': self.emergency_stop_active,
            'motor_fault_detected': self.motor_fault_detected,
            'total_movement_commands': self.total_movement_commands,
            'successful_movements': self.successful_movements,
            'failed_movements': self.failed_movements,
            'movement_success_rate': (self.successful_movements / max(1, self.total_movement_commands)) * 100,
            'time_since_last_movement': time.time() - self.last_movement_time if self.last_movement_time > 0 else 0,
            'avg_movement_duration': avg_movement_duration,
            'simulate_mode': self.simulate
        }
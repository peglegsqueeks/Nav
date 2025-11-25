#!/usr/bin/env python3
"""
IMU System for Robot Orientation Tracking
Provides roll, pitch, yaw data for navigation and coordinate transforms using Oak-D Pro built-in IMU
"""
import math
import time
import statistics
from collections import deque

try:
    import depthai as dai
    DEPTHAI_AVAILABLE = True
except ImportError:
    DEPTHAI_AVAILABLE = False

class RobotIMUSystem:
    """
    IMU system for tracking robot orientation using Oak-D Pro built-in IMU
    Provides roll, pitch, yaw data for navigation and coordinate transforms
    """
    
    def __init__(self):
        self.imu_queue = None
        self.device = None
        self.imu_initialized = False
        self.imu_error_message = ""
        
        # IMU data
        self.current_orientation = {
            'roll': 0.0,
            'pitch': 0.0, 
            'yaw': 0.0,
            'timestamp': 0.0
        }
        
        self.current_angular_velocity = {
            'x': 0.0,
            'y': 0.0,
            'z': 0.0,
            'timestamp': 0.0
        }
        
        self.current_acceleration = {
            'x': 0.0,
            'y': 0.0,
            'z': 0.0,
            'timestamp': 0.0
        }
        
        # IMU performance tracking
        self.imu_update_count = 0
        self.imu_error_count = 0
        self.last_imu_update = time.time()
        
        # Orientation smoothing
        self.orientation_history = deque(maxlen=10)
        self.smoothed_yaw = 0.0

    def add_imu_to_pipeline(self, pipeline):
        """Add IMU node to existing DepthAI pipeline"""
        if not DEPTHAI_AVAILABLE:
            return False
            
        try:
            # Create IMU node
            imu = pipeline.create(dai.node.IMU)
            
            # Configure IMU sensors
            imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, 500)  # 500Hz
            imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, 400)     # 400Hz
            imu.enableIMUSensor(dai.IMUSensor.ROTATION_VECTOR, 400)   # 400Hz for orientation
            
            # Set batch report threshold
            imu.setBatchReportThreshold(1)
            imu.setMaxBatchReports(10)
            
            # Create output
            imu_out = pipeline.create(dai.node.XLinkOut)
            imu_out.setStreamName("imu")
            imu.out.link(imu_out.input)
            
            return True
            
        except Exception as e:
            self.imu_error_message = f"IMU pipeline creation failed: {e}"
            return False

    def initialize_imu_queue(self, device):
        """Initialize IMU queue from device"""
        try:
            self.device = device
            self.imu_queue = device.getOutputQueue(name="imu", maxSize=50, blocking=False)
            self.imu_initialized = True
            return True
            
        except Exception as e:
            self.imu_error_message = f"IMU queue initialization failed: {e}"
            return False

    def update_imu_data(self):
        """Update IMU data from queue"""
        if not (self.imu_initialized and self.imu_queue):
            return False
            
        try:
            imu_data = self.imu_queue.tryGet()
            if imu_data is not None:
                imu_packets = imu_data.packets
                
                for packet in imu_packets:
                    if packet.acceleroMeter is not None:
                        accel = packet.acceleroMeter
                        self.current_acceleration = {
                            'x': accel.x,
                            'y': accel.y,
                            'z': accel.z,
                            'timestamp': time.time()
                        }
                    
                    if packet.gyroscope is not None:
                        gyro = packet.gyroscope
                        self.current_angular_velocity = {
                            'x': gyro.x,
                            'y': gyro.y,
                            'z': gyro.z,
                            'timestamp': time.time()
                        }
                    
                    if packet.rotationVector is not None:
                        rot = packet.rotationVector
                        # Convert quaternion to Euler angles
                        roll, pitch, yaw = self.quaternion_to_euler(rot.i, rot.j, rot.k, rot.real)
                        
                        self.current_orientation = {
                            'roll': math.degrees(roll),
                            'pitch': math.degrees(pitch),
                            'yaw': math.degrees(yaw),
                            'timestamp': time.time()
                        }
                        
                        # Apply orientation smoothing
                        self.orientation_history.append(yaw)
                        if len(self.orientation_history) > 1:
                            self.smoothed_yaw = statistics.mean(self.orientation_history)
                        else:
                            self.smoothed_yaw = yaw
                
                self.imu_update_count += 1
                self.last_imu_update = time.time()
                return True
                
        except Exception as e:
            self.imu_error_count += 1
            return False
            
        return False

    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to Euler angles (roll, pitch, yaw)"""
        try:
            # Roll (x-axis rotation)
            sinr_cosp = 2 * (w * x + y * z)
            cosr_cosp = 1 - 2 * (x * x + y * y)
            roll = math.atan2(sinr_cosp, cosr_cosp)

            # Pitch (y-axis rotation)
            sinp = 2 * (w * y - z * x)
            if abs(sinp) >= 1:
                pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
            else:
                pitch = math.asin(sinp)

            # Yaw (z-axis rotation)
            siny_cosp = 2 * (w * z + x * y)
            cosy_cosp = 1 - 2 * (y * y + z * z)
            yaw = math.atan2(siny_cosp, cosy_cosp)

            return roll, pitch, yaw
            
        except Exception:
            return 0.0, 0.0, 0.0

    def get_imu_status(self):
        """Get current IMU status for logging and display"""
        current_time = time.time()
        imu_update_rate = 0.0
        
        if self.imu_update_count > 0 and self.last_imu_update > 0:
            time_elapsed = current_time - (self.last_imu_update - (self.imu_update_count * 0.1))  # Rough estimate
            if time_elapsed > 0:
                imu_update_rate = self.imu_update_count / time_elapsed
        
        return {
            'imu_initialized': self.imu_initialized,
            'roll': self.current_orientation['roll'],
            'pitch': self.current_orientation['pitch'],
            'yaw': self.current_orientation['yaw'],
            'smoothed_yaw': math.degrees(self.smoothed_yaw) if self.orientation_history else 0.0,
            'angular_vel_x': self.current_angular_velocity['x'],
            'angular_vel_y': self.current_angular_velocity['y'],
            'angular_vel_z': self.current_angular_velocity['z'],
            'accel_x': self.current_acceleration['x'],
            'accel_y': self.current_acceleration['y'],
            'accel_z': self.current_acceleration['z'],
            'imu_update_count': self.imu_update_count,
            'imu_error_count': self.imu_error_count,
            'imu_update_rate': imu_update_rate,
            'time_since_last_imu_update': current_time - self.last_imu_update if self.last_imu_update > 0 else 0
        }
#!/usr/bin/env python3
"""
Optimized LiDAR System
Handles RPLiDAR data acquisition and obstacle detection
"""
import time
import threading
import queue

try:
    from pyrplidar import PyRPlidar
    LIDAR_AVAILABLE = True
except ImportError as e:
    print(f"WARNING: Missing LiDAR library: {e}")
    LIDAR_AVAILABLE = False

class OptimizedLidarSystem:
    """Optimized LiDAR system for obstacle detection and mapping"""
    
    def __init__(self, port='/dev/ttyUSB0', baudrate=256000):
        self.port = port
        self.baudrate = baudrate
        self.lidar = None
        self.scan_queue = queue.Queue(maxsize=2)
        self.latest_obstacles = []
        self.data_lock = threading.Lock()
        self.obstacle_confidence = {}
        self.angle_resolution = 1.0
        self.distance_resolution = 50
        self.confidence_threshold = 0.15
        self.confidence_increment = 0.25
        self.confidence_decay = 0.05
        self.max_confidence = 1.0
        self.scan_rate = 0
        self.last_scan_time = time.time()
        self.scan_count = 0
        self.running = False
        self.shutdown_flag = [False]
        self.threads = []

    def data_acquisition_thread(self):
        """LiDAR data acquisition thread"""
        if not LIDAR_AVAILABLE:
            return
            
        from pyrplidar import PyRPlidar
        
        try:
            self.lidar = PyRPlidar()
            self.lidar.connect(port=self.port, baudrate=self.baudrate, timeout=2.0)
            self.lidar.stop()
            time.sleep(1.0)
            self.lidar.set_motor_pwm(600)
            time.sleep(3.0)
            
            # Try express mode first, fallback to force scan
            try:
                scan_generator = self.lidar.start_scan_express(4)()
            except:
                scan_generator = self.lidar.force_scan()()
            
            scan_buffer = []
            last_angle = None
            consecutive_failures = 0
            
            for measurement in scan_generator:
                if not self.running or self.shutdown_flag[0] or consecutive_failures > 50:
                    break
                    
                if measurement:
                    consecutive_failures = 0
                    try:
                        quality = getattr(measurement, 'quality', 0)
                        angle = getattr(measurement, 'angle', 0)
                        distance = getattr(measurement, 'distance', 0)
                        
                        if quality > 8 and 200 < distance < 6000:
                            scan_buffer.append((quality, angle, distance))
                            if (last_angle is not None and angle < last_angle and len(scan_buffer) > 100):
                                self.process_scan_data(scan_buffer)
                                scan_buffer = []
                            last_angle = angle
                    except Exception:
                        continue
                else:
                    consecutive_failures += 1
                    time.sleep(0.1)
                    
        except Exception:
            pass
        finally:
            if self.lidar:
                try:
                    self.lidar.stop()
                    time.sleep(0.5)
                    self.lidar.set_motor_pwm(0)
                    time.sleep(0.5)
                    self.lidar.disconnect()
                except:
                    pass

    def process_scan_data(self, scan_data):
        """Process scan data and update obstacle confidence"""
        seen_obstacles = set()
        
        for quality, angle, distance in scan_data:
            if quality > 8 and 200 < distance < 6000:
                qa = round(angle / self.angle_resolution) * self.angle_resolution
                qd = round(distance / self.distance_resolution) * self.distance_resolution
                key = (qa, qd)
                seen_obstacles.add(key)
                cur = self.obstacle_confidence.get(key, 0)
                self.obstacle_confidence[key] = min(self.max_confidence, cur + self.confidence_increment)
        
        # Decay unseen obstacles
        to_remove = []
        for key, conf in list(self.obstacle_confidence.items()):
            if key not in seen_obstacles:
                new_conf = conf - self.confidence_decay
                if new_conf <= 0:
                    to_remove.append(key)
                else:
                    self.obstacle_confidence[key] = new_conf
        
        for k in to_remove:
            del self.obstacle_confidence[k]
        
        # Update obstacle list
        stable_obstacles = [(a, d) for (a, d), c in self.obstacle_confidence.items() if c >= self.confidence_threshold]
        
        if stable_obstacles:
            with self.data_lock:
                self.latest_obstacles = stable_obstacles
            try:
                self.scan_queue.put_nowait(stable_obstacles)
            except queue.Full:
                try:
                    self.scan_queue.get_nowait()
                    self.scan_queue.put_nowait(stable_obstacles)
                except queue.Empty:
                    pass

    def get_display_obstacles(self):
        """Get obstacles for display"""
        with self.data_lock:
            return self.latest_obstacles.copy() if self.latest_obstacles else []

    def start(self):
        """Start LiDAR system"""
        if not LIDAR_AVAILABLE:
            return False
        try:
            self.running = True
            self.shutdown_flag[0] = False
            t = threading.Thread(target=self.data_acquisition_thread, daemon=True)
            t.start()
            self.threads.append(t)
            return True
        except Exception:
            return False

    def stop(self):
        """Stop LiDAR system"""
        try:
            self.running = False
            self.shutdown_flag[0] = True
            for t in self.threads:
                if t.is_alive():
                    t.join(timeout=2.0)
        except Exception:
            pass
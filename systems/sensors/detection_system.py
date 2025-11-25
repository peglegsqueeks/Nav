#!/usr/bin/env python3
"""
Enhanced RGB + Reuse Logic Camera Detection System
ENHANCED RGB + REUSE LOGIC detection system with maximum field of view and IMU integration
"""
import os
import time
import numpy as np
import pygame
from collections import deque
from typing import Optional, Dict, Any
from .imu_system import RobotIMUSystem

try:
    import depthai as dai
    import cv2
    DEPTHAI_AVAILABLE = True
except ImportError as e:
    print(f"WARNING: Missing DepthAI libraries: {e}")
    DEPTHAI_AVAILABLE = False

class EnhancedRgbReuseDetectionSystem:
    """ENHANCED RGB + REUSE LOGIC detection system with maximum field of view and IMU integration"""
    
    def __init__(self):
        self.device = None
        self.pipeline = None
        self.detection_queue = None
        self.preview_queue = None
        self.has_detection = False
        self.camera_initialized = False
        self.camera_error_message = ""
        
        # ENHANCED: RGB camera settings
        self.rgb_preview_width = 300
        self.rgb_preview_height = 300
        self.target_fps = 25
        
        # MAINTAINED: Detection settings - confidence only
        self.confidence_threshold = 0.65
        self.depth_lower_threshold = 1
        self.depth_upper_threshold = 50000
        self.bbox_scale_factor = 0.5
        self.stereo_confidence = 150
        
        # ENHANCED: Reuse logic parameters
        self.reuse_window_ms = 300.0  # Brief gap reuse window
        self.last_good: Optional[Dict[str, Any]] = None
        self.last_good_t: float = 0.0
        
        # Frame processing
        self.detection_skip_frames = 1
        self.frame_counter = 0
        
        # Detection history
        self.detection_history = deque(maxlen=3)
        
        # Depth smoothing
        self.z_depth_smoother = deque(maxlen=3)
        self.confidence_weights = deque(maxlen=3)
        self.smoothed_z_depth = 0
        self.depth_trust_threshold = 0.7
        
        # OPTIMIZED: Display settings for maximum FOV
        self.camera_display_size = (600, 450)
        self.camera_surface = None
        self.show_camera_debug = True
        
        # ENHANCED: Mono camera settings for stereo depth
        self.mono_width = 1280  # 800P resolution
        self.mono_height = 800
        
        # ENHANCED: IMU system integration
        self.imu_system = RobotIMUSystem()
    
    def check_camera_connection(self):
        """Check camera connection"""
        try:
            devices = dai.Device.getAllAvailableDevices()
            return len(devices) > 0, f"Found {len(devices)} device(s)"
        except Exception as e:
            return False, f"Device detection error: {str(e)}"
    
    def create_enhanced_rgb_fov_pipeline_with_imu(self):
        """ENHANCED: Create pipeline with RGB camera, maximum FOV settings, and IMU"""
        try:
            pipeline = dai.Pipeline()
            
            local_blob_path = "./mobilenet-ssd_openvino_2021.4_5shave.blob"
            if not os.path.exists(local_blob_path):
                raise Exception(f"Blob file not found: {local_blob_path}")
            
            # ENHANCED: RGB ColorCamera for NN input (CAM_A)
            rgb_cam = pipeline.create(dai.node.ColorCamera)
            rgb_cam.setBoardSocket(dai.CameraBoardSocket.CAM_A)
            rgb_cam.setPreviewSize(self.rgb_preview_width, self.rgb_preview_height)
            rgb_cam.setInterleaved(False)
            rgb_cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
            rgb_cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
            rgb_cam.setFps(self.target_fps)
            # ENHANCED: Maximum FOV - no aspect preservation
            try:
                rgb_cam.setPreviewKeepAspectRatio(False)
            except Exception:
                pass
            
            # MAINTAINED: Mono cameras for stereo depth
            mono_left = pipeline.create(dai.node.MonoCamera)
            mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)  
            mono_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
            mono_left.setFps(self.target_fps)
            
            mono_right = pipeline.create(dai.node.MonoCamera)
            mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)
            mono_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
            mono_right.setFps(self.target_fps)
            
            # MAINTAINED: Display ImageManip using mono camera for compatibility
            manip_display = pipeline.create(dai.node.ImageManip)
            manip_display.initialConfig.setResize(self.camera_display_size[0], self.camera_display_size[1])
            manip_display.initialConfig.setKeepAspectRatio(False)  # OPTIMIZED: False for max FOV display
            manip_display.initialConfig.setFrameType(dai.ImgFrame.Type.GRAY8)
            mono_right.out.link(manip_display.inputImage)
            
            # ENHANCED: Stereo depth aligned to RGB camera (CAM_A)
            depth = pipeline.create(dai.node.StereoDepth)
            depth.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
            depth.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_5x5)
            depth.initialConfig.setConfidenceThreshold(self.stereo_confidence)
            depth.setLeftRightCheck(True)
            depth.setSubpixel(True)
            depth.setRectifyEdgeFillColor(0)
            depth.setDepthAlign(dai.CameraBoardSocket.CAM_A)  # ENHANCED: Align to RGB camera
            depth.setInputResolution(self.mono_width, self.mono_height)  # ENHANCED: Set input resolution
            
            mono_left.out.link(depth.left)
            mono_right.out.link(depth.right)
            
            # ENHANCED: Detection network using RGB input directly
            detection_nn = pipeline.create(dai.node.MobileNetSpatialDetectionNetwork)
            detection_nn.setConfidenceThreshold(self.confidence_threshold)
            detection_nn.setBlobPath(local_blob_path)
            detection_nn.setBoundingBoxScaleFactor(self.bbox_scale_factor)
            detection_nn.setDepthLowerThreshold(self.depth_lower_threshold)
            detection_nn.setDepthUpperThreshold(self.depth_upper_threshold)
            
            # ENHANCED: Direct RGB preview to NN (no intermediate ImageManip)
            rgb_cam.preview.link(detection_nn.input)
            depth.depth.link(detection_nn.inputDepth)
            
            # ENHANCED: Add IMU to pipeline
            self.imu_system.add_imu_to_pipeline(pipeline)
            
            # Outputs
            detection_out = pipeline.create(dai.node.XLinkOut)
            detection_out.setStreamName("detections")
            detection_nn.out.link(detection_out.input)
            
            preview_out = pipeline.create(dai.node.XLinkOut)
            preview_out.setStreamName("preview")
            manip_display.out.link(preview_out.input)
            
            return pipeline
        except Exception as e:
            return None
    
    def initialize(self):
        """Initialize camera system with enhanced RGB + FOV settings and IMU"""
        if not DEPTHAI_AVAILABLE:
            return False
            
        try:
            connected, message = self.check_camera_connection()
            if not connected:
                self.camera_error_message = message
                return False
            
            self.pipeline = self.create_enhanced_rgb_fov_pipeline_with_imu()
            if not self.pipeline:
                self.camera_error_message = "Enhanced RGB FOV pipeline with IMU creation failed"
                return False
            
            try:
                self.device = dai.Device(self.pipeline)
            except Exception as e:
                self.camera_error_message = f"Device connection failed: {str(e)}"
                return False
            
            # ENHANCED: IR settings using intensity APIs
            try:
                if hasattr(self.device, 'setLogLevel'):
                    self.device.setLogLevel(dai.LogLevel.WARN)
                # Use intensity APIs instead of deprecated brightness
                try:
                    self.device.setIrLaserDotProjectorIntensity(0.01)
                except:
                    self.device.setIrLaserDotProjectorIntensity(0)
                try:
                    self.device.setIrFloodLightIntensity(0.01)
                except:
                    self.device.setIrFloodLightIntensity(0)
            except Exception:
                pass
            
            # Get queues
            try:
                self.detection_queue = self.device.getOutputQueue("detections", maxSize=1, blocking=False)
                self.has_detection = True
            except Exception:
                self.detection_queue = None
                self.has_detection = False
            
            try:
                self.preview_queue = self.device.getOutputQueue("preview", maxSize=1, blocking=False)
            except Exception:
                self.preview_queue = None
            
            # ENHANCED: Initialize IMU queue
            self.imu_system.initialize_imu_queue(self.device)
            
            # Camera testing
            if self.preview_queue:
                for i in range(30):
                    test_frame = self.preview_queue.tryGet()
                    if test_frame:
                        self.camera_initialized = True
                        break
                    time.sleep(0.05)
            
            time.sleep(1.0)
            return True
            
        except Exception as e:
            self.camera_error_message = f"Camera initialization error: {str(e)}"
            return False
    
    def update_camera_frame(self):
        """Update camera display"""
        if not self.preview_queue or not self.camera_initialized:
            return
        
        try:
            frame_data = self.preview_queue.tryGet()
            if frame_data is not None:
                frame = frame_data.getFrame()
                if frame is not None and frame.size > 0:
                    if frame.shape[0] != self.camera_display_size[1] or frame.shape[1] != self.camera_display_size[0]:
                        frame = cv2.resize(frame, self.camera_display_size)
                    
                    if len(frame.shape) == 2:
                        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2RGB)
                    else:
                        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    
                    rgb_frame_transposed = np.transpose(rgb_frame, (1, 0, 2))
                    self.camera_surface = pygame.surfarray.make_surface(rgb_frame_transposed)
        except Exception:
            if self.show_camera_debug:
                self.show_camera_debug = False
    
    def update_imu_data(self):
        """Update IMU data from Oak-D Pro"""
        return self.imu_system.update_imu_data()
    
    def get_imu_status(self):
        """Get current IMU status"""
        return self.imu_system.get_imu_status()
    
    def _reuse_if_recent(self, queue_empty: bool, raw_depth_valid: Optional[bool] = None) -> Optional[Dict[str, Any]]:
        """ENHANCED: Reuse last good detection if within time window"""
        now = time.time()
        age_ms = (now - self.last_good_t) * 1000.0
        if self.last_good and age_ms <= self.reuse_window_ms:
            reused = dict(self.last_good)
            meta = dict(reused.get("meta", {}))
            if raw_depth_valid is None:
                raw_ok = int(meta.get("raw_depth_valid", 1))
            else:
                raw_ok = 1 if raw_depth_valid else 0
            meta.update({
                "reused_last_good": 1,
                "last_good_age_ms": age_ms,
                "queue_empty": int(queue_empty),
                "raw_depth_valid": raw_ok,
                "nn_input": "RGB",
            })
            reused["meta"] = meta
            return reused
        return None

    def _record_last_good(self, fresh: Dict[str, Any]):
        """ENHANCED: Record last good detection for reuse"""
        self.last_good = dict(fresh)
        self.last_good_t = time.time()
    
    def simple_z_depth_smoothing(self, raw_z_depth, confidence):
        """Simple depth smoothing"""
        try:
            self.z_depth_smoother.append(raw_z_depth)
            self.confidence_weights.append(confidence)
            
            if len(self.z_depth_smoother) < 2:
                self.smoothed_z_depth = raw_z_depth
                return raw_z_depth
            
            total_weight = 0
            weighted_sum = 0
            
            for i, (z_val, conf) in enumerate(zip(self.z_depth_smoother, self.confidence_weights)):
                recency_weight = (i + 1) / len(self.z_depth_smoother)
                confidence_weight = max(0.1, conf)
                combined_weight = recency_weight * confidence_weight
                
                weighted_sum += z_val * combined_weight
                total_weight += combined_weight
            
            smoothed = weighted_sum / total_weight if total_weight > 0 else raw_z_depth
            
            if confidence < self.depth_trust_threshold and len(self.z_depth_smoother) > 1:
                prev_z = self.z_depth_smoother[-2]
                max_jump = 300
                if abs(smoothed - prev_z) > max_jump:
                    blend_factor = confidence / self.depth_trust_threshold
                    smoothed = prev_z + (smoothed - prev_z) * blend_factor
            
            self.smoothed_z_depth = smoothed
            return smoothed
        except Exception:
            return raw_z_depth
    
    def get_detection(self):
        """ENHANCED: Get person detection with reuse logic and confidence validation"""
        self.frame_counter += 1
        
        if not (self.has_detection and self.detection_queue and self.camera_initialized):
            self.detection_history.append(False)
            return None
        
        try:
            # Try to get fresh detection
            detections = self.detection_queue.tryGet()
            
            det_info = None
            if detections is None:
                # No fresh detection - try reuse
                det_info = self._reuse_if_recent(queue_empty=True)
            else:
                # Got fresh detections - look for person
                person_detections = [det for det in detections.detections if det.label == 15]
                if not person_detections:
                    # No person in fresh detection - try reuse
                    det_info = self._reuse_if_recent(queue_empty=False)
                else:
                    # Found person - validate and use
                    closest_person = min(person_detections, key=lambda p: p.spatialCoordinates.z)
                    
                    x_camera = closest_person.spatialCoordinates.x
                    y_camera = closest_person.spatialCoordinates.y
                    raw_z_depth = closest_person.spatialCoordinates.z
                    confidence = closest_person.confidence
                    
                    # Check if detection meets criteria
                    z_ok = (self.depth_lower_threshold <= raw_z_depth <= self.depth_upper_threshold)
                    if confidence >= self.confidence_threshold and z_ok:
                        # Good fresh detection
                        smoothed_z_depth = self.simple_z_depth_smoothing(raw_z_depth, confidence)
                        
                        bbox_xmin = closest_person.xmin
                        bbox_xmax = closest_person.xmax
                        bbox_ymin = closest_person.ymin
                        bbox_ymax = closest_person.ymax
                        
                        x_midpoint_normalized = (bbox_xmin + bbox_xmax) / 2.0
                        y_midpoint_normalized = (bbox_ymin + bbox_ymax) / 2.0
                        
                        det_info = {
                            'x_camera': x_camera,
                            'y_camera': y_camera,
                            'z_camera': smoothed_z_depth,
                            'raw_z_depth': raw_z_depth,
                            'confidence': confidence,
                            'bbox_center': {
                                'x_normalized': x_midpoint_normalized,
                                'y_normalized': y_midpoint_normalized,
                            },
                            'bounding_box': {
                                'xmin': bbox_xmin,
                                'ymin': bbox_ymin,
                                'xmax': bbox_xmax,
                                'ymax': bbox_ymax
                            },
                            'meta': {
                                "reused_last_good": 0,
                                "last_good_age_ms": 0.0,
                                "queue_empty": 0,
                                "raw_depth_valid": 1,
                                "nn_input": "RGB",
                            }
                        }
                        # Record for future reuse
                        self._record_last_good(det_info)
                    else:
                        # Fresh detection doesn't meet criteria - try reuse
                        det_info = self._reuse_if_recent(queue_empty=False, raw_depth_valid=z_ok)
            
            # Convert reused detection back to expected format if we got one
            if det_info and 'meta' in det_info:
                # This is a reused detection - convert format
                if det_info['meta'].get('reused_last_good', 0) == 1:
                    # Already in correct format from reuse
                    self.detection_history.append(True)
                    return det_info
            
            if det_info:
                self.detection_history.append(True)
                return det_info
            else:
                self.detection_history.append(False)
                return None
                
        except Exception:
            self.detection_history.append(False)
            return None
    
    def shutdown(self):
        """Shutdown camera system"""
        try:
            if self.device:
                self.device.close()
        except Exception:
            pass
#!/usr/bin/env python3
"""
Multi-Camera Map Data Extractor for BEVFormer Training

This script extracts synchronized multi-camera images and ego poses
from ROS2 bags for training BEVFormer-based map reconstruction models.

Features:
- Support for multiple ROSBags with single map in one extraction run
- Append mode to add new bags to existing dataset
- Separate storage: trajectories (per bag) and maps (shared, hash-based)
- Auto-detect cameras from ROSBag topics
- Synchronized data collection across cameras and ego pose
- Extract camera calibration (intrinsics/extrinsics)

Directory Structure:
    /output_dir/
    ├── maps/
    │   └── {map_hash}/
    │       ├── map_info.json
    │       └── resampled_lanelets.pkl
    ├── trajectories/
    │   └── {bag_name}/
    │       ├── trajectory.json
    │       └── images/
    └── index.json
"""

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
import cv2
import os
import sys
import numpy as np
import yaml
import json
from scipy.spatial.transform import Rotation as R
from typing import Dict, List, Any, Optional, Tuple
from pathlib import Path
from collections import defaultdict
import argparse
import glob

# Import map storage module
from map_preprocessor.map_storage import (
    get_or_create_map,
    load_map_info,
    update_index_map,
    update_index_trajectory,
    trajectory_exists,
    sanitize_bag_name,
    load_index,
)


def read_camera_config(camera_config_file: str) -> Dict[str, Any]:
    """Read camera configuration from YAML file"""
    with open(camera_config_file, 'r') as f:
        camera_config = yaml.safe_load(f)
    
    # Extract intrinsics
    K = np.array(camera_config['camera_matrix']['data']).reshape(3, 3)
    D = np.array(camera_config['distortion_coefficients']['data'])
    P = np.array(camera_config['projection_matrix']['data']).reshape(3, 4)
    
    return {
        'K': K,
        'D': D,
        'P': P,
        'width': camera_config.get('width', 1920),
        'height': camera_config.get('height', 1080)
    }


def calib_dict_to_matrix(calib_dict: Dict[str, float]) -> np.ndarray:
    """Convert calibration dictionary to 4x4 transformation matrix"""
    x = calib_dict['x']
    y = calib_dict['y']
    z = calib_dict['z']
    roll = calib_dict['roll']
    pitch = calib_dict['pitch']
    yaw = calib_dict['yaw']
    
    T = np.eye(4)
    T[0:3, 3] = np.array([x, y, z])
    T[0:3, 0:3] = R.from_euler('xyz', [roll, pitch, yaw]).as_matrix()
    return T


def pose_to_matrix(pose_msg: Odometry) -> np.ndarray:
    """Convert ROS Odometry message to 4x4 transformation matrix"""
    T = np.eye(4)
    pos = np.array([
        pose_msg.pose.pose.position.x,
        pose_msg.pose.pose.position.y,
        pose_msg.pose.pose.position.z
    ])
    T[0:3, 3] = pos
    
    quat = np.array([
        pose_msg.pose.pose.orientation.x,
        pose_msg.pose.pose.orientation.y,
        pose_msg.pose.pose.orientation.z,
        pose_msg.pose.pose.orientation.w
    ])
    T[0:3, 0:3] = R.from_quat(quat).as_matrix()
    return T


def extract_heading_from_transform(transform: np.ndarray) -> float:
    """Extract heading angle (yaw) from 4x4 transformation matrix"""
    rot_matrix = transform[0:3, 0:3]
    r = R.from_matrix(rot_matrix)
    euler = r.as_euler('xyz')
    return euler[2]  # Return yaw


class MultiCameraMapExtractor:
    """Extract multi-camera images and ego poses from ROS2 bags"""
    
    def __init__(
        self,
        bag_paths: List[str],
        map_path: str,
        sensor_config_dir: str,
        output_dir: str,
        camera_info_dir: str,
        bev_range: float = 80.0,  # meters
        lane_point_number: int = 20,
        downsample_factor: int = 2,
        camera_cycle_time: float = 0.1,
        max_camera_offset: float = 0.08,
        append_mode: bool = False,
    ):
        self.bag_paths = bag_paths
        self.map_path = map_path
        self.sensor_config_dir = sensor_config_dir
        self.output_dir = output_dir
        self.camera_info_dir = camera_info_dir
        self.bev_range = bev_range
        self.lane_point_number = lane_point_number
        self.downsample_factor = downsample_factor
        self.camera_cycle_time = camera_cycle_time
        self.max_camera_offset = max_camera_offset
        self.append_mode = append_mode
        
        # Create output directories
        os.makedirs(output_dir, exist_ok=True)
        os.makedirs(os.path.join(output_dir, 'maps'), exist_ok=True)
        os.makedirs(os.path.join(output_dir, 'trajectories'), exist_ok=True)
        
        # Initialize
        self.cv_bridge = CvBridge()
        self.camera_configs = {}
        self.camera_names = []
        
        # Map hash (set during run)
        self.map_hash = None
    
    def _reset_for_new_bag(self):
        """Reset state for processing a new bag"""
        self.image_buffers = defaultdict(list)
        self.pose_buffer = []
        self.last_camera_times = {}
        self.last_processed_cycle = -1
        self.debug_rejection_counts = {
            'time_spread': 0,
            'no_pose': 0,
            'pose_too_far': 0,
            'already_processed': 0
        }
        self.frame_id = 0
        self.extracted_frames = []
    
    def discover_cameras(self, bag_path: str):
        """Auto-discover cameras from bag topics (only those with data)"""
        print(f"Discovering cameras from bag {bag_path}...")
        storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3")
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'
        )
        
        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, converter_options)
        
        camera_message_counts = defaultdict(int)
        sample_count = 0
        max_samples = 2000
        
        print("Sampling bag to detect active cameras...")
        while reader.has_next() and sample_count < max_samples:
            topic, data, timestamp = reader.read_next()
            if '/sensing/camera/' in topic and 'image_raw/compressed' in topic:
                camera_message_counts[topic] += 1
            sample_count += 1
        
        self.camera_names = []
        skipped_cameras = []
        for topic, count in camera_message_counts.items():
            parts = topic.split('/')
            camera_idx = parts.index('camera')
            if camera_idx + 1 < len(parts):
                camera_name = parts[camera_idx + 1]
                
                if count > 0:
                    self.camera_names.append(camera_name)
                    print(f"  {camera_name}: {count} messages in sample")
                else:
                    skipped_cameras.append(camera_name)
        
        print(f"Discovered {len(self.camera_names)} active cameras: {self.camera_names}")
        if skipped_cameras:
            print(f"Skipped {len(skipped_cameras)} cameras with no data: {skipped_cameras}")
        
        return self.camera_names
    
    def load_camera_calibrations(self):
        """Load camera calibrations from sensor config directory"""
        print("Loading camera calibrations...")
        
        sensor_calib_file = os.path.join(self.sensor_config_dir, "sensors_calibration.yaml")
        sensor_kit_calib_file = os.path.join(self.sensor_config_dir, "sensor_kit_calibration.yaml")
        
        with open(sensor_calib_file, 'r') as f:
            sensor_calibration = yaml.safe_load(f)
        with open(sensor_kit_calib_file, 'r') as f:
            sensor_kit_calibration = yaml.safe_load(f)
        
        sensor_kit_base = calib_dict_to_matrix(
            sensor_calibration['base_link']['sensor_kit_base_link']
        )
        
        for camera_name in self.camera_names:
            camera_info_file = os.path.join(self.camera_info_dir, f'{camera_name}_info.yaml')
            if not os.path.exists(camera_info_file):
                print(f"Warning: Camera info file not found for {camera_name}")
                continue
            
            camera_config = read_camera_config(camera_info_file)
            
            camera_link_to_sensorkit = calib_dict_to_matrix(
                sensor_kit_calibration['sensor_kit_base_link'][f'{camera_name}/camera_link']
            )
            
            camera_optical_to_link = np.eye(4)
            camera_optical_to_link[0:3, 0:3] = np.array([
                [0, 0, 1],
                [-1, 0, 0],
                [0, -1, 0]
            ])
            
            camera_to_base = sensor_kit_base @ camera_link_to_sensorkit @ camera_optical_to_link
            
            self.camera_configs[camera_name] = {
                'intrinsics': {
                    'fx': float(camera_config['K'][0, 0]),
                    'fy': float(camera_config['K'][1, 1]),
                    'cx': float(camera_config['K'][0, 2]),
                    'cy': float(camera_config['K'][1, 2]),
                    'K': camera_config['K'].tolist(),
                    'D': camera_config['D'].tolist(),
                    'P': camera_config['P'].tolist(),
                },
                'extrinsics': {
                    'camera_to_base_link': camera_to_base.tolist()
                },
                'width': camera_config['width'],
                'height': camera_config['height'],
                'topic': f'/sensing/camera/{camera_name}/image_raw/compressed'
            }
        
        print(f"Loaded calibrations for {len(self.camera_configs)} cameras")
    
    def process_and_sync_data(self, bag_path: str, images_dir: str):
        """Process bag and synchronize multi-camera images with ego pose"""
        print(f"Processing bag data from {bag_path}...")
        
        storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3")
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'
        )
        
        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, converter_options)
        
        topics = [self.camera_configs[cam]['topic'] for cam in self.camera_names]
        topics.append('/localization/kinematic_state')
        
        storage_filter = rosbag2_py.StorageFilter(topics=topics)
        reader.set_filter(storage_filter)
        
        topic_types = reader.get_all_topics_and_types()
        type_map = {tt.name: tt.type for tt in topic_types}
        
        message_count = 0
        while reader.has_next():
            topic, data, timestamp = reader.read_next()
            timestamp_sec = timestamp / 1e9
            
            msg_type = get_message(type_map[topic])
            msg = deserialize_message(data, msg_type)
            
            if topic == '/localization/kinematic_state':
                pose_matrix = pose_to_matrix(msg)
                self.pose_buffer.append((timestamp_sec, pose_matrix))
            else:
                camera_name = None
                for cam in self.camera_names:
                    if self.camera_configs[cam]['topic'] == topic:
                        camera_name = cam
                        break
                
                if camera_name:
                    image = self.cv_bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
                    self.image_buffers[camera_name].append((timestamp_sec, image))
                    self.last_camera_times[camera_name] = timestamp_sec
                    
                    self._try_create_frame_from_cycle(timestamp_sec, images_dir)
                    self._clean_buffers(timestamp_sec)
            
            message_count += 1
            if message_count % 1000 == 0:
                print(f"Processed {message_count} messages, extracted {self.frame_id} frames")
                if sum(self.debug_rejection_counts.values()) > 0:
                    print(f"  Rejection reasons: {dict(self.debug_rejection_counts)}")
        
        print(f"Extraction complete: {self.frame_id} frames extracted")
        if sum(self.debug_rejection_counts.values()) > 0:
            print(f"Final rejection stats: {dict(self.debug_rejection_counts)}")
    
    def _try_create_frame_from_cycle(self, current_time: float, images_dir: str):
        """Try to create a synchronized frame from current camera cycle"""
        num_cameras_ready = sum(1 for cam in self.camera_names if cam in self.last_camera_times)
        if not all(cam in self.last_camera_times for cam in self.camera_names):
            if num_cameras_ready > 0 and num_cameras_ready % 3 == 0:
                missing = [c for c in self.camera_names if c not in self.last_camera_times]
                print(f"Waiting for cameras: {num_cameras_ready}/{len(self.camera_names)} ready, missing: {missing[:3]}...")
            return
        
        recent_camera_times = list(self.last_camera_times.values())
        if not recent_camera_times:
            return
        
        min_cam_time = min(recent_camera_times)
        max_cam_time = max(recent_camera_times)
        
        time_spread = max_cam_time - min_cam_time
        if time_spread > self.max_camera_offset:
            self.debug_rejection_counts['time_spread'] += 1
            if self.debug_rejection_counts['time_spread'] <= 5:
                print(f"Camera time spread too large: {time_spread*1000:.1f}ms > {self.max_camera_offset*1000:.1f}ms")
            return
        
        cycle_ref_time = sorted(recent_camera_times)[len(recent_camera_times) // 2]
        current_cycle = int(cycle_ref_time / self.camera_cycle_time)
        
        if current_cycle <= self.last_processed_cycle:
            self.debug_rejection_counts['already_processed'] += 1
            return
        
        camera_images = {}
        camera_timestamps = {}
        
        for camera_name in self.camera_names:
            if camera_name not in self.image_buffers or len(self.image_buffers[camera_name]) == 0:
                print(f"Camera {camera_name} buffer empty despite being in last_camera_times")
                return
            
            latest_img = self.image_buffers[camera_name][-1]
            img_timestamp, image = latest_img
            
            camera_images[camera_name] = image
            camera_timestamps[camera_name] = img_timestamp
        
        closest_pose = self._find_closest_pose(cycle_ref_time)
        if closest_pose is None:
            self.debug_rejection_counts['no_pose'] += 1
            if self.debug_rejection_counts['no_pose'] <= 5:
                print(f"No pose available (pose buffer size: {len(self.pose_buffer)})")
            return
        
        pose_timestamp, pose_matrix = closest_pose
        
        pose_offset = abs(pose_timestamp - cycle_ref_time)
        if pose_offset > self.camera_cycle_time / 2:
            self.debug_rejection_counts['pose_too_far'] += 1
            if self.debug_rejection_counts['pose_too_far'] <= 5:
                print(f"Pose too far from camera cycle: {pose_offset*1000:.1f}ms > {self.camera_cycle_time*500:.1f}ms")
            return
        
        print(f"✓ Creating frame {self.frame_id}: cycle {current_cycle}, "
              f"camera time spread: {time_spread*1000:.1f}ms, "
              f"pose offset: {pose_offset*1000:.1f}ms")
        
        self._save_frame(cycle_ref_time, pose_matrix, camera_images, images_dir)
        
        self.last_processed_cycle = current_cycle
    
    def _find_closest_pose(self, target_time: float) -> Optional[Tuple[float, np.ndarray]]:
        """Find the closest pose to target time from pose buffer"""
        if not self.pose_buffer:
            return None
        
        closest_pose = None
        min_time_diff = float('inf')
        
        for timestamp, pose in self.pose_buffer:
            time_diff = abs(timestamp - target_time)
            if time_diff < min_time_diff:
                min_time_diff = time_diff
                closest_pose = (timestamp, pose)
        
        return closest_pose
    
    def _save_frame(
        self,
        timestamp: float,
        ego_pose: np.ndarray,
        camera_images: Dict[str, np.ndarray],
        images_dir: str
    ):
        """Save synchronized frame data (images only, no map elements)"""
        # Save images
        frame_images = {}
        for camera_name, image in camera_images.items():
            h, w = image.shape[:2]
            image_resized = cv2.resize(
                image,
                (w // self.downsample_factor, h // self.downsample_factor),
                interpolation=cv2.INTER_AREA
            )
            
            image_filename = f"frame_{self.frame_id:06d}_{camera_name}.jpg"
            image_path = os.path.join(images_dir, image_filename)
            cv2.imwrite(image_path, image_resized, [cv2.IMWRITE_JPEG_QUALITY, 95])
            
            frame_images[camera_name] = image_filename
        
        # Extract ego position and heading
        position = ego_pose[:3, 3].tolist()
        heading = extract_heading_from_transform(ego_pose)
        
        # Create frame data (NO map_elements - will be queried at training time)
        frame_data = {
            'frame_id': self.frame_id,
            'timestamp': timestamp,
            'ego_pose': {
                'transform': ego_pose.tolist(),
                'position': position,
                'heading': float(heading)
            },
            'camera_images': frame_images,
        }
        
        self.extracted_frames.append(frame_data)
        self.frame_id += 1
    
    def _clean_buffers(self, current_time: float):
        """Clean old data from buffers"""
        max_buffer_time = self.camera_cycle_time * 2
        
        for camera_name in list(self.image_buffers.keys()):
            self.image_buffers[camera_name] = [
                (t, img) for t, img in self.image_buffers[camera_name]
                if current_time - t < max_buffer_time
            ]
        
        pose_buffer_time = max_buffer_time * 2
        self.pose_buffer = [
            (t, pose) for t, pose in self.pose_buffer
            if current_time - t < pose_buffer_time
        ]
    
    def _get_adjusted_camera_configs(self) -> Dict:
        """Get camera configs adjusted for downsampled images"""
        adjusted_camera_configs = {}
        for camera_name, cam_config in self.camera_configs.items():
            adjusted_config = cam_config.copy()
            adjusted_intrinsics = cam_config['intrinsics'].copy()
            
            scale = 1.0 / self.downsample_factor
            adjusted_intrinsics['fx'] = cam_config['intrinsics']['fx'] * scale
            adjusted_intrinsics['fy'] = cam_config['intrinsics']['fy'] * scale
            adjusted_intrinsics['cx'] = cam_config['intrinsics']['cx'] * scale
            adjusted_intrinsics['cy'] = cam_config['intrinsics']['cy'] * scale
            
            K = np.array(cam_config['intrinsics']['K'])
            K[0, 0] *= scale
            K[1, 1] *= scale
            K[0, 2] *= scale
            K[1, 2] *= scale
            adjusted_intrinsics['K'] = K.tolist()
            
            P = np.array(cam_config['intrinsics']['P'])
            P[0, 0] *= scale
            P[1, 1] *= scale
            P[0, 2] *= scale
            P[1, 2] *= scale
            adjusted_intrinsics['P'] = P.tolist()
            
            adjusted_config['intrinsics'] = adjusted_intrinsics
            adjusted_config['width'] = cam_config['width'] // self.downsample_factor
            adjusted_config['height'] = cam_config['height'] // self.downsample_factor
            
            adjusted_camera_configs[camera_name] = adjusted_config
        
        return adjusted_camera_configs
    
    def save_trajectory(self, trajectory_name: str, trajectory_dir: str):
        """Save trajectory data to JSON (no map elements, just frames)"""
        print(f"Saving trajectory {trajectory_name}...")
        
        adjusted_camera_configs = self._get_adjusted_camera_configs()
        
        trajectory_data = {
            'metadata': {
                'map_hash': self.map_hash,
                'camera_configs': adjusted_camera_configs,
                'bev_range': self.bev_range,
                'lane_point_number': self.lane_point_number,
                'num_frames': len(self.extracted_frames),
                'num_cameras': len(self.camera_names),
                'camera_names': self.camera_names,
                'downsample_factor': self.downsample_factor,
            },
            'frames': self.extracted_frames
        }
        
        trajectory_file = os.path.join(trajectory_dir, 'trajectory.json')
        with open(trajectory_file, 'w') as f:
            json.dump(trajectory_data, f, indent=2)
        
        print(f"Saved trajectory to {trajectory_file}")
        print(f"  - {len(self.extracted_frames)} frames")
        print(f"  - Map hash: {self.map_hash}")
    
    def extract_single_bag(self, bag_path: str, bag_name: str):
        """Extract data from a single bag"""
        # Create trajectory directory
        trajectory_dir = os.path.join(self.output_dir, 'trajectories', bag_name)
        images_dir = os.path.join(trajectory_dir, 'images')
        os.makedirs(images_dir, exist_ok=True)
        
        # Reset state for new bag
        self._reset_for_new_bag()
        
        # Discover cameras from this bag
        self.discover_cameras(bag_path)
        
        # Load camera calibrations (only needs to be done once if same vehicle)
        if not self.camera_configs:
            self.load_camera_calibrations()
        
        # Process bag
        self.process_and_sync_data(bag_path, images_dir)
        
        # Save trajectory
        self.save_trajectory(bag_name, trajectory_dir)
        
        # Update index
        relative_path = f"trajectories/{bag_name}"
        update_index_trajectory(
            self.output_dir,
            bag_name,
            self.map_hash,
            len(self.extracted_frames),
            relative_path
        )
        
        return len(self.extracted_frames)
    
    def run(self):
        """Run complete extraction pipeline for all bags"""
        print("=" * 60)
        print("Multi-Camera Map Extractor - Scalable Multi-Bag Mode")
        print("=" * 60)
        
        # 1. Process map once
        print(f"\n[1/3] Processing map: {self.map_path}")
        self.map_hash = get_or_create_map(
            self.map_path,
            self.output_dir,
            self.lane_point_number
        )
        
        # Update index with map info
        map_info = load_map_info(self.map_hash, self.output_dir)
        update_index_map(
            self.output_dir,
            self.map_hash,
            map_info['original_path'],
            map_info['num_lanelets']
        )
        
        print(f"Map hash: {self.map_hash}")
        print(f"Map has {map_info['num_lanelets']} lanelets")
        
        # 2. Process each bag
        print(f"\n[2/3] Processing {len(self.bag_paths)} bags...")
        total_frames = 0
        skipped_bags = []
        
        for i, bag_path in enumerate(self.bag_paths):
            bag_name = sanitize_bag_name(bag_path)
            print(f"\n--- Bag {i+1}/{len(self.bag_paths)}: {bag_name} ---")
            
            # Check if already exists (unless append mode)
            if trajectory_exists(self.output_dir, bag_name) and not self.append_mode:
                print(f"Trajectory {bag_name} already exists, skipping (use --append to overwrite)")
                skipped_bags.append(bag_name)
                continue
            
            try:
                num_frames = self.extract_single_bag(bag_path, bag_name)
                total_frames += num_frames
            except Exception as e:
                print(f"Error processing {bag_path}: {e}")
                import traceback
                traceback.print_exc()
                continue
        
        # 3. Summary
        print("\n" + "=" * 60)
        print("[3/3] Extraction Summary")
        print("=" * 60)
        print(f"Map: {self.map_path} (hash: {self.map_hash})")
        print(f"Bags processed: {len(self.bag_paths) - len(skipped_bags)}/{len(self.bag_paths)}")
        if skipped_bags:
            print(f"Bags skipped: {skipped_bags}")
        print(f"Total frames extracted: {total_frames}")
        print(f"Output directory: {self.output_dir}")
        
        # Show index summary
        index = load_index(self.output_dir)
        print(f"\nDataset index:")
        print(f"  Maps: {len(index['maps'])}")
        print(f"  Trajectories: {len(index['trajectories'])}")
        
        print("\nExtraction complete!")


def find_bags_in_directory(bag_dir: str) -> List[str]:
    """Find all ROS2 bag directories in a directory"""
    bag_paths = []
    
    # Look for directories containing metadata.yaml (ROS2 bag format)
    for item in os.listdir(bag_dir):
        item_path = os.path.join(bag_dir, item)
        if os.path.isdir(item_path):
            metadata_path = os.path.join(item_path, 'metadata.yaml')
            if os.path.exists(metadata_path):
                bag_paths.append(item_path)
    
    return sorted(bag_paths)


def main():
    parser = argparse.ArgumentParser(
        description='Extract multi-camera data from ROS2 bags for BEVFormer training',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Extract multiple bags with one map
  python multi_camera_map_extractor.py \\
      --map_path /path/to/map.osm \\
      --bag_paths /path/to/bag1,/path/to/bag2,/path/to/bag3 \\
      --output_dir /path/to/extracted_data \\
      --sensor_config_dir /path/to/sensor_config \\
      --camera_info_dir /path/to/camera_info

  # Extract from directory of bags
  python multi_camera_map_extractor.py \\
      --map_path /path/to/map.osm \\
      --bag_dir /path/to/bags/ \\
      --output_dir /path/to/extracted_data \\
      ...

  # Append more bags to existing dataset
  python multi_camera_map_extractor.py \\
      --map_path /path/to/map.osm \\
      --bag_paths /path/to/new_bag \\
      --output_dir /path/to/extracted_data \\
      --append \\
      ...
        """
    )
    
    # Input arguments
    input_group = parser.add_mutually_exclusive_group(required=True)
    input_group.add_argument('--bag_path', type=str, 
                             help='Path to single ROS2 bag (legacy, use --bag_paths for multiple)')
    input_group.add_argument('--bag_paths', type=str,
                             help='Comma-separated paths to ROS2 bags')
    input_group.add_argument('--bag_dir', type=str,
                             help='Directory containing multiple ROS2 bags')
    
    parser.add_argument('--map_path', type=str, required=True, 
                        help='Path to lanelet2 map (.osm)')
    parser.add_argument('--sensor_config_dir', type=str, required=True,
                        help='Directory containing sensors_calibration.yaml and sensor_kit_calibration.yaml')
    parser.add_argument('--camera_info_dir', type=str, required=True,
                        help='Directory containing camera info YAML files')
    parser.add_argument('--output_dir', type=str, required=True, 
                        help='Output directory for extracted data')
    
    # Processing options
    parser.add_argument('--bev_range', type=float, default=80.0,
                        help='BEV range in meters (default: 80)')
    parser.add_argument('--lane_points', type=int, default=20,
                        help='Number of points per lane boundary (default: 20)')
    parser.add_argument('--downsample', type=int, default=2,
                        help='Image downsample factor (default: 2)')
    parser.add_argument('--camera_cycle_time', type=float, default=0.1,
                        help='Camera trigger cycle time in seconds (default: 0.1 for 10Hz)')
    parser.add_argument('--max_camera_offset', type=float, default=0.1,
                        help='Max time offset between cameras in same cycle (default: 0.1s)')
    parser.add_argument('--append', action='store_true',
                        help='Append to existing dataset (overwrite existing trajectories)')
    
    args = parser.parse_args()
    
    # Collect bag paths
    if args.bag_path:
        bag_paths = [args.bag_path]
    elif args.bag_paths:
        bag_paths = [p.strip() for p in args.bag_paths.split(',')]
    elif args.bag_dir:
        bag_paths = find_bags_in_directory(args.bag_dir)
        if not bag_paths:
            print(f"No ROS2 bags found in {args.bag_dir}")
            sys.exit(1)
        print(f"Found {len(bag_paths)} bags in {args.bag_dir}")
    
    # Validate paths
    for bag_path in bag_paths:
        if not os.path.exists(bag_path):
            print(f"Error: Bag path does not exist: {bag_path}")
            sys.exit(1)
    
    if not os.path.exists(args.map_path):
        print(f"Error: Map path does not exist: {args.map_path}")
        sys.exit(1)
    
    # Create extractor and run
    extractor = MultiCameraMapExtractor(
        bag_paths=bag_paths,
        map_path=args.map_path,
        sensor_config_dir=args.sensor_config_dir,
        output_dir=args.output_dir,
        camera_info_dir=args.camera_info_dir,
        bev_range=args.bev_range,
        lane_point_number=args.lane_points,
        downsample_factor=args.downsample,
        camera_cycle_time=args.camera_cycle_time,
        max_camera_offset=args.max_camera_offset,
        append_mode=args.append,
    )
    
    extractor.run()


if __name__ == '__main__':
    main()

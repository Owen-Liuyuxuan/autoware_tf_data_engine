#!/usr/bin/env python3
"""
Map Storage Module for Scalable Multi-Bag Dataset

This module handles:
- Map content hashing for deduplication
- Lanelet resampling and storage as pickle
- Projector info serialization for map reload
- Index management for trajectory-to-map mapping

The goal is to store maps separately from trajectories, enabling:
- Multiple bags to share the same preprocessed map
- Efficient spatial queries using lanelet2's R-tree
- Fast boundary access via pre-resampled numpy arrays
"""

import os
import json
import pickle
import hashlib
import numpy as np
from datetime import datetime
from typing import Dict, List, Any, Optional, Tuple
from pathlib import Path

import lanelet2
from autoware_lanelet2_extension_python.utility import (
    load_info_from_yaml,
    MapProjectorInfo,
)
from autoware_lanelet2_extension_python.projection import MGRSProjector, TransverseMercatorProjector
import autoware_lanelet2_extension_python.utility.utilities as utilities


# Lanelet attribute mappings (shared with dataset.py)
LANELET_TYPE_MAPPING = {
    "road": 0,
    "private": 1,
    "highway": 2,
    "play_street": 3,
    "emergency_lane": 4,
    "bus_lane": 5,
    "bicycle_lane": 6,
    "exit": 7,
    "walkway": 8,
    "shared_walkway": 9,
    "crosswalk": 10,
    "stairs": 11,
    "road_shoulder": 12,
    "pedestrian_lane": 13,
    "bicycle_lane": 14,
    "none": 15,
}

LANELET_LOCATION_MAPPING = {"urban": 0, "nonurban": 1, "private": 2, "none": 3}
LANELET_TURN_DIRECTION_MAPPING = {"straight": 0, "left": 1, "right": 2}


def attribute_or(lanelet, key: str, default: str) -> str:
    """Get lanelet attribute or default value"""
    if key in lanelet.attributes:
        return lanelet.attributes[key]
    return default


def find_projector_yaml(map_path: str) -> Optional[str]:
    """Find projector info YAML file in the same directory as the map"""
    map_dir = os.path.dirname(map_path)
    projector_info = os.path.join(map_dir, "map_projector_info.yaml")
    if os.path.exists(projector_info):
        return projector_info
    return None


def get_lanelet2_projector(projector_info: MapProjectorInfo):
    """Get appropriate lanelet2 projector based on projector info"""
    if projector_info.projector_type == "LOCAL_CARTESIAN_UTM":
        position = lanelet2.GPSPoint(
            projector_info.map_origin.latitude,
            projector_info.map_origin.longitude,
            projector_info.map_origin.altitude,
        )
        origin = lanelet2.io.Origin(position)
        return lanelet2.projection.UtmProjector(origin)
    elif projector_info.projector_type == "MGRS":
        projector = MGRSProjector(lanelet2.io.Origin(0, 0))
        projector.setMGRSCode(projector_info.mgrs_grid)
        return projector
    elif projector_info.projector_type == "TRANSVERSE_MERCATOR":
        position = lanelet2.core.GPSPoint(
            projector_info.map_origin.latitude,
            projector_info.map_origin.longitude,
            projector_info.map_origin.altitude,
        )
        origin = lanelet2.Origin(position)
        return TransverseMercatorProjector(origin)
    elif projector_info.projector_type == "LOCAL_CARTESIAN":
        position = lanelet2.core.GPSPoint(
            projector_info.map_origin.latitude,
            projector_info.map_origin.longitude,
            projector_info.map_origin.altitude,
        )
        origin = lanelet2.io.Origin(position)
        return lanelet2.projection.LocalCartesianProjector(origin)


def get_lanelet2_projector_from_dict(projector_info_dict: Dict) -> Any:
    """Get lanelet2 projector from serialized projector info dictionary"""
    projector_type = projector_info_dict.get("projector_type")
    
    if projector_type == "LOCAL_CARTESIAN_UTM":
        origin_data = projector_info_dict.get("map_origin", {})
        position = lanelet2.GPSPoint(
            origin_data.get("latitude", 0.0),
            origin_data.get("longitude", 0.0),
            origin_data.get("altitude", 0.0),
        )
        origin = lanelet2.io.Origin(position)
        return lanelet2.projection.UtmProjector(origin)
    elif projector_type == "MGRS":
        projector = MGRSProjector(lanelet2.io.Origin(0, 0))
        projector.setMGRSCode(projector_info_dict.get("mgrs_grid", ""))
        return projector
    elif projector_type == "TRANSVERSE_MERCATOR":
        origin_data = projector_info_dict.get("map_origin", {})
        position = lanelet2.core.GPSPoint(
            origin_data.get("latitude", 0.0),
            origin_data.get("longitude", 0.0),
            origin_data.get("altitude", 0.0),
        )
        origin = lanelet2.Origin(position)
        return TransverseMercatorProjector(origin)
    elif projector_type == "LOCAL_CARTESIAN":
        origin_data = projector_info_dict.get("map_origin", {})
        position = lanelet2.core.GPSPoint(
            origin_data.get("latitude", 0.0),
            origin_data.get("longitude", 0.0),
            origin_data.get("altitude", 0.0),
        )
        origin = lanelet2.io.Origin(position)
        return lanelet2.projection.LocalCartesianProjector(origin)
    else:
        # Default to MGRS if unknown
        return MGRSProjector(lanelet2.io.Origin(0, 0))


def serialize_projector_info(projector_info: Optional[MapProjectorInfo]) -> Dict:
    """Serialize MapProjectorInfo to a JSON-compatible dictionary"""
    if projector_info is None:
        return {"projector_type": "MGRS", "mgrs_grid": "", "map_origin": None}
    
    result = {"projector_type": projector_info.projector_type}
    
    if hasattr(projector_info, 'mgrs_grid') and projector_info.mgrs_grid:
        result["mgrs_grid"] = projector_info.mgrs_grid
    
    if hasattr(projector_info, 'map_origin') and projector_info.map_origin:
        result["map_origin"] = {
            "latitude": projector_info.map_origin.latitude,
            "longitude": projector_info.map_origin.longitude,
            "altitude": projector_info.map_origin.altitude,
        }
    
    return result


def compute_map_hash(osm_path: str) -> str:
    """
    Compute SHA256 hash of the map file content.
    
    This ensures the same map file always maps to the same folder,
    avoiding duplication even if the file is copied to different locations.
    
    Args:
        osm_path: Path to the .osm lanelet2 map file
        
    Returns:
        First 16 characters of SHA256 hex digest (64-bit, collision-resistant enough)
    """
    sha256_hash = hashlib.sha256()
    with open(osm_path, "rb") as f:
        # Read in chunks to handle large files
        for chunk in iter(lambda: f.read(8192), b""):
            sha256_hash.update(chunk)
    # Return first 16 characters for shorter folder names
    return sha256_hash.hexdigest()[:16]


def preprocess_and_store_map(
    osm_path: str,
    output_root: str,
    lane_point_number: int = 20
) -> str:
    """
    Preprocess map and store resampled lanelet data.
    
    This function:
    1. Computes content hash of the map file
    2. Checks if preprocessed data already exists
    3. If not, loads the map, resamples all lanelets, and saves as pickle
    4. Saves map info JSON with projector settings for reload
    
    Args:
        osm_path: Path to the .osm lanelet2 map file
        output_root: Root output directory (e.g., /path/to/extracted_data)
        lane_point_number: Number of points to resample each boundary to
        
    Returns:
        Map content hash (used as folder name under maps/)
    """
    # 1. Compute hash
    map_hash = compute_map_hash(osm_path)
    map_dir = os.path.join(output_root, 'maps', map_hash)
    
    # 2. Check if already exists
    if os.path.exists(os.path.join(map_dir, 'resampled_lanelets.pkl')):
        print(f"Map {map_hash}... already exists, skipping preprocessing")
        return map_hash
    
    os.makedirs(map_dir, exist_ok=True)
    print(f"Preprocessing map {map_hash}...")
    
    # 3. Load map with projector
    projector_yaml = find_projector_yaml(osm_path)
    projector_info = load_info_from_yaml(projector_yaml) if projector_yaml else None
    
    if projector_info:
        projector = get_lanelet2_projector(projector_info)
    else:
        print("Warning: No projector info found, using default MGRS")
        projector = MGRSProjector(lanelet2.io.Origin(0.0, 0.0))
    
    lanelet_map = lanelet2.io.load(osm_path, projector)
    
    # 4. Resample all lanelets
    resampled = {}
    for lanelet in lanelet_map.laneletLayer:
        lanelet_id = int(lanelet.id)
        
        # Calculate resolution based on lanelet length
        left_length = lanelet2.geometry.length(lanelet.leftBound)
        right_length = lanelet2.geometry.length(lanelet.rightBound)
        length = max(left_length, right_length)
        
        if length < 0.001:
            continue
        
        resolution = length / (lane_point_number - 1) + 0.0001
        
        # Resample boundaries using existing utilities
        right_bound = utilities.getRightBoundWithOffset(lanelet, 0.0, resolution)
        left_bound = utilities.getLeftBoundWithOffset(lanelet, 0.0, resolution)
        center_line = utilities.generateFineCenterline(lanelet, resolution)
        
        # Convert to numpy arrays for fast access
        left_bound_np = np.array([[p.x, p.y, p.z] for p in left_bound], dtype=np.float32)
        right_bound_np = np.array([[p.x, p.y, p.z] for p in right_bound], dtype=np.float32)
        center_line_np = np.array([[p.x, p.y, p.z] for p in center_line], dtype=np.float32)
        
        # Extract and convert attributes
        subtype_str = attribute_or(lanelet, 'subtype', 'road')
        subtype = LANELET_TYPE_MAPPING.get(subtype_str, LANELET_TYPE_MAPPING['none'])
        
        location_str = attribute_or(lanelet, 'location', 'urban')
        location = LANELET_LOCATION_MAPPING.get(location_str, LANELET_LOCATION_MAPPING['none'])
        
        turn_dir_str = attribute_or(lanelet, 'turn_direction', 'straight')
        turn_direction = LANELET_TURN_DIRECTION_MAPPING.get(turn_dir_str, 0)
        
        speed_limit_str = attribute_or(lanelet, 'speed_limit', '50')
        try:
            speed_limit = float(speed_limit_str) / 100.0  # Normalize to [0, 1]
        except ValueError:
            speed_limit = 0.5
        
        resampled[lanelet_id] = {
            'left_bound': left_bound_np,
            'right_bound': right_bound_np,
            'center_line': center_line_np,
            'subtype': subtype,
            'location': location,
            'turn_direction': turn_direction,
            'speed_limit': speed_limit,
        }
    
    print(f"Resampled {len(resampled)} lanelets")
    
    # 5. Save resampled data as pickle
    pickle_path = os.path.join(map_dir, 'resampled_lanelets.pkl')
    with open(pickle_path, 'wb') as f:
        pickle.dump(resampled, f)
    print(f"Saved resampled lanelets to {pickle_path}")
    
    # 6. Save map info (including projector info for reload)
    map_info = {
        'original_path': os.path.abspath(osm_path),
        'created': datetime.now().isoformat(),
        'num_lanelets': len(resampled),
        'lane_point_number': lane_point_number,
        'projector_info': serialize_projector_info(projector_info),
    }
    
    map_info_path = os.path.join(map_dir, 'map_info.json')
    with open(map_info_path, 'w') as f:
        json.dump(map_info, f, indent=2)
    print(f"Saved map info to {map_info_path}")
    
    return map_hash


def get_or_create_map(
    osm_path: str,
    output_root: str,
    lane_point_number: int = 20
) -> str:
    """
    Get existing preprocessed map or create it if not exists.
    
    This is the main entry point for map preprocessing. It computes the
    map hash and either returns immediately if the map is already processed,
    or calls preprocess_and_store_map to create it.
    
    Args:
        osm_path: Path to the .osm lanelet2 map file
        output_root: Root output directory
        lane_point_number: Number of points to resample each boundary to
        
    Returns:
        Map content hash
    """
    return preprocess_and_store_map(osm_path, output_root, lane_point_number)


def load_map_info(map_hash: str, output_root: str) -> Dict:
    """
    Load map info JSON for a given map hash.
    
    Args:
        map_hash: Map content hash
        output_root: Root output directory
        
    Returns:
        Dictionary containing map info (original_path, projector_info, etc.)
    """
    map_info_path = os.path.join(output_root, 'maps', map_hash, 'map_info.json')
    with open(map_info_path, 'r') as f:
        return json.load(f)


def load_resampled_map(
    map_hash: str,
    output_root: str
) -> Tuple[Any, Dict]:
    """
    Load lanelet2 map (for R-tree queries) and resampled data (for boundaries).
    
    This function loads both:
    1. The original lanelet2 map with correct projector (for spatial queries)
    2. The preprocessed resampled data as numpy arrays (for fast boundary access)
    
    Args:
        map_hash: Map content hash
        output_root: Root output directory
        
    Returns:
        Tuple of (lanelet2.LaneletMap, Dict of resampled lanelet data)
    """
    map_dir = os.path.join(output_root, 'maps', map_hash)
    
    # Load map info
    map_info = load_map_info(map_hash, output_root)
    
    # Load lanelet2 map with correct projector
    projector = get_lanelet2_projector_from_dict(map_info['projector_info'])
    lanelet_map = lanelet2.io.load(map_info['original_path'], projector)
    
    # Load resampled data
    pickle_path = os.path.join(map_dir, 'resampled_lanelets.pkl')
    with open(pickle_path, 'rb') as f:
        resampled = pickle.load(f)
    
    return lanelet_map, resampled


# ============================================================================
# Index Management Functions
# ============================================================================

def load_index(output_root: str) -> Dict:
    """
    Load the master index.json or create empty if not exists.
    
    The index tracks:
    - maps: {hash: {original_path, created, num_lanelets}}
    - trajectories: {name: {map_hash, num_frames, path}}
    
    Args:
        output_root: Root output directory
        
    Returns:
        Index dictionary
    """
    index_path = os.path.join(output_root, 'index.json')
    if os.path.exists(index_path):
        with open(index_path, 'r') as f:
            return json.load(f)
    return {'maps': {}, 'trajectories': {}}


def save_index(output_root: str, index: Dict):
    """
    Save the master index.json.
    
    Args:
        output_root: Root output directory
        index: Index dictionary to save
    """
    index_path = os.path.join(output_root, 'index.json')
    with open(index_path, 'w') as f:
        json.dump(index, f, indent=2)


def update_index_map(
    output_root: str,
    map_hash: str,
    original_path: str,
    num_lanelets: int
):
    """
    Add or update a map entry in the index.
    
    Args:
        output_root: Root output directory
        map_hash: Map content hash
        original_path: Original path to .osm file
        num_lanelets: Number of lanelets in the map
    """
    index = load_index(output_root)
    
    if map_hash not in index['maps']:
        index['maps'][map_hash] = {
            'original_path': original_path,
            'created': datetime.now().isoformat(),
            'num_lanelets': num_lanelets,
        }
        save_index(output_root, index)


def update_index_trajectory(
    output_root: str,
    trajectory_name: str,
    map_hash: str,
    num_frames: int,
    relative_path: str
):
    """
    Add or update a trajectory entry in the index.
    
    Args:
        output_root: Root output directory
        trajectory_name: Name of the trajectory (sanitized bag name)
        map_hash: Map content hash that this trajectory uses
        num_frames: Number of frames in the trajectory
        relative_path: Relative path to trajectory folder (e.g., "trajectories/bag_001")
    """
    index = load_index(output_root)
    
    index['trajectories'][trajectory_name] = {
        'map_hash': map_hash,
        'num_frames': num_frames,
        'path': relative_path,
    }
    
    save_index(output_root, index)


def get_trajectories_for_map(output_root: str, map_hash: str) -> List[str]:
    """
    Get all trajectory names that use a specific map.
    
    Args:
        output_root: Root output directory
        map_hash: Map content hash
        
    Returns:
        List of trajectory names
    """
    index = load_index(output_root)
    return [
        name for name, info in index['trajectories'].items()
        if info['map_hash'] == map_hash
    ]


def trajectory_exists(output_root: str, trajectory_name: str) -> bool:
    """
    Check if a trajectory already exists in the index.
    
    Args:
        output_root: Root output directory
        trajectory_name: Name of the trajectory
        
    Returns:
        True if trajectory exists
    """
    index = load_index(output_root)
    return trajectory_name in index['trajectories']


def sanitize_bag_name(bag_path: str) -> str:
    """
    Convert bag path to a safe trajectory name.
    
    Handles both file and directory bag paths, extracting a clean name
    suitable for use as a folder name.
    
    Args:
        bag_path: Path to the ROS2 bag (file or directory)
        
    Returns:
        Sanitized name string
    """
    bag_path = bag_path.rstrip('/')
    
    # Get the base name
    if os.path.isdir(bag_path):
        name = os.path.basename(bag_path)
    else:
        name = os.path.splitext(os.path.basename(bag_path))[0]
    
    # Remove problematic characters
    name = name.replace(' ', '_').replace('/', '_').replace('\\', '_')
    
    return name


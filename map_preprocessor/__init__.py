"""Map Preprocessor Module

This module provides utilities for:
- Map storage with content-based hashing
- Lanelet resampling and pickle storage
- Index management for trajectory-to-map mapping
"""

from .map_storage import (
    compute_map_hash,
    preprocess_and_store_map,
    get_or_create_map,
    load_map_info,
    load_resampled_map,
    load_index,
    save_index,
    update_index_map,
    update_index_trajectory,
    get_trajectories_for_map,
    trajectory_exists,
    sanitize_bag_name,
    get_lanelet2_projector,
    get_lanelet2_projector_from_dict,
    serialize_projector_info,
    find_projector_yaml,
    attribute_or,
    LANELET_TYPE_MAPPING,
    LANELET_LOCATION_MAPPING,
    LANELET_TURN_DIRECTION_MAPPING,
)

__all__ = [
    'compute_map_hash',
    'preprocess_and_store_map',
    'get_or_create_map',
    'load_map_info',
    'load_resampled_map',
    'load_index',
    'save_index',
    'update_index_map',
    'update_index_trajectory',
    'get_trajectories_for_map',
    'trajectory_exists',
    'sanitize_bag_name',
    'get_lanelet2_projector',
    'get_lanelet2_projector_from_dict',
    'serialize_projector_info',
    'find_projector_yaml',
    'attribute_or',
    'LANELET_TYPE_MAPPING',
    'LANELET_LOCATION_MAPPING',
    'LANELET_TURN_DIRECTION_MAPPING',
]


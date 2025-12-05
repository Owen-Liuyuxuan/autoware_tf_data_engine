# Pilot-Auto Data Extraction Engine

## Overview

The AutoDrive Data Extraction Engine is a specialized tool designed for extracting and processing data from ROS bags for autonomous driving applications. It focuses on preparing training and testing datasets for local learnable planners in the Autoware ecosystem.

This tool extracts critical information from ROS bags including:

- Object detection and tracking data
- Vehicle trajectory information (both historical and future)
- Map data and route information
- Vehicle positioning and kinematics

![gif](docs/overfitting_scene_demo.gif)

## Features

- [x] **Comprehensive Data Extraction**: Processes ROS bags to extract object tracking, ego vehicle positioning, traffic_light and map data (WIP, currently only visualize and deal with all lanelet elements.)
- [x] **Map Processing**: Integrates with Lanelet2 maps to provide contextual road network information (WIP, currently not supporting lanelet2 polygon)
- [ ] **Object Tracking**: Maintains object state information for surrounding vehicles and obstacles (WIP, currently only record and utilize the current tracked objects)
- [x] **Traffic Light Information**: Record the most recent traffic light information at each key frame.
- [x] **Meta Information**: Record the vehicle informations and the lanelet2 maps information used for each sequences of bags.
- [x] **Trajectory Analysis**: Captures both historical and predicted future trajectories
- [x] **Visualization**: Optional visualization of extracted scenes with pygame-based renderer
- [x] **Frame Selection**: Constant skipping frame selection to reduce redundancy in the dataset

## Requirements

- ROS 2 (with Autoware packages)
- Python 3.6+
- Lanelet2 Python API for Autoware
- NumPy, Matplotlib, PyGame

## Installation

Ensure you have the required Autoware environment set up:

```bash
source /path/to/pilot-auto/install/setup.bash
source /path/to/lanelet2_python_api_for_autoware/setup.bash
```

## Usage

Run the extraction tool using the provided shell script:

```bash
./main.sh <rosbag_path> <osm_map_path> <output_path> <vehicle_model>
```

### Parameters:
- `<rosbag_path>`: Path to the ROS bag file containing driving data
- `<osm_map_path>`: Path to the OSM/Lanelet2 map file (.osm)
- `<output_path>`: Directory where extracted data will be saved
- `<vehicle_model>`: Vehicle model name used in autoware, which will be used to find the `vehicle_info.yaml`

## Architecture

The engine consists of several key components:

1. **BagExtractor**: Main class that processes ROS bags and orchestrates the extraction process
2. **BaseStepEngine**: Controls the extraction flow and key frame selection
3. **MapManager**: Handles map loading and processing, providing local map information and traffic light information
4. **BaseTracker**: Processes object detection messages and maintains object state
5. **SceneVisualizer**: Optional component for visualizing the extracted scenes

## Output Format

The extracted data is saved in a structured format containing:

- Ego vehicle and map meta information
- Traffic light information
- Frame information
- Object detection data (position, velocity, type)
- Ego vehicle trajectory (past and future)
- Map elements and route information

This data is specifically formatted for training machine learning models for autonomous driving planning tasks.

### New Compact JSON Scheme (v2)

The new schema eliminates data duplication by storing all timestamped data only once in map frame:

```python
{
  "metadata": {
    "lanelet2_map": "path/to/map.osm",
    "vehicle_params": {
      "wheel_base": float,
      "max_steer_angle": float,
      "front_overhang": float,
      "rear_overhang": float,
      "left_overhang": float,
      "right_overhang": float
    },
    "start_timestamp": float,
    "end_timestamp": float,
    "time_step": float  // average time between steps
  },
  "ego_states": [
    {
      "step": int,
      "timestamp": float,
      "transform": [[4, 4]],  // 4x4 matrix in map frame
      "velocity": [3],  // velocity in map frame
      "operation_mode": int,
      "vehicle_status": int
    }
  ],
  "object_detections": [
    {
      "step": int,
      "timestamp": float,
      "objects": [
        {
          "id": "uuid",
          "type": int,
          "transform": [[4, 4]],  // in map frame
          "velocity": [3],  // in map frame
          "global_footprint": [[N, 3]]  // in map frame
        }
      ]
    }
  ],
  "traffic_lights": [
    {
      "step": int,
      "timestamp": float,
      "status": {
        "group_id": [
          {
            "color": int,
            "shape": int,
            "status": int,
            "confidence": float
          }
        ]
      }
    }
  ],
  "key_frames": [
    {
      "step": int,
      "timestamp": float,
      "routes": [int],  // lanelet IDs
      "nearby_drivable_path": [int],
      "nearby_lanelets_ids": [int],
      "associated_traffic_light_ids": [int]
    }
  ]
}
```

### Key Improvements

1. **No Data Duplication**: Each timestamp stored only once (90% storage reduction)
2. **Map Frame Storage**: All data in map frame, transformed to ego frame during training
3. **Flexible History**: Can change history/future window sizes without re-extraction
4. **Efficient Lookup**: Key frames reference steps to extract time windows

### Legacy Format Support

The dataset loader supports both old and new formats for backward compatibility. Old format data will need to be re-extracted to benefit from the new compact structure.

## License

This project is part of the Autoware ecosystem and follows its licensing terms.

## Contributing

Contributions to improve the extraction engine are welcome. Please ensure your code follows the project's coding standards.
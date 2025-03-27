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

### Current JSON Scheme

```python
"""
Json file structure:
  str(frame_id) {
    frame: frame,
    objects: [
        {
            "id": object_id,
            "type": object_type as long,
            "transform": T [4, 4], - transform of the object in the world frame.
            "velocity": v [3], - velocity of the object in the world frame.
            "global_footprint": [4, 3] - footprint of the object in the world frame.
        },
        {},
    ],
    history_trajectories_transform_list: [N_h, 4, 4], past trajectory of the vehicle in the world frame. N=10,
    future_trajectories_transform_list: [N_f, 4, 4], future trajectory of the vehicle in the world frame. N=30,
    history_trajectories_speed_list: [N_h, 1], past speed of the vehicle in the world frame. N=10,
    future_trajectories_speed_list: [N_f, 3], future speed of the vehicle in the world frame. N=30,
    routes: [], list of lanelet2 id that is route of the vehicle in the neighborhood.
    nearby_lanelets_ids: [], list of lanelet2 id that is nearby the vehicle in the neighborhood.
  }
"""
```

## License

This project is part of the Autoware ecosystem and follows its licensing terms.

## Contributing

Contributions to improve the extraction engine are welcome. Please ensure your code follows the project's coding standards.
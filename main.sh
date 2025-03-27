#!/bin/bash

# Source the required setup files
source /home/ukenryu/pilot-auto.xx1/install/setup.bash
source /home/ukenryu/map_test/lanelet2_python_api_for_autoware/setup.bash

# Check if both arguments are provided
if [ $# -ne 4 ]; then
    echo "Error: Missing arguments"
    echo "Usage: $0 <rosbag_path> <osm_path> <output_path> <vehicle_model>"
    exit 1
fi

# Store arguments in named variables for better readability
ROSBAG_PATH="$1"
OSM_PATH="$2"
OUTPUT_PATH="$3"
VEHICLE_MODEL="$4"

# Validate if output directory was created successfully
if [ ! -f "$OSM_PATH" ]; then
    echo "Error: OSMMap path not found: $OSM_PATH"
    exit 1
fi

# Create output directory if it doesn't exist
mkdir -p "$OUTPUT_PATH"

if [ ! -d "$OUTPUT_PATH" ]; then
    echo "Error: Failed to create output directory: $OUTPUT_PATH"
    exit 1
fi

echo "Processing rosbag: $ROSBAG_PATH"
echo "Output directory: $OUTPUT_PATH"
echo "Vehicle Model: $VEHICLE_MODEL"

python3 bag_preprocessor.py "$ROSBAG_PATH" "$OSM_PATH" "$OUTPUT_PATH" "$VEHICLE_MODEL"

#!/bin/bash
set -e
# Check if all arguments are provided
if [ $# -ne 4 ]; then
    echo "Error: Missing arguments"
    echo "Usage: $0 <data_directory> <osm_map_path> <output_base_path> <vehicle_model>"
    exit 1
fi

# Store arguments in named variables for better readability
DATA_DIR="$1"
OSM_MAP_PATH="$2"
OUTPUT_BASE_PATH="$3"
VEHICLE_MODEL="$4"

# Check if data directory exists
if [ ! -d "$DATA_DIR" ]; then
    echo "Error: Data directory not found: $DATA_DIR"
    exit 1
fi

# Check if OSM map file exists
if [ ! -f "$OSM_MAP_PATH" ]; then
    echo "Error: OSM map file not found: $OSM_MAP_PATH"
    exit 1
fi

# Create output base directory if it doesn't exist
mkdir -p "$OUTPUT_BASE_PATH"

# Loop through all subdirectories in the data directory
for bag_dir in "$DATA_DIR"/*/ ; do
    if [ -d "$bag_dir" ]; then
        # Extract the directory name for the output folder
        dir_name=$(basename "$bag_dir")
        
        echo "Processing bag directory: $bag_dir"
        echo "Output directory: $output_dir"
        
        # Run the main.sh script with the appropriate arguments
        bash ~/map_test/lanelet2_python_api_for_autoware/extract_bags/main.sh "$bag_dir" "$OSM_MAP_PATH" "$OUTPUT_BASE_PATH" "$VEHICLE_MODEL"
        
        sleep 3
        echo "Finished processing: $bag_dir"
        echo "------------------------"
    fi
done

echo "All bag directories have been processed."
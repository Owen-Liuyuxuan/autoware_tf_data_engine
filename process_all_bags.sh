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
source /opt/ros/humble/setup.bash

# Function to check bag duration
check_bag_duration() {
    local bag_dir="$1"
    local min_duration=10  # Minimum duration in seconds
    
    # Run ros2 bag info and capture the output
    local bag_info=$(ros2 bag info "$bag_dir" 2>/dev/null)
    
    # Check if the command was successful
    if [ $? -ne 0 ]; then
        echo "Error: Failed to get bag info for $bag_dir"
        return 1
    fi
    
    # Extract duration from the output
    local duration_line=$(echo "$bag_info" | grep "Duration")
    if [ -z "$duration_line" ]; then
        echo "Error: Could not find duration information for $bag_dir"
        return 1
    fi
    
    # Parse the duration (format: Duration: XXs (XX.XXXs)
    local duration=$(echo "$duration_line" | sed -E 's/.*Duration:[[:space:]]+([0-9]+)\.?[0-9]*s.*/\1/')
    
    # Check if duration is a number
    if ! [[ "$duration" =~ ^[0-9]+$ ]]; then
        echo "Error: Could not parse duration for $bag_dir"
        return 1
    fi
    
    # Compare with minimum duration
    if [ "$duration" -lt "$min_duration" ]; then
        echo "Skipping $bag_dir: Duration ($duration seconds) is less than minimum required ($min_duration seconds)"
        return z
    fi
    
    return 0
}

# Loop through all subdirectories in the data directory
for bag_dir in "$DATA_DIR"/*/ ; do
    if [ -d "$bag_dir" ]; then
        # Extract the directory name for the output folder
        dir_name=$(basename "$bag_dir")
        output_dir="$OUTPUT_BASE_PATH/$dir_name"
        
        echo "Checking bag directory: $bag_dir"
        
        # Check if the bag duration is longer than 10 seconds
        if check_bag_duration "$bag_dir"; then
            echo "Processing bag directory: $bag_dir"
            echo "Output directory: $output_dir"
            
            # Run the main.sh script with the appropriate arguments
            bash ~/map_test/lanelet2_python_api_for_autoware/extract_bags/main.sh "$bag_dir" "$OSM_MAP_PATH" "$OUTPUT_BASE_PATH" "$VEHICLE_MODEL"
            
            sleep 3
            echo "Finished processing: $bag_dir"
        else
            echo "Skipping bag directory: $bag_dir (duration < 10 seconds)"
        fi
        echo "------------------------"
    fi
done

echo "All bag directories have been processed."
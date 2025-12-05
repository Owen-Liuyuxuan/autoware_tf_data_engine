#!/bin/bash
set -e

# Check if all arguments are provided
if [ $# -lt 4 ] || [ $# -gt 5 ]; then
    echo "Error: Missing or extra arguments"
    echo "Usage: $0 <data_directory> <osm_map_path> <output_base_path> <vehicle_model> [max_parallel_jobs]"
    echo "  max_parallel_jobs: Optional, number of parallel extractions (default: number of CPU cores)"
    exit 1
fi

# Store arguments in named variables for better readability
DATA_DIR="$1"
OSM_MAP_PATH="$2"
OUTPUT_BASE_PATH="$3"
VEHICLE_MODEL="$4"
MAX_PARALLEL_JOBS="${5:-$(nproc)}"  # Default to number of CPU cores if not specified

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

# Create logs directory for parallel execution
LOG_DIR="$OUTPUT_BASE_PATH/logs"
mkdir -p "$LOG_DIR"

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
        return 1
    fi
    
    return 0
}

# Function to process a single bag directory
process_bag() {
    local bag_dir="$1"
    local dir_name=$(basename "$bag_dir")
    local log_file="$LOG_DIR/${dir_name}.log"
    local start_time=$(date +%s)
    
    {
        echo "=========================================="
        echo "Processing: $bag_dir"
        echo "Started at: $(date)"
        echo "=========================================="
        
        # Check if the bag duration is longer than 10 seconds
        if check_bag_duration "$bag_dir"; then
            echo "Bag duration check passed, starting extraction..."
            
            # Run the main.sh script with the appropriate arguments
            bash ~/map_test/lanelet2_python_api_for_autoware/extract_bags/main.sh \
                "$bag_dir" "$OSM_MAP_PATH" "$OUTPUT_BASE_PATH" "$VEHICLE_MODEL"
            
            local end_time=$(date +%s)
            local duration=$((end_time - start_time))
            echo "=========================================="
            echo "Finished processing: $bag_dir"
            echo "Duration: ${duration}s"
            echo "Completed at: $(date)"
            echo "=========================================="
        else
            echo "Skipping bag directory: $bag_dir (duration < 10 seconds)"
        fi
    } > "$log_file" 2>&1
    
    # Return the exit status
    return $?
}

# Track job statistics
TOTAL_BAGS=0
PROCESSED_BAGS=0
SKIPPED_BAGS=0
FAILED_BAGS=0
declare -a PIDS=()
declare -a BAG_DIRS=()

echo "=========================================="
echo "Parallel Bag Extraction"
echo "=========================================="
echo "Data directory: $DATA_DIR"
echo "Output directory: $OUTPUT_BASE_PATH"
echo "Max parallel jobs: $MAX_PARALLEL_JOBS"
echo "Log directory: $LOG_DIR"
echo "=========================================="
echo ""

# First pass: collect all valid bag directories
echo "Scanning bag directories..."
VALID_BAG_DIRS=()
for bag_dir in "$DATA_DIR"/*/ ; do
    if [ -d "$bag_dir" ]; then
        VALID_BAG_DIRS+=("$bag_dir")
        TOTAL_BAGS=$((TOTAL_BAGS + 1))
    fi
done

echo "Found $TOTAL_BAGS bag directories"
echo "Starting parallel processing with up to $MAX_PARALLEL_JOBS jobs..."
echo ""

# Process bags in parallel
for bag_dir in "${VALID_BAG_DIRS[@]}"; do
    # Wait if we've reached the maximum number of parallel jobs
    while [ ${#PIDS[@]} -ge "$MAX_PARALLEL_JOBS" ]; do
        # Check for completed jobs
        for i in "${!PIDS[@]}"; do
            if ! kill -0 "${PIDS[$i]}" 2>/dev/null; then
                # Job completed, check exit status
                wait "${PIDS[$i]}"
                exit_code=$?
                
                if [ $exit_code -eq 0 ]; then
                    PROCESSED_BAGS=$((PROCESSED_BAGS + 1))
                else
                    FAILED_BAGS=$((FAILED_BAGS + 1))
                    echo "WARNING: Failed to process ${BAG_DIRS[$i]} (check log: $LOG_DIR/$(basename "${BAG_DIRS[$i]}").log)"
                fi
                
                # Remove completed job from arrays
                unset PIDS[$i]
                unset BAG_DIRS[$i]
            fi
        done
        
        # Rebuild arrays to remove gaps
        PIDS=("${PIDS[@]}")
        BAG_DIRS=("${BAG_DIRS[@]}")
        
        # Small sleep to avoid busy waiting
        sleep 0.1
    done
    
    # Start new job in background
    process_bag "$bag_dir" &
    pid=$!
    PIDS+=($pid)
    BAG_DIRS+=("$bag_dir")
    
    echo "Started processing: $bag_dir (PID: $pid)"
done

# Wait for all remaining jobs to complete
echo ""
echo "Waiting for all jobs to complete..."
for i in "${!PIDS[@]}"; do
    wait "${PIDS[$i]}"
    exit_code=$?
    
    if [ $exit_code -eq 0 ]; then
        PROCESSED_BAGS=$((PROCESSED_BAGS + 1))
    else
        FAILED_BAGS=$((FAILED_BAGS + 1))
        echo "WARNING: Failed to process ${BAG_DIRS[$i]} (check log: $LOG_DIR/$(basename "${BAG_DIRS[$i]}").log)"
    fi
done

# Calculate skipped bags (those that didn't pass duration check)
SKIPPED_BAGS=$((TOTAL_BAGS - PROCESSED_BAGS - FAILED_BAGS))

# Print summary
echo ""
echo "=========================================="
echo "Processing Summary"
echo "=========================================="
echo "Total bags: $TOTAL_BAGS"
echo "Successfully processed: $PROCESSED_BAGS"
echo "Failed: $FAILED_BAGS"
echo "Skipped (duration < 10s): $SKIPPED_BAGS"
echo "Log files: $LOG_DIR"
echo "=========================================="

if [ $FAILED_BAGS -gt 0 ]; then
    echo ""
    echo "WARNING: $FAILED_BAGS bag(s) failed to process. Check log files for details."
    exit 1
fi

echo "All bag directories have been processed successfully."

cd ~/map_test/lanelet2_python_api_for_autoware/naiveTF

python3 main.py --train_cache /media/ukenryu/external_ssd/jpt_naivetf/  \
    --val_cache /media/ukenryu/external_ssd/jpt_naivetf/cache_d051d971-3ed6-432f-b347-4e416906c773_2025-07-08-14-03-57.json \
    --epochs 300 --lr 1e-4
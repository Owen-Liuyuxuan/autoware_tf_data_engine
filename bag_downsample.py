#!/usr/bin/env python3

"""
ROS 2 Bag Downsampler

This script downsamples specific topics in ROS 2 bag files to reduce their size and frequency.
It's particularly useful for handling topics that have unnecessarily high publication rates.

Key features:
- Processes single bag files or directories containing multiple bags
- Selectively downsamples specified topics while preserving all other topics
- Configurable downsampling ratio

Note on Traffic Light Topics:
In random simulation environments, traffic light recognition topics 
('/perception/traffic_light_recognition/traffic_signals') are often published at 
approximately 4 times the frequency needed for practical use. This higher-than-necessary 
publication rate leads to:
1. Unnecessarily large bag files
2. Redundant information (minimal state changes between consecutive messages)
3. Increased processing overhead when replaying bags

The default downsampling ratio of 4:1 is specifically chosen to address this issue,
bringing the traffic light topic frequency down to a more reasonable level while
still maintaining all the necessary state transitions and information.
"""

import os
import sys
import argparse
import subprocess
import tempfile
import shutil
from pathlib import Path
import yaml
import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py
from rosbag2_py import StorageOptions, ConverterOptions, TopicMetadata

class BagDownsampler:
    def __init__(self, input_bag_path, output_bag_path, topic_to_downsample, downsample_ratio=4):
        """
        Initialize the bag downsampler.
        
        Args:
            input_bag_path: Path to the input bag
            output_bag_path: Path to save the output bag
            topic_to_downsample: Topic to downsample
            downsample_ratio: Keep 1 message for every N messages
        """
        self.input_bag_path = input_bag_path
        self.output_bag_path = output_bag_path
        self.topic_to_downsample = topic_to_downsample
        self.downsample_ratio = downsample_ratio
        
        # Create output directory if it doesn't exist
        os.makedirs(os.path.dirname(os.path.abspath(output_bag_path)), exist_ok=True)
        
        # Initialize ROS 2
        rclpy.init()
    
    def process_bag(self):
        """Process the bag file and create a new downsampled version."""
        print(f"Processing bag: {self.input_bag_path}")
        print(f"Downsampling topic {self.topic_to_downsample} (1 in {self.downsample_ratio})")
        
        # Set up reader
        storage_options_read = StorageOptions(uri=self.input_bag_path, storage_id="sqlite3")
        converter_options = ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr"
        )
        
        # Open reader
        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options_read, converter_options)

        # Set up writer
        storage_options_write = StorageOptions(uri=self.output_bag_path, storage_id="sqlite3")
        writer = rosbag2_py.SequentialWriter()
        writer.open(storage_options_write, converter_options)
        
        # Get topic info from metadata
        topic_types = {}
        topic_types = reader.get_all_topics_and_types()
        for topic_type in topic_types:
            writer.create_topic(topic_type)
        
        # Process messages
        counter = 0
        downsample_counter = 0
        
        print("Starting to process messages...")
        
        while reader.has_next():
            topic_name, data, timestamp = reader.read_next()
            
            # If it's the topic we want to downsample
            if topic_name == self.topic_to_downsample:
                downsample_counter += 1
                if downsample_counter % self.downsample_ratio != 0:
                    continue  # Skip this message
            
            # Write the message to the output bag
            writer.write(topic_name, data, timestamp)
            counter += 1
            
            if counter % 1000 == 0:
                print(f"Processed {counter} messages...")
        
        print(f"Finished processing. Total messages written: {counter}")
        print(f"Output bag saved to: {self.output_bag_path}")
        
        # Clean up
        rclpy.shutdown()

def process_directory(input_dir, output_dir, topic, ratio):
    """Process all bag files in a directory."""
    input_path = Path(input_dir)
    output_path = Path(output_dir)
    
    # Create output directory
    output_path.mkdir(parents=True, exist_ok=True)
    
    # Find all bag directories
    bag_dirs = [d for d in input_path.iterdir() if d.is_dir() and (d / "metadata.yaml").exists()]
    
    if not bag_dirs:
        print(f"No bag files found in {input_dir}")
        return
    
    print(f"Found {len(bag_dirs)} bag files to process")
    
    # Process each bag
    for bag_dir in bag_dirs:
        bag_name = bag_dir.name
        output_bag_path = str(output_path / bag_name)
        
        # Skip if output already exists
        if Path(output_bag_path).exists():
            print(f"Skipping {bag_name} - output already exists")
            continue
        
        # Process the bag
        downsampler = BagDownsampler(str(bag_dir), output_bag_path, topic, ratio)
        downsampler.process_bag()

def main():
    parser = argparse.ArgumentParser(description='Downsample a topic in ROS 2 bag files')
    parser.add_argument('--input', '-i', required=True, help='Input bag file or directory containing bags')
    parser.add_argument('--output', '-o', required=True, help='Output bag file or directory')
    parser.add_argument('--topic', '-t', default='/perception/traffic_light_recognition/traffic_signals', 
                        help='Topic to downsample')
    parser.add_argument('--ratio', '-r', type=int, default=4, 
                        help='Downsample ratio (keep 1 in N messages)')
    
    args = parser.parse_args()
    
    # Check if input is a directory or a single bag
    input_path = Path(args.input)
    if input_path.is_dir() and not (input_path / "metadata.yaml").exists():
        # It's a directory containing multiple bags
        process_directory(args.input, args.output, args.topic, args.ratio)
    else:
        # It's a single bag
        downsampler = BagDownsampler(args.input, args.output, args.topic, args.ratio)
        downsampler.process_bag()

if __name__ == '__main__':
    main()
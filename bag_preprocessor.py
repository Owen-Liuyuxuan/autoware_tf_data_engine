#!/usr/bin/env python3

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import json
import numpy as np
import sys
import os
from typing import Dict, List, Any
from datetime import datetime
from map_preprocessor.map_manager import MapManager
from object_tracker.base_object_tracker import BaseTracker
from step_engine.base_step_engine import BaseStepEngine

from autoware_perception_msgs.msg import TrackedObjects
from nav_msgs.msg import Odometry


class BagExtractor:
    def __init__(self, bag_path: str, map_path: str, output_path: str):
        self.bag_path = bag_path
        self.output_path = output_path

        # Define topics to extract
        self.topics = {
            "objects": "/perception/object_recognition/objects",  # adjust topic name as needed
            "ego_position": "/localization/kinematic_state",  # adjust topic name as needed
            "global_plan": "/planning/mission_planning/route",
            "local_plan": "/planning/scenario_planning/trajectory",
            "traffic_light": "/perception/traffic_light_recognition/traffic_signals",
        }

        self.trace_back_step = 10  # Number of steps to look back in history
        self.look_ahead_steps = 30  # Number of steps to look ahead in future
        self.key_frame_step = 5  # Step size for key frames
        self.ego_state_skip_frame = 5
        self.ego_state_small_counter = 0

        # if the bag path is a directory, use the directory name as the bag_name
        if os.path.isdir(bag_path):
            self.bag_name = os.path.basename(bag_path)
        else:
            self.bag_name = os.path.splitext(os.path.basename(bag_path))[0]  # Extract the bag name without extension

        # Storage for extracted data
        self.extracted_data = {}

        self.map_manager = MapManager(map_path)
        self.object_tracker = BaseTracker(self.trace_back_step, self.look_ahead_steps)
        self.step_engine = BaseStepEngine(
            self.object_tracker,
            self.map_manager,
            self.trace_back_step,
            self.look_ahead_steps,
            self.key_frame_step,
            enable_visualization=True,
            visualization_path="temp_vis",
        )

    def open_bag(self):
        """Initialize and open the rosbag with topic filtering"""
        storage_options = rosbag2_py.StorageOptions(
            uri=self.bag_path, 
            storage_id="sqlite3"
        )
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr"
        )
        
        # Create a list of topics we want to read
        target_topics = list(self.topics.values())
        
        # Create storage filter
        storage_filter = rosbag2_py.StorageFilter(
            topics=target_topics
        )
        
        self.reader = rosbag2_py.SequentialReader()
        self.reader.open(storage_options, converter_options)
        self.reader.set_filter(storage_filter)

    def extract_neighbouring_objects(self, msg: TrackedObjects):
        """Extract information about neighbouring objects"""
        # Implement object extraction logic
        # Example: position, velocity, classification, etc.
        self.step_engine.process_objects(msg)

    def extract_current_position(self, msg: Odometry):
        """Extract current vehicle position"""
        # Implement current position extraction logic
        # Example: x, y, z, orientation
        if (self.ego_state_small_counter+1) % self.ego_state_skip_frame == 0:
            self.ego_state_small_counter = 0
            return self.step_engine.process_ego(msg)
        self.ego_state_small_counter += 1


    def process_message(self, topic_name: str, msg):
        """Process each message based on its topic"""
        ## For the basic implementation, we only use objects and ego position
        if topic_name == self.topics["objects"]:
            return self.extract_neighbouring_objects(msg)
        elif topic_name == self.topics["ego_position"]:
            return self.extract_current_position(msg)
        elif topic_name == self.topics["global_plan"]:
            return self.map_manager.set_global_path(msg)

    def extract_data(self):
        """Main extraction loop"""
        self.open_bag()

        while self.reader.has_next():
            topic_name, data, timestamp = self.reader.read_next()
            # Get the message type for this topic
            topic_types = self.reader.get_all_topics_and_types()
            msg_type = None
            for topic_type in topic_types:
                if topic_type.name == topic_name:
                    msg_type = topic_type.type
                    break

            if msg_type is None:
                continue

            # Deserialize the message
            # print("msg type", msg_type)
            # print("topic name", topic_name)
            msg_cls = get_message(msg_type)
            msg = deserialize_message(data, msg_cls)

            # Process the message
            return_data_dict = self.process_message(topic_name, msg)
            if return_data_dict is not None:
                frame = return_data_dict["frame"]
                selected_keys = ["frame", "objects",
                                  "history_trajectories_transform_list", "future_trajectories_transform_list",
                                  "history_trajectories_speed_list", "future_trajectories_speed_list",
                                   "routes", "nearby_lanelets_ids"]
                self.extracted_data[frame] = {}
                for key in selected_keys:
                    self.extracted_data[frame][key] = return_data_dict[key]

        


    def save_results(self):
        """Save extracted data to JSON files"""
        try:
            # Save each data type to a separate file
            with open(os.path.join(self.output_path, f"cache_{self.bag_name}.json"), "w") as f:
                json.dump(self.extracted_data, f, indent=2)
                
            print(f"Data successfully saved to {self.output_path}")
        except Exception as e:
            print(f"Error saving results: {str(e)}")
            raise


def main():
    if len(sys.argv) != 4:
        print("Usage: python3 extract_bag.py <bag_path> <map_path> <output_path>")
        sys.exit(1)

    bag_path = sys.argv[1]
    map_path = sys.argv[2]
    output_path = sys.argv[3]

    extractor = BagExtractor(bag_path, map_path, output_path)
    extractor.extract_data()
    extractor.save_results()


if __name__ == "__main__":
    main()

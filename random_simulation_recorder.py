#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import subprocess
import signal
import os
import datetime
from autoware_auto_planning_msgs.msg import Route  # Assuming this is the message type for the 
from tier4_planning_msgs.msg import RouteState
import time

ROSBAG_DIRECTORY="/media/ukenryu/External SSD/random_simulation_data"

class BagRecorder(Node):
    def __init__(self):
        super().__init__('bag_recorder')
        
        # Define topics to record
        self.topics_to_record = [
            "/perception/object_recognition/objects",  # adjust topic name as needed
            "/localization/kinematic_state",  # adjust topic name as needed
            "/planning/mission_planning/route",
            "/planning/scenario_planning/trajectory",
            "/perception/traffic_light_recognition/traffic_signals",
            "/api/operation_mode/state", # identify if we have launch start in autoware
            "/vehicle/status/control_mode" # identify if we are manually driving
        ]
        
        # Initialize recorder process
        self.recorder_process = None
        self.recording_active = False
        self.current_bag_path = None
        
        
        self.route_state_subscription = self.create_subscription(
            RouteState, 
            "/planning/mission_planning/state",
            self.route_state_callback,
            10
        )

        self.get_logger().info('Bag recorder initialized. Waiting for route updates...')
    
    
    def route_state_callback(self, msg:RouteState):
        if msg.state == 4: # SET
            if self.recording_active:
                self.stop_recording()
            # Start a new recording
            self.start_recording()
        else:
            if self.recording_active:
                self.stop_recording()

    
    def start_recording(self):
        """Start a new rosbag recording"""
        # Create timestamp for bag name
        timestamp = datetime.datetime.now().strftime("%Y_%m_%d-%H_%M_%S")
        
        # Create directory for bags if it doesn't exist
        bags_dir = os.path.expanduser(ROSBAG_DIRECTORY)
        os.makedirs(bags_dir, exist_ok=True)
        
        # Set bag path
        self.current_bag_path = f"{bags_dir}/route_{timestamp}"
        
        # Prepare command
        cmd = ["ros2", "bag", "record", "-o", self.current_bag_path]
        cmd.extend(self.topics_to_record)
        
        # Start recording process
        self.get_logger().info(f"Starting recording to {self.current_bag_path}")
        self.recorder_process = subprocess.Popen(cmd)
        self.recording_active = True
    
    def stop_recording(self):
        """Stop the current recording"""
        if self.recorder_process and self.recording_active:
            self.get_logger().info(f"Stopping recording: {self.current_bag_path}")
            
            # Send SIGINT (equivalent to Ctrl+C) to gracefully terminate the process
            self.recorder_process.send_signal(signal.SIGINT)
            
            # Wait for process to terminate
            try:
                self.recorder_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.get_logger().warning("Recorder process did not terminate gracefully, forcing termination")
                self.recorder_process.kill()
            
            self.recording_active = False
            self.get_logger().info("Recording stopped")
    
    def shutdown(self):
        """Clean shutdown of the node"""
        self.stop_recording()


def main(args=None):
    rclpy.init(args=args)
    
    bag_recorder = BagRecorder()
    
    try:
        rclpy.spin(bag_recorder)
    except KeyboardInterrupt:
        pass
    finally:
        bag_recorder.shutdown()
        bag_recorder.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

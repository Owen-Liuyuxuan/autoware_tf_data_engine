#!/usr/bin/python3
"""This module defines how we step over a rosbag and collect the data"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from typing import Dict, List, Optional
import os

from autoware_perception_msgs.msg import PredictedObjects, TrafficLightGroupArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
import pygame
import numpy as np
from typing import Dict, List, Optional
import os

class Colors:
    WHITE = (255, 255, 255)
    BLACK = (0, 0, 0)
    RED = (255, 0, 0)
    GREEN = (0, 255, 0)
    BLUE = (0, 0, 255)
    YELLOW = (255, 255, 0)
    GRAY = (128, 128, 128)

class SceneVisualizer:
    def __init__(self, window_size=(1200, 1200), pixels_per_meter=4):
        pygame.init()
        self.window_size = window_size
        self.pixels_per_meter = pixels_per_meter  # Scale factor
        self.screen = pygame.display.set_mode(window_size)
        pygame.display.set_caption("Driving Scene Visualization")
        
        # Fonts
        self.font = pygame.font.Font(None, 24)
        
        # Camera/view parameters
        self.offset_x = window_size[0] // 2
        self.offset_y = window_size[1] // 2
        
    def world_to_screen(self, x, y):
        """Convert world coordinates to screen coordinates"""
        screen_x = int(x * self.pixels_per_meter + self.offset_x)
        screen_y = int(-y * self.pixels_per_meter + self.offset_y)  # Flip y-axis
        return (screen_x, screen_y)

    def draw_polygon(self, points, color, width=1):
        """Draw a polygon from world coordinates"""
        screen_points = [self.world_to_screen(x, y) for x, y in points[:, :2]]
        if len(screen_points) > 2:
            pygame.draw.polygon(self.screen, color, screen_points, width)

    def draw_arrow(self, start_pos, velocity, color=Colors.YELLOW):
        """Draw an arrow representing velocity"""
        start_screen = self.world_to_screen(start_pos[0], start_pos[1])
        end_pos = start_pos[0:2] + velocity[:2] * 2.0  # 2 seconds prediction
        end_screen = self.world_to_screen(end_pos[0], end_pos[1])
        
        if np.linalg.norm(velocity[:2]) > 0.1:
            pygame.draw.line(self.screen, color, start_screen, end_screen, 2)
            # Draw arrowhead
            angle = np.arctan2(end_screen[1]-start_screen[1], end_screen[0]-start_screen[0])
            head_length = 10
            pygame.draw.polygon(self.screen, color, [
                end_screen,
                (end_screen[0] - head_length * np.cos(angle - np.pi/6),
                 end_screen[1] - head_length * np.sin(angle - np.pi/6)),
                (end_screen[0] - head_length * np.cos(angle + np.pi/6),
                 end_screen[1] - head_length * np.sin(angle + np.pi/6))
            ])

    def render_frame(self, data_dict: Dict):
        """Render a single frame"""
        self.screen.fill(Colors.WHITE)
        
        # Center view on ego vehicle
        ego_pos = np.array(data_dict["history_trajectories"]["transforms"][-1][:3, 3])
        self.offset_x = self.window_size[0]//2 - int(ego_pos[0] * self.pixels_per_meter)
        self.offset_y = self.window_size[1]//2 + int(ego_pos[1] * self.pixels_per_meter)

        # Draw map elements
        if "map_elements" in data_dict:
            self._draw_map_elements(data_dict["map_elements"], data_dict.get("routes", []))

        # Draw ego trajectory
        self._draw_ego_trajectory(data_dict["history_trajectories"], data_dict["future_trajectories"])

        # Draw objects
        self._draw_objects(data_dict["objects"])

        # Draw frame information
        self._draw_info(data_dict)

        pygame.display.flip()

    def _draw_map_elements(self, map_elements: List, routes: List):
        """Draw map elements"""
        for element in map_elements:
            # if element.attributes["subtype"] == "road":
            color = Colors.GREEN if element.id in routes else Colors.GRAY
            width = 2 if element.id in routes else 1
            
            # Draw left boundary
            points = np.array([(p.x, p.y) for p in element.leftBound])
            screen_points = [self.world_to_screen(x, y) for x, y in points]
            if len(screen_points) > 1:
                pygame.draw.lines(self.screen, color, False, screen_points, width)
            
            # Draw right boundary
            points = np.array([(p.x, p.y) for p in element.rightBound])
            screen_points = [self.world_to_screen(x, y) for x, y in points]
            if len(screen_points) > 1:
                pygame.draw.lines(self.screen, color, False, screen_points, width)

    def _draw_ego_trajectory(self, history: Dict, future: Dict):
        """Draw ego vehicle trajectory"""
        # Draw history
        history_poses = np.array([t[:3, 3] for t in history["transforms"]])
        points = [self.world_to_screen(x, y) for x, y in history_poses[:, :2]]
        if len(points) > 1:
            pygame.draw.lines(self.screen, Colors.BLUE, False, points, 3)

        # Draw future
        future_poses = np.array([t[:3, 3] for t in future["transforms"]])
        points = [self.world_to_screen(x, y) for x, y in future_poses[:, :2]]
        if len(points) > 1:
            pygame.draw.lines(self.screen, Colors.RED, False, points, 2)

        # Draw current position
        current_pos = history_poses[-1]
        center = self.world_to_screen(current_pos[0], current_pos[1])
        pygame.draw.circle(self.screen, Colors.BLACK, center, 5)

    def _draw_objects(self, objects: List):
        """Draw objects with footprints and velocity vectors"""
        for obj in objects:
            # Draw footprint
            footprint = np.array(obj["global_footprint"])
            self.draw_polygon(footprint, Colors.RED, 2)
            
            # Draw velocity vector
            center = np.mean(footprint, axis=0)
            velocity = np.array(obj["velocity"])
            self.draw_arrow(center, velocity)
            
            # Draw label
            speed = np.linalg.norm(velocity[:2])
            label = f'{obj["type"]} {speed:.1f}m/s'
            text = self.font.render(label, True, Colors.BLACK)
            screen_pos = self.world_to_screen(center[0], center[1])
            self.screen.blit(text, (screen_pos[0]-text.get_width()//2, screen_pos[1]-30))

    def _draw_info(self, data_dict: Dict):
        """Draw frame information"""
        info_text = f"Frame: {data_dict['frame']}"
        if "timestamp" in data_dict:
            info_text += f" | Time: {data_dict['timestamp']:.2f}s"
        text = self.font.render(info_text, True, Colors.BLACK)
        self.screen.blit(text, (10, 10))

    def save_frame(self, filepath):
        """Save current frame to file"""
        pygame.image.save(self.screen, filepath)

    def close(self):
        """Clean up pygame"""
        pygame.quit()

class BaseStepEngine:
    def __init__(
        self,
        object_tracker,
        map_preprocessor,
        trace_back_step=10,
        look_ahead_steps=30,
        key_frame_step=5,
        enable_visualization=False,
        visualization_path=None,
        **args,
    ):
        self.object_tracker = object_tracker
        self.map_preprocessor = map_preprocessor

        self.trace_back_step = trace_back_step
        self.look_ahead_steps = look_ahead_steps
        self.key_frame_step = key_frame_step

        ## the current steps we utilize the ego position as current_step
        self.current_step = -1
        self.current_key_frame = -1

        # Visualization settings
        self.enable_visualization = enable_visualization
        self.visualization_path = visualization_path
        if enable_visualization and visualization_path:
            # os.makedirs(visualization_path, exist_ok=True)
            self.visualizer = SceneVisualizer()
        else:
            self.visualizer = None

    def process_objects(self, msg: PredictedObjects):
        """Base Step Engine focuses on ego steps"""
        self.object_tracker.step_objects(msg, self.current_step)

    def activate_key_frame(self):
        """Base Step Engine focuses on ego steps"""
        self.current_key_frame = self.current_step - self.look_ahead_steps
        tracker_outputs = self.object_tracker.step_key_frame(self.current_key_frame)

        data_dict = {
            "frame": self.current_key_frame,
            "objects": tracker_outputs["objects"],
            "history_trajectories": tracker_outputs["ego_history"],
            "future_trajectories": tracker_outputs["ego_future"],
            "history_trajectories_transform_list": [
                tracker_outputs["ego_history"]["transforms"][i].tolist()
                for i in range(len(tracker_outputs["ego_history"]["transforms"]))
            ],
            "history_trajectories_speed_list": [
                tracker_outputs["ego_history"]["velocities"][i][0]
                for i in range(len(tracker_outputs["ego_history"]["velocities"]))
            ],
            "future_trajectories_transform_list": [
                tracker_outputs["ego_future"]["transforms"][i].tolist()
                for i in range(len(tracker_outputs["ego_future"]["transforms"]))
            ],
            "future_trajectories_speed_list": [
                tracker_outputs["ego_future"]["velocities"][i][0]
                for i in range(len(tracker_outputs["ego_future"]["velocities"]))
            ]
        }

        return data_dict

    def process_ego(self, msg: Odometry):
        self.current_step += 1
        self.object_tracker.step_ego_pose(msg)

        if self.current_step % self.key_frame_step == 0 and self.current_step > (
            self.trace_back_step + self.look_ahead_steps
        ):
            data_dict = {}
            
            data_dict.update(self.activate_key_frame())

            pose: Pose = msg.pose.pose
            map_data = self.map_preprocessor.fetch_local_map(pose)
            data_dict.update(map_data)
            if self.enable_visualization:
                self.visualize_frame(data_dict)

            return data_dict

    def visualize_frame(self, data_dict: Dict):
        """Visualize the current frame"""
        if not self.visualizer:
            return

        self.visualizer.render_frame(data_dict)
        
        if self.visualization_path:
            self.visualizer.save_frame(
                os.path.join(
                    self.visualization_path,
                    f"frame_{self.current_key_frame:06d}.png"
                )
            )
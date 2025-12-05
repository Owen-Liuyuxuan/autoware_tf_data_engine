#!/usr/bin/python3
"""This module handles object detection processing from ROSBags
Objects are processed as current frame detections without historical tracking
"""
from queue import Queue
import numpy as np
from typing import Dict, List, Optional

from autoware_perception_msgs.msg import TrackedObjects, TrackedObject, Shape
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Transform, Twist, Polygon, Point32

def create_transform_matrix(position, orientation) -> np.ndarray:
    """Convert position and quaternion to 4x4 transform matrix"""
    from scipy.spatial.transform import Rotation
    
    # Create rotation matrix from quaternion
    r = Rotation.from_quat([
        orientation.x, 
        orientation.y, 
        orientation.z, 
        orientation.w
    ])
    rotation_matrix = r.as_matrix()
    
    # Create 4x4 transform matrix
    transform = np.eye(4)
    transform[:3, :3] = rotation_matrix
    transform[:3, 3] = [position.x, position.y, position.z]
    
    return transform

def extract_velocity_vector(twist: Twist) -> np.ndarray:
    """Extract 3D velocity vector from Twist message"""
    return np.array([
        twist.linear.x,
        twist.linear.y, 
        twist.linear.z
    ])

def extract_object_footprint(shape: Shape) -> np.ndarray:
    """Extract object footprint from shape message"""
    if shape.type == Shape.BOUNDING_BOX:
        # Convert box to polygon points
        length = shape.dimensions.x
        width = shape.dimensions.y
        
        # Create box corners in local coordinates
        corners = np.array([
            [length/2, width/2, 0],
            [length/2, -width/2, 0],
            [-length/2, -width/2, 0],
            [-length/2, width/2, 0]
        ])
        return corners
    elif shape.type == Shape.POLYGON:
        # Convert polygon points to numpy array
        points = []
        for point in shape.footprint.points:
            points.append([point.x, point.y, 0])
        return np.array(points)
    else:
        # Default to simple point
        return np.array([[0, 0, 0]])

class CurrentFrameObject:
    """Represents a single object in current frame"""
    def __init__(self, obj: TrackedObject):
        # Basic properties
        self.uuid = ''.join([chr(x) for x in obj.object_id.uuid])
        self.object_type = obj.classification[0].label if obj.classification else "UNKNOWN"
        
        # Pose and motion
        self.transform = create_transform_matrix(
            obj.kinematics.initial_pose_with_covariance.pose.position,
            obj.kinematics.initial_pose_with_covariance.pose.orientation
        )
        self.velocity = extract_velocity_vector(
            obj.kinematics.initial_twist_with_covariance.twist
        )

        ## The velocity is in the local frame of the object, so we need to transform it to the global frame
        self.velocity = self.transform[:3, :3] @ self.velocity

        
        # Shape information
        self.footprint = extract_object_footprint(obj.shape)
        # Transform footprint to global coordinates
        self.global_footprint = self._transform_footprint()
        
    def _transform_footprint(self) -> np.ndarray:
        """Transform footprint points to global coordinates"""
        # Add homogeneous coordinate
        points_h = np.hstack([self.footprint, np.ones((len(self.footprint), 1))])
        # Transform points
        global_points_h = points_h @ self.transform.T
        # Return 3D points
        return global_points_h[:, :3]

class BaseTracker:
    def __init__(self, trace_back_step=10, look_ahead_steps=80):
        self.trace_back_step = trace_back_step
        self.look_ahead_steps = look_ahead_steps
        self.buffer_size = trace_back_step + look_ahead_steps
        
        # Store all timestamped data (new compact format)
        # Each entry: {"step": int, "timestamp": float, "data": ...}
        self.ego_states: List[Dict] = []  # All ego states with timestamps
        self.object_detections: List[Dict] = []  # All object detections with timestamps
        
        # Current step counter
        self.current_step = -1
        self.current_timestamp: Optional[float] = None
        
        # Temporary buffers for operation modes and vehicle states (for backward compatibility)
        self.operation_modes: Queue = Queue(maxsize=self.buffer_size)
        self.vehicle_states: Queue = Queue(maxsize=self.buffer_size)

    def step_operation_mode(self, msg):
        # constants for mode
        # uint8 UNKNOWN = 0
        # uint8 STOP = 1
        # uint8 AUTONOMOUS = 2
        # uint8 LOCAL = 3
        # uint8 REMOTE = 4
        if self.operation_modes.full():
            self.operation_modes.get()
        self.operation_modes.put(msg.mode)
        

    def step_vehicle_state(self, msg):
        # constants for mode
        # uint8 NO_COMMAND = 0
        # uint8 AUTONOMOUS = 1
        # uint8 AUTONOMOUS_STEER_ONLY = 2
        # uint8 AUTONOMOUS_VELOCITY_ONLY = 3
        # uint8 MANUAL = 4
        # uint8 DISENGAGED = 5
        # uint8 NOT_READY = 6
        if self.vehicle_states.full():
            self.vehicle_states.get()
        self.vehicle_states.put(msg.mode)

    def step_objects(self, msg: TrackedObjects, current_step=None):
        """Process objects in current frame and store with timestamp"""
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.current_timestamp = timestamp
        
        # Use current_step if provided, otherwise use self.current_step
        step = current_step if current_step is not None else self.current_step
        
        # Process new objects
        objects_data = []
        for tracked_obj in msg.objects:
            obj = CurrentFrameObject(tracked_obj)
            objects_data.append({
                "id": obj.uuid,
                "type": obj.object_type,
                "transform": obj.transform.tolist(),  # Already in map frame
                "velocity": obj.velocity.tolist(),  # Already in map frame
                "global_footprint": obj.global_footprint.tolist()  # Already in map frame
            })
        
        # Store with timestamp and step
        self.object_detections.append({
            "step": step,
            "timestamp": timestamp,
            "objects": objects_data
        })

    def step_ego_pose(self, msg: Odometry):
        """Update ego vehicle state and store with timestamp"""
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.current_step += 1
        
        # Create transform matrix (in map frame)
        transform = create_transform_matrix(
            msg.pose.pose.position,
            msg.pose.pose.orientation
        )
        
        # Extract velocity (in map frame - already global from Odometry)
        velocity = extract_velocity_vector(msg.twist.twist)
        
        # Get operation mode and vehicle status from queues if available
        operation_mode = None
        vehicle_status = None
        if not self.operation_modes.empty():
            # Get latest operation mode
            operation_modes_list = list(self.operation_modes.queue)
            if operation_modes_list:
                operation_mode = operation_modes_list[-1]
        if not self.vehicle_states.empty():
            # Get latest vehicle status
            vehicle_states_list = list(self.vehicle_states.queue)
            if vehicle_states_list:
                vehicle_status = vehicle_states_list[-1]
        
        # Store ego state with timestamp
        self.ego_states.append({
            "step": self.current_step,
            "timestamp": timestamp,
            "transform": transform.tolist(),  # In map frame
            "velocity": velocity.tolist(),  # In map frame
            "operation_mode": operation_mode,
            "vehicle_status": vehicle_status
        })

    def get_ego_state_at_step(self, step: int) -> Optional[Dict]:
        """Get ego state at a specific step"""
        for state in self.ego_states:
            if state["step"] == step:
                return state
        return None
    
    def get_objects_at_step(self, step: int) -> List[Dict]:
        """Get object detections at a specific step"""
        for detection in self.object_detections:
            if detection["step"] == step:
                return detection["objects"]
        return []
    
    def get_all_timestamped_data(self) -> Dict:
        """Get all timestamped data for saving to JSON"""
        return {
            "ego_states": self.ego_states,
            "object_detections": self.object_detections
        }
    
    def step_key_frame(self, keyframe_step):
        """Generate output for key frame (backward compatibility method)
        
        This method is kept for backward compatibility but now extracts
        data from timestamped storage instead of queues.
        """
        # Find ego state at keyframe
        keyframe_ego = self.get_ego_state_at_step(keyframe_step)
        if keyframe_ego is None:
            return None
        
        # Extract history and future ego states
        history_start = max(0, keyframe_step - self.trace_back_step)
        future_end = min(len(self.ego_states), keyframe_step + self.look_ahead_steps)
        
        ego_history = []
        ego_future = []
        
        for i in range(history_start, keyframe_step):
            if i < len(self.ego_states):
                ego_history.append(self.ego_states[i])
        
        for i in range(keyframe_step + 1, future_end):
            if i < len(self.ego_states):
                ego_future.append(self.ego_states[i])
        
        # Get objects at keyframe step
        objects_at_keyframe = self.get_objects_at_step(keyframe_step)
        
        # Filter objects by distance (within 80m of ego at keyframe)
        keyframe_transform = np.array(keyframe_ego["transform"])
        ego_xyz = keyframe_transform[:3, 3]
        filtered_objects = []
        for obj in objects_at_keyframe:
            obj_transform = np.array(obj["transform"])
            obj_xyz = obj_transform[:3, 3]
            distance = np.linalg.norm(ego_xyz - obj_xyz)
            if distance <= 80:
                filtered_objects.append(obj)
        
        # Format for backward compatibility
        return {
            "timestamp": keyframe_ego["timestamp"],
            "objects": filtered_objects,
            "ego_history": {
                "transforms": [state["transform"] for state in ego_history],
                "velocities": [state["velocity"] for state in ego_history],
                "timestamps": [state["timestamp"] for state in ego_history],
                "operational_modes": [state.get("operation_mode") for state in ego_history],
                "vehicle_statuses": [state.get("vehicle_status") for state in ego_history]
            },
            "ego_future": {
                "transforms": [state["transform"] for state in ego_future],
                "velocities": [state["velocity"] for state in ego_future],
                "timestamps": [state["timestamp"] for state in ego_future],
                "operational_modes": [state.get("operation_mode") for state in ego_future],
                "vehicle_statuses": [state.get("vehicle_status") for state in ego_future]
            }
        }
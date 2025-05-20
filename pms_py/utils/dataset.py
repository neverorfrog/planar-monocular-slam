from dataclasses import dataclass, field
from typing import List
import numpy as np
from pms_py.utils.math import Pose2D, Pose3D

@dataclass
class Camera:
    """A camera in the world"""
    
    camera_matrix: np.ndarray
    """The camera matrix of the camera"""
    
    pose: Pose3D
    """The pose of the camera in the world"""
    
    z_near: float
    z_far: float
    """The near and far clipping planes of the camera"""
    
    width: int
    height: int
    """The width and height of the camera image"""
    

@dataclass
class Landmark:
    """A landmark in the world"""
    position: np.ndarray
    """The position of the landmark in the world"""
    
    id: int
    """The ID of the landmark"""
    
    def __post_init__(self):
        if self.position.shape != (3,):
            raise ValueError(f"Position must be a 3D vector, got {self.position.shape}")
        
@dataclass
class Measurement:
    """A measurement of a landmark"""
    
    seq_number: int
    """The sequence number of the measurement"""
    
    current_id: int
    """The ID of the currently observed landmark"""
    
    actual_id: int
    """The ID of the landmark that was actually observed"""
    
    image_point: np.ndarray
    """The position of the measurement in the image"""

    def __post_init__(self):
        if self.image_point.shape != (2,):
            raise ValueError(f"Image point must be a 2D vector, got {self.image_point.shape}")
        
@dataclass
class TrajPoint:
    """A point in the trajectory"""
    
    id: int
    """The ID of the trajectory point"""
    
    odometry: Pose2D
    """The odometry of the trajectory point"""
    
    ground_truth: Pose2D
    """The ground truth of the trajectory point"""
    

@dataclass
class Dataset:
    camera: Camera
    """The camera used to capture the dataset"""
    
    world: List[Landmark] = field(default_factory=list)
    """A list of landmarks populating the world"""

    trajectory: List[TrajPoint] = field(default_factory=list)
    """A list of trajectory points"""
    
    measurements: List[Measurement] = field(default_factory=list)
    """A list of measurements"""
    
    def __post_init__(self):
        for landmark in self.world:
            if not isinstance(landmark, Landmark):
                raise TypeError(f"Expected Landmark, got {type(landmark)}")
        
        for traj_point in self.trajectory:
            if not isinstance(traj_point, TrajPoint):
                raise TypeError(f"Expected TrajPoint, got {type(traj_point)}")
        
        for measurement in self.measurements:
            if not isinstance(measurement, Measurement):
                raise TypeError(f"Expected Measurement, got {type(measurement)}")
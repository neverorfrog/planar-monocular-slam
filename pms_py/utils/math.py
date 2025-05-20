from dataclasses import dataclass
import numpy as np
from typing import overload

@dataclass
class Pose2D:
    """A 2D pose in the world"""
    
    position: np.ndarray
    """The position of the pose in the world"""
    
    orientation: float
    """The orientation of the pose in the world"""
    
    def __post_init__(self):
        if self.position.shape != (2,):
            raise ValueError(f"Position must be a 2D vector, got {self.position.shape}")
        
        
@dataclass
class Pose3D:
    """A 3D pose in the world"""
    
    position: np.ndarray
    """The position of the pose in the world"""
    
    orientation: np.ndarray
    """The orientation of the pose in the world"""
    
    def __post_init__(self):
        if self.position.shape != (3,):
            raise ValueError(f"Position must be a 3D vector, got {self.position.shape}")
        if self.orientation.shape != (3, 3):
            raise ValueError(f"Orientation must be a 3D rotation matrix, got {self.orientation.shape}")


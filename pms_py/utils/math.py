from dataclasses import dataclass
import numpy as np

@dataclass
class Pose2D:
    """A 2D pose in the world"""
    
    def __init__(self, *args, **kwargs) -> None:
        if len(args) == 2:
            self.position, self.orientation = args
        elif len(args) == 1:
            pose_vector = args[0]
            self.position = pose_vector[:2]
            self.orientation = pose_vector[2]
        else:
            raise ValueError("Invalid arguments")
        if self.position.shape != (2,):
            raise ValueError(f"Position must be a 2D vector, got {self.position.shape}")
    
    position: np.ndarray
    """The position of the pose in the world"""
    
    orientation: float
    """The orientation of the pose in the world"""
    
        
@dataclass
class Pose3D:
    """A 3D pose in the world"""
    
    def __init__(self, *args, **kwargs) -> None:
        if len(args) == 2:
            self.position, self.orientation = args
        elif len(args) == 1:
            homogeneous_matrix = args[0]
            self.position = homogeneous_matrix[:3, 3]
            self.orientation = homogeneous_matrix[:3, :3]
        else:
            raise ValueError("Invalid arguments")
        if self.position.shape != (3,):
            raise ValueError(f"Position must be a 3D vector, got {self.position.shape}")
        if self.orientation.shape != (3, 3):
            raise ValueError(f"Orientation must be a 3D rotation matrix, got {self.orientation.shape}")
     
    position: np.ndarray
    """The position of the pose in the world"""
    
    orientation: np.ndarray
    """The orientation of the pose in the world"""
    

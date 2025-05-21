from dataclasses import dataclass, field
import os
from typing import List
import numpy as np
from pms_py.utils import project_root
from pms_py.utils.math import Pose2D, Pose3D
import re

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
    
    measurements: List[Measurement] = field(default_factory=list)
    

@dataclass
class Dataset:
    camera: Camera
    """The camera used to capture the dataset"""
    
    world: List[Landmark] = field(default_factory=list)
    """A list of landmarks populating the world"""

    trajectory: List[TrajPoint] = field(default_factory=list)
    """A list of trajectory points"""
    
    
    def __post_init__(self):
        for landmark in self.world:
            if not isinstance(landmark, Landmark):
                raise TypeError(f"Expected Landmark, got {type(landmark)}")
        
        for traj_point in self.trajectory:
            if not isinstance(traj_point, TrajPoint):
                raise TypeError(f"Expected TrajPoint, got {type(traj_point)}")
            
            
def load_trajectory(folderpath: str) -> List[TrajPoint]:
    """
    Loads trajectory data from a file.
    Each line in the file should contain:
    ID odom_x odom_y odom_theta gt_x gt_y gt_theta
    """
    
    if not os.path.exists(folderpath):
        raise FileNotFoundError(f"Folder not found: {folderpath}")

    trajectory = _load_trajectory_without_meas(os.path.join(folderpath, "trajectory.dat"))
    for filename in os.listdir(folderpath):
        if re.fullmatch(r"meas-\d{5}\.dat", filename):
            filepath = os.path.join(folderpath, filename)
            _add_measurements(trajectory, filepath)
            
    return trajectory


def _load_trajectory_without_meas(filepath: str) -> List[TrajPoint]:
    trajectory: List[TrajPoint] = []
    
    if not os.path.exists(filepath):
        raise FileNotFoundError(f"File not found: {filepath}")
    
    with open(filepath, 'r') as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) == 7:
                try:
                    traj_id = int(parts[0])
                    odom_x = float(parts[1])
                    odom_y = float(parts[2])
                    odom_theta = float(parts[3])
                    gt_x = float(parts[4])
                    gt_y = float(parts[5])
                    gt_theta = float(parts[6])
                    
                    odometry = Pose2D(np.array([odom_x, odom_y]), odom_theta)
                    ground_truth = Pose2D(np.array([gt_x, gt_y]), gt_theta)

                    traj_point = TrajPoint(traj_id, odometry, ground_truth)
                    trajectory.append(traj_point)
                except ValueError as e:
                    print(f"Skipping line due to parsing error: {line.strip()} - {e}")
            else:
                raise ValueError(f"Invalid line format: {line.strip()}")
    return trajectory

def _add_measurements(trajectory: List[TrajPoint], meas_filepath: str) -> None:
    
    if not os.path.exists(meas_filepath):
        raise FileNotFoundError(f"File not found: {meas_filepath}")
    
    seq_number = -1
    with open(meas_filepath, 'r') as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) == 2 and parts[0] == "seq:" and seq_number == -1:
                seq_number = int(parts[1])
            if len(parts) == 5 and parts[0] == "point":
                try:
                    current_id = int(parts[1])
                    actual_id = int(parts[2])
                    image_point_x = float(parts[3])
                    image_point_y = float(parts[4])

                    image_point = np.array([image_point_x, image_point_y])
                    if seq_number == -1:
                        raise ValueError("Sequence number not found before point data")
                    measurement = Measurement(seq_number, current_id, actual_id, image_point)
                    
                    trajectory[seq_number].measurements.append(measurement)
                except ValueError as e:
                    print(f"Skipping line due to parsing error: {line.strip()} - {e}")
                    


def _load_camera(folderpath: str) -> Camera:
    """
    Loads camera data from a file.
    The file should contain the following lines:
    camera matrix:
    fx 0 cx
    0 fy cy
    0 0 1
    cam_transform:
    r11 r12 r13 tx
    r21 r22 r23 ty
    r31 r32 r33 tz
    0   0   0  1
    z_near: value
    z_far:  value
    width:  value
    height: value
    """
    filepath = os.path.join(folderpath, "camera.dat")

    if not os.path.exists(filepath):
        raise FileNotFoundError(f"File not found: {filepath}")

    params = {}
    with open(filepath, 'r') as f:
        lines = [line.strip() for line in f.readlines()]

    try:
        idx = 0
        # Camera Matrix
        if lines[idx] != "camera matrix:":
            raise ValueError("Missing 'camera matrix:' header")
        idx += 1
        cam_matrix_lines = lines[idx:idx+3]
        if len(cam_matrix_lines) < 3:
            raise ValueError("Insufficient lines for camera matrix")
        params['camera_matrix'] = np.array([list(map(float, line.split())) for line in cam_matrix_lines])
        if params['camera_matrix'].shape != (3,3):
            raise ValueError(f"Camera matrix must be 3x3, got {params['camera_matrix'].shape}")
        idx += 3

        # Camera Transform (Pose)
        if lines[idx] != "cam_transform:":
            raise ValueError("Missing 'cam_transform:' header")
        idx += 1
        cam_transform_lines = lines[idx:idx+4]
        if len(cam_transform_lines) < 4:
            raise ValueError("Insufficient lines for camera transform")
        transform_matrix = np.array([list(map(float, line.split())) for line in cam_transform_lines])
        if transform_matrix.shape != (4,4):
            raise ValueError(f"Camera transform matrix must be 4x4, got {transform_matrix.shape}")
        params['pose'] = Pose3D(transform_matrix) # Assuming Pose3D can be initialized with a 4x4 matrix
        idx += 4
        
        # Scalar parameters
        expected_params = ['z_near', 'z_far', 'width', 'height']
        for param_name in expected_params:
            line = lines[idx]
            if not line.startswith(f"{param_name}:"):
                raise ValueError(f"Expected '{param_name}:' but got '{line}'")
            parts = line.split(':')
            if len(parts) != 2:
                raise ValueError(f"Malformed line for {param_name}: {line}")
            value_str = parts[1].strip()
            try:
                if param_name in ['width', 'height']:
                    params[param_name] = int(value_str)
                else:
                    params[param_name] = float(value_str)
            except ValueError:
                raise ValueError(f"Could not parse value for {param_name} from '{value_str}'")
            idx +=1
            
    except IndexError:
        raise ValueError(f"File {filepath} is incomplete or has an unexpected format.")
    except Exception as e:
        raise ValueError(f"Error parsing {filepath}: {e}")

    return Camera(
        camera_matrix=params['camera_matrix'],
        pose=params['pose'],
        z_near=params['z_near'],
        z_far=params['z_far'],
        width=params['width'],
        height=params['height']
    )
    
    
def _load_world(folderpath: str) -> List[Landmark]:
    """
    Loads world data from a file.
    Each line in the file should contain:
    ID x y z
    """
    
    filepath = os.path.join(folderpath, "world.dat")
    
    if not os.path.exists(filepath):
        raise FileNotFoundError(f"File not found: {filepath}")

    world = []
    with open(filepath, 'r') as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) == 4:
                try:
                    landmark_id = int(parts[0])
                    x = float(parts[1])
                    y = float(parts[2])
                    z = float(parts[3])
                    
                    landmark = Landmark(np.array([x, y, z]), landmark_id)
                    world.append(landmark)
                except ValueError as e:
                    print(f"Skipping line due to parsing error: {line.strip()} - {e}")
            else:
                raise ValueError(f"Invalid line format: {line.strip()}")
                
    return world
    
    
def load_dataset(folderpath: str) -> Dataset:
    """
    Loads a dataset from a folder.
    The folder should contain:
    - trajectory.dat
    - camera.dat
    - meas-xxxxx.dat
    - world.dat
    """
    
    if not os.path.exists(folderpath):
        raise FileNotFoundError(f"Folder not found: {folderpath}")

    trajectory = load_trajectory(folderpath)
    camera = _load_camera(folderpath)
    world = _load_world(folderpath)
    
    return Dataset(camera=camera, world=world, trajectory=trajectory)
    
    
def main():
    data_path = os.path.join(project_root(), "data")
    dataset = load_dataset(data_path)
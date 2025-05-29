import os
from matplotlib import pyplot as plt
import numpy as np
from pms_py.utils import project_root
from pms import Dataset, triangulate, BundleAdjuster, BundleAdjustmentConfig, triangulateRansac
from pms_py.evaluation import initial_guess_error
from pms_py.visualization import plot_landmarks, plot_trajectory
import tqdm

def main():
    # Load dataset
    data_path = os.path.join(project_root(), "data")
    dataset = Dataset(data_path)
    gt_positions = np.array(dataset.getLandmarkPositions())
    
    # Triangulate landmarks for initial guess
    landmarks = triangulateRansac(dataset)
    guessed_positions = np.array([lm.position for lm in landmarks])
    valid_landmarks = np.array([lm.valid for lm in landmarks])
    initial_guess_error(guessed_positions, gt_positions, valid_landmarks)
    
    # Plotting
    odom_traj = dataset.getOdometryPoses3()
    gt_traj = dataset.getGroundTruthPoses3()
    
    # Bundle Adjustment
    ba_config = BundleAdjustmentConfig()
    ba_config.max_iterations = 51
    ba_config.pose_landmark = True
    ba_config.pose_pose = True
    ba = BundleAdjuster(landmarks, dataset, ba_config)
    with tqdm.tqdm(range(ba_config.max_iterations)) as pbar:
        for iteration in pbar:
            stats = ba.performIteration()
            
            pbar.set_description(f"Iter {stats.num_iterations}, Landmark error: {stats.landmark_chi:.4f}")
            
            if(stats.converged):
                pbar.set_postfix(status="Converged")
                break
            
            state = ba.getState()
            pbar.set_postfix(
                pose_err=f"{stats.pose_chi:.4f}",
                landmarks=len(state.landmarks),
                poses=len(state.robot_poses)
            )
            
            if iteration % 10 == 0:
                traj = state.robot_poses
                plot_trajectory(traj, odom_traj, gt_traj)
                plt.legend()
                plt.show()

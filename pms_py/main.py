import os
from matplotlib import pyplot as plt
import numpy as np
from pms_py.utils import project_root
from pms import Dataset, triangulate, BundleAdjuster, BundleAdjustmentConfig
from pms_py.evaluation import initial_guess_error
from pms_py.visualization import plot_landmarks, plot_trajectory
import tqdm

def main():
    # Load dataset
    data_path = os.path.join(project_root(), "data")
    dataset = Dataset(data_path)
    gt_positions = np.array(dataset.getLandmarkPositions())
    
    # Triangulate landmarks for initial guess
    landmarks = triangulate(dataset)
    guessed_positions = np.array([lm.position for lm in landmarks])
    valid_landmarks = np.array([lm.valid for lm in landmarks])
    initial_guess_error(guessed_positions, gt_positions, valid_landmarks)
    
    # Plotting
    odom_traj = dataset.getOdometryPoses3()
    gt_traj = dataset.getGroundTruthPoses3()
    # plot_landmarks(gt_positions, guessed_positions, valid_landmarks)
    plot_trajectory(odom_traj, gt_traj)
    # plt.legend()
    # plt.show()
    
    # Bundle Adjustment
    ba_config = BundleAdjustmentConfig()
    ba_config.max_iterations = 10
    ba = BundleAdjuster(landmarks, dataset, ba_config)
    for iteration in tqdm.tqdm(range(ba_config.max_iterations)):
        stats = ba.performIteration()
        
        print(f"Iteration {stats.num_iterations}")
        print(f"Error: {stats.landmark_chi}")
        
        if(stats.converged):
            print(f"Converged after {stats.num_iterations} iterations")
            break
        
        state = ba.getState()
        
        odom_traj = state.robot_poses
        plot_trajectory(odom_traj, gt_traj)
        plt.legend()
        plt.show()
        # exit(0)
        
        # TODO: Print information about intermadiate optimization results
        # TODO: Plot intermediate results
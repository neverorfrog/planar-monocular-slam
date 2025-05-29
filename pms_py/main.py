import os
from matplotlib import pyplot as plt
import numpy as np
from pms_py.utils import project_root
from pms import Dataset, triangulate, BundleAdjuster, BundleAdjustmentConfig, triangulateRansac
from pms_py.evaluation import compute_pose_error, initial_guess_error, compute_map_error
from pms_py.visualization import plot_landmarks, plot_trajectory
import tqdm

plotting = True

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
    pos_errors = []
    theta_errors = []
    map_errors = []
    
    # Bundle Adjustment
    ba_config = BundleAdjustmentConfig()
    ba_config.max_iterations = 41
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
            
            # Error compution
            map_error = compute_map_error(np.array(state.getLandmarkPositions()), gt_positions, valid_landmarks)
            map_errors.append(map_error)
            pos_error, theta_error = compute_pose_error(state.robot_poses, gt_traj)
            pos_errors.append(pos_error)
            theta_errors.append(theta_error)
            pbar.set_postfix(
                pos_error=f"{pos_error:.4f}",
                theta_error=f"{theta_error:.4f}",
                map_error=f"{map_error:.4f}"
            )
            
            if plotting:
                figure_dir = os.path.join(project_root(), "figures")
                os.makedirs(figure_dir, exist_ok=True)
                
                # Poses
                traj = state.robot_poses
                plot_trajectory(traj, odom_traj, gt_traj)
                plt.savefig(f"{figure_dir}/trajectory_{iteration:03d}.png")
                
                # Landmarks
                landmarks = state.getLandmarkPositions()
                plot_landmarks(gt_positions, np.array(landmarks), valid_landmarks, with_error=True)
                plt.savefig(f"{figure_dir}/landmarks_{iteration:03d}.png")

                if iteration == ba_config.max_iterations - 1 or stats.converged:
                    plt.clf()
                    plt.figure()
                    plt.plot(pos_errors, label="Position Error")
                    plt.plot(theta_errors, label="Orientation Error")
                    plt.xlabel("Iteration")
                    plt.ylabel("Error")
                    plt.legend()
                    plt.savefig(f"{figure_dir}/pose_error_{iteration:03d}.png")
                    
                    plt.clf()
                    plt.figure()
                    plt.plot(map_errors, label="Map Error")
                    plt.xlabel("Iteration")
                    plt.ylabel("Map Error")
                    plt.legend()
                    plt.savefig(f"{figure_dir}/map_error_{iteration:03d}.png")



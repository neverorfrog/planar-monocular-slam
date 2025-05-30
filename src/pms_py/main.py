import json
import os
import numpy as np
from pms_py.utils import project_root
from pms import Dataset, BundleAdjuster, BundleAdjustmentConfig, triangulateRansac
from pms_py.evaluation import initial_guess_error
from pms_py.visualization import (
    plot_errors,
    plot_landmarks,
    plot_trajectory,
)
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
    ba_config.pose_landmark = True
    ba_config.pose_pose = False
    ba_config.tolerance = 1e-3
    ba = BundleAdjuster(landmarks, dataset, ba_config)
    with tqdm.tqdm(range(ba_config.max_iterations)) as pbar:
        for iteration in pbar:
            stats = ba.performIteration()

            pbar.set_description(
                f"Iter {stats.num_iterations}, Landmark error: {stats.landmark_chi:.4f}"
            )

            state = ba.getState()

            # Error compution
            map_errors.append(stats.map_error)
            pos_errors.append(stats.position_error)
            theta_errors.append(stats.orientation_error)
            pbar.set_postfix(
                pos_error=f"{stats.position_error:.4f}",
                theta_error=f"{stats.orientation_error:.4f}",
                map_error=f"{stats.map_error:.4f}",
            )

            if plotting:
                # Poses
                traj = state.robot_poses
                plot_trajectory(traj, odom_traj, gt_traj, iteration)

                # Landmarks
                landmarks = state.getLandmarkPositions()
                plot_landmarks(
                    gt_positions,
                    np.array(landmarks),
                    iteration,
                    valid_landmarks,
                    with_error=True,
                )

                if iteration == ba_config.max_iterations - 1 or stats.converged:
                    plot_errors(pos_errors, theta_errors, map_errors)
                    
            stats_dict = {
                "iteration": iteration,
                "num_iterations": stats.num_iterations,
                "landmark_chi": float(stats.landmark_chi),
                "map_error": float(stats.map_error),
                "position_error": float(stats.position_error),
                "orientation_error": float(stats.orientation_error),
                "converged": bool(stats.converged)
            }
            
            stats_dir = os.path.join(project_root(), "docs", "stats")
            os.makedirs(stats_dir, exist_ok=True)
            with open(os.path.join(stats_dir, f"stats_{iteration:03d}.json"), "w") as f:
                json.dump(stats_dict, f, indent=2)

            if stats.converged:
                break
            

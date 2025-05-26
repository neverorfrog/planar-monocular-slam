import os
import numpy as np
import matplotlib.pyplot as plt
from pms_py.utils import project_root
from pms import Dataset, triangulate, Solution

def plot_trajectory(dataset: Dataset) -> None:
    """
    Plot the trajectory of the dataset.
    """
    odom_traj = dataset.getOdometryPoses()
    gt_traj = dataset.getGroundTruthPoses()

    plt.plot(
        [odom.translation[0] for odom in odom_traj],
        [odom.translation[1] for odom in odom_traj],
        "ro",
        label="odometry",
    )
    plt.plot(
        [gt.translation[0] for gt in gt_traj],
        [gt.translation[1] for gt in gt_traj],
        "go",
        label="ground truth",
    )


def plot_landmarks(landmarks: np.ndarray, color: str = "r") -> None:
    """
    Plot the dataset
    """
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.scatter(
        landmarks[:, 0],
        landmarks[:, 1],
        landmarks[:, 2],
        c=color,
        marker="o",
        label="landmarks",
    )
    ax.set_title("Landmarks")
    
    
def plot_initial_guess(gt: np.ndarray, guess: np.ndarray, successes: np.ndarray) -> None:
    """
    Plot the initial guess of the landmarks.
    """
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.scatter(
        gt[:, 0],
        gt[:, 1],
        gt[:, 2],
        c="g",
        marker="o",
        label="ground truth",
    )
    ax.scatter(
        guess[:, 0],
        guess[:, 1],
        guess[:, 2],
        c="b",
        marker="o",
        label="initial guess",
    )
    ax.set_title("Initial Guess vs Ground Truth")
    
    for i in range(len(gt)):
        if successes[i]:
            ax.plot(
                [gt[i, 0], guess[i, 0]],  # X coordinates (gt_x, guess_x)
                [gt[i, 1], guess[i, 1]],  # Y coordinates (gt_y, guess_y)
                [gt[i, 2], guess[i, 2]],  # Z coordinates (gt_z, guess_z)
                color="r",  # Error lines in red
                linestyle="--",
                linewidth=0.7,
                label="error" if i == 0 else None,  # Add label only once for the legend
            )


def main():
    # Load dataset
    data_path = os.path.join(project_root(), "data")
    dataset = Dataset(data_path)
    gt_positions = np.array(dataset.getLandmarkPositions())
    
    # triangulate landmarks for initial guess
    solution = Solution(dataset)
    triangulate(solution, dataset)
    guessed_landmarks = solution.world
    
    # Logging and plotting
    guessed_positions = np.array([lm.position for lm in guessed_landmarks])
    valid_landmarks = np.array([lm.valid for lm in guessed_landmarks])
    plot_initial_guess(gt_positions, guessed_positions, valid_landmarks)
    plot_trajectory(dataset)
    plt.legend()
    plt.show()
    
    valid_mask = valid_landmarks
    differences = gt_positions[valid_mask] - guessed_positions[valid_mask]
    
    # 1. Root Mean Square Error (RMSE)
    squared_errors = np.sum(differences**2, axis=1)  # Sum of squared differences per landmark
    rmse = np.sqrt(np.mean(squared_errors))
    print(f"Root Mean Square Error (RMSE): {rmse:.4f}")
    
    # 2. Mean Absolute Error (MAE)
    absolute_errors = np.abs(differences)  # Absolute differences for each coordinate
    mae_per_coordinate = np.mean(absolute_errors, axis=0)  # Mean per coordinate (x,y,z)
    mae_overall = np.mean(absolute_errors)  # Overall MAE across all coordinates
    print(f"Mean Absolute Error (MAE): {mae_overall:.4f}")
    print(f"  MAE per coordinate: X={mae_per_coordinate[0]:.4f}, Y={mae_per_coordinate[1]:.4f}, Z={mae_per_coordinate[2]:.4f}")
    
    # 3. Mean Euclidean Distance
    euclidean_distances = np.sqrt(np.sum(differences**2, axis=1))  # Euclidean distance per landmark
    mean_euclidean_distance = np.mean(euclidean_distances)
    median_euclidean_distance = np.median(euclidean_distances)
    print(f"Mean Euclidean Distance: {mean_euclidean_distance:.4f}")
    print(f"Median Euclidean Distance: {median_euclidean_distance:.4f}")
    
    # 4. Component-wise statistics
    print("\nComponent-wise errors:")
    for i, axis in enumerate(['X', 'Y', 'Z']):
        component_errors = differences[:, i]
        print(f"  {axis}-axis: Mean={np.mean(component_errors):.4f}, Std={np.std(component_errors):.4f}, "
              f"Min={np.min(component_errors):.4f}, Max={np.max(component_errors):.4f}")
    
    # Count of valid landmarks
    num_valid = np.sum(valid_landmarks)
    total_landmarks = len(valid_landmarks)
    print(f"\nValid landmarks: {num_valid}/{total_landmarks} ({num_valid/total_landmarks*100:.1f}%)")



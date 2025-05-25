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
        print(f"Landmark {i}: {'valid' if successes[i] else 'invalid'}")
        print(f"Ground Truth: {gt[i]}")
        print(f"Guess: {guess[i]}")
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
    error = np.linalg.norm(gt_positions[valid_landmarks] - guessed_positions[valid_landmarks], axis=1)
    plot_initial_guess(gt_positions, guessed_positions, valid_landmarks)
    plot_trajectory(dataset)
    plt.legend()
    plt.show()
    print(f"Mean error: {np.mean(error):.4f}")
    
    

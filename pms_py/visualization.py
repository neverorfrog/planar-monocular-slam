import numpy as np
import matplotlib.pyplot as plt
from pms import Pose3

def plot_trajectory(odom_traj: list[Pose3], gt_traj: list[Pose3]) -> None:
    """
    Plot the trajectory of the dataset.
    """
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

def plot_landmarks(gt: np.ndarray, guess: np.ndarray, successes: np.ndarray, with_error: bool = False) -> None:
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
    
    if with_error:
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
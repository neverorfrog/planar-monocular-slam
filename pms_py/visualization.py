import numpy as np
import matplotlib.pyplot as plt
from pms import Pose3

def plot_trajectory(traj: list[Pose3], odom_traj: list[Pose3], gt_traj: list[Pose3]) -> None:
    """
    Plot the trajectory of the dataset.
    """
    plt.clf()
    plt.figure()
    plt.axis("equal")
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
    
    plt.plot(
        [pose.translation[0] for pose in traj],
        [pose.translation[1] for pose in traj],
        "bo",
        label="estimated trajectory",
    )
    plt.legend()
    plt.title("Trajectory Comparison")
    

def plot_landmarks(gt: np.ndarray, guess: np.ndarray, successes: np.ndarray = None, with_error: bool = False) -> None:
    """
    Plot the initial guess of the landmarks.
    """
    plt.clf()
    plt.figure()
    plt.xlim(-13, 13)
    plt.ylim(-13, 13)
    plt.scatter(
        gt[:, 0],
        gt[:, 1],
        c="g",
        marker="o",
        label="ground truth",
    )
    plt.scatter(
        guess[:, 0],
        guess[:, 1],
        c="b",
        marker="o",
        label="estimate",
    )
    plt.title("Estimated Map vs Ground Truth Map")
    plt.legend()

    if with_error:
        for i in range(len(gt)):
            if successes[i]:
                plt.plot(
                    [gt[i, 0], guess[i, 0]],  # X coordinates (gt_x, guess_x)
                    [gt[i, 1], guess[i, 1]],  # Y coordinates (gt_y, guess_y)
                    color="r",  # Error lines in red
                    linestyle="--",
                    linewidth=0.7,
                    label="error" if i == 0 else None,  # Add label only once for the legend
                )
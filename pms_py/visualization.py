import os
import numpy as np
import matplotlib.pyplot as plt
from pms import Pose3
from pms_py.utils import project_root


def plot_trajectory(
    traj: list[Pose3],
    odom_traj: list[Pose3],
    gt_traj: list[Pose3],
    iteration: int,
    save: bool = True,
) -> None:
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
    if save:
        figure_dir = os.path.join(project_root(), "figures")
        os.makedirs(figure_dir, exist_ok=True)
        plt.savefig(f"{figure_dir}/trajectory_{iteration:03d}.png")
    plt.close()


def plot_landmarks(
    gt: np.ndarray,
    guess: np.ndarray,
    iteration: int,
    successes: np.ndarray = None,
    with_error: bool = False,
    save: bool = True,
) -> None:
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
                    label="error"
                    if i == 0
                    else None,  # Add label only once for the legend
                )

    if save:
        figure_dir = os.path.join(project_root(), "figures")
        os.makedirs(figure_dir, exist_ok=True)
        plt.savefig(f"{figure_dir}/landmarks_{iteration:03d}.png")
    plt.close()


def plot_errors(
    pos_errors: list[float],
    theta_errors: list[float],
    map_errors: list[float],
    save: bool = True,
) -> None:
    """
    Plot the trajectory error.
    """
    plt.clf()
    plt.figure()
    plt.plot(pos_errors, label="Position Error")
    plt.plot(theta_errors, label="Orientation Error")
    plt.plot(map_errors, label="Map Error")
    plt.xlabel("Iteration")
    plt.ylabel("Error")
    plt.title("Errors Over Iterations")
    plt.legend()

    if save:
        figure_dir = os.path.join(project_root(), "figures")
        os.makedirs(figure_dir, exist_ok=True)
        plt.savefig(f"{figure_dir}/errors.png")
    plt.close()

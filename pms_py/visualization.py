import os
import numpy as np
import matplotlib.pyplot as plt
from pms_py.utils import project_root
from pms import Dataset


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
    plt.legend()
    plt.show()


def plot(dataset: Dataset, with_trajectory: bool = True) -> None:
    """
    Plot the dataset
    """
    landmarks = np.array(dataset.getLandmarkPositions())
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(landmarks[:, 0], landmarks[:, 1], landmarks[:, 2], c="r", marker="o", label="landmarks")
    ax.set_title("Landmarks")
    
    if with_trajectory:
        plot_trajectory(dataset)
    
    plt.show()


def main():
    data_path = os.path.join(project_root(), "data")
    dataset = Dataset(data_path)
    plot(dataset)

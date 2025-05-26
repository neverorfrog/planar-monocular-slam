import os
from matplotlib import pyplot as plt
import numpy as np
from pms_py.utils import project_root
from pms import Dataset, triangulate, Solution, bundleAdjust
from pms_py.evaluation import initial_guess_error
from pms_py.visualization import plot_landmarks, plot_trajectory

def main():
    # Load dataset
    data_path = os.path.join(project_root(), "data")
    dataset = Dataset(data_path)
    gt_positions = np.array(dataset.getLandmarkPositions())
    
    # triangulate landmarks for initial guess
    solution = Solution(dataset)
    triangulate(solution, dataset)
    guessed_landmarks = solution.world
    guessed_positions = np.array([lm.position for lm in guessed_landmarks])
    valid_landmarks = np.array([lm.valid for lm in guessed_landmarks])
    initial_guess_error(guessed_positions, gt_positions, valid_landmarks)
    
    # Plotting
    plot_landmarks(gt_positions, guessed_positions, valid_landmarks)
    plot_trajectory(dataset)
    plt.legend()
    # plt.show()
    
    # Bundle adjustment
    bundleAdjust(solution, dataset)
    
    
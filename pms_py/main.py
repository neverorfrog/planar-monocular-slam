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
    plot_landmarks(gt_positions, guessed_positions, valid_landmarks)
    plot_trajectory(dataset)
    plt.legend()
    plt.show()
    
    # Bundle Adjustment
    # ba_config = BundleAdjustmentConfig()
    # ba = BundleAdjuster(landmarks, dataset, ba_config)
    # for iteration in tqdm.tqdm(range(ba_config.max_iterations)):
    #     ba.performIteration()
        
    #     # TODO: Print information about intermadiate optimization results
    #     # TODO: Plot intermediate results
    #     print(ba.getStats())

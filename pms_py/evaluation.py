import numpy as np


def initial_guess_error(
    guessed_positions: np.ndarray, gt_positions: np.ndarray, valid_landmarks: np.ndarray
) -> None:
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
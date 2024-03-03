# Planar Monocular SLAM

## Goal

- Get an estimate of points in the world from images correspondences (use the points id for data association) using the noisy odometry, you can achieve this in two ways:
  1. Use always pair of images where correspondence appear, find correspondences, triangulate (you will have duplication of 3D points due to noise in image features, if this is the case merge same 3D points in position)
  2. use all the correspondences from all the images, to find the set of world points that minimizes the distance among all features directions
- once you have an initial estimate of 3D points, SE(2) poses (your odometry), you can perform Bundle Adjustment, see Total Least Squares but remember that your motion is planar

## Evaluation

### Evaluation of poses

Evaluate your solution comparing it with the gt using the delta between poses

For example, suppose we these two consecutive poses: T_0, T_1

- compute the relative motion rel_T = inv(T_0)*T_1
- compute the relative gt motion rel_GT = inv(GT_0)*GT_1
- compute the SE(2) error error_T = inv(rel_T)*rel_GT
- rotation part:
        - atan2(error_T(2, 1), error_T(1, 1))
- translation part
  - compute the RMSE on translations error_T(1:3, 4)

### Evaluation of MAP

- compute the whole RMSE

## Hints by the professor

- develop each component independently and use the ground truth for testing
- when everything is working put all together and create your Planar Monocular SLAM
- the better will be the inital guess the better you final estimate after BA

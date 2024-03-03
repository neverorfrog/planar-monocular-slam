# Dataset

Dataset is composed by the following files:

- world.dat (you need this only for evaluation) It contains information about the map.
  Every row contains: LANDMARK_ID (1) POSITION (2:4)

- camera.dat
  It contains information about the camera used to gather data:
  - camera matrix
  - cam_transform: pose of the camera w.r.t. robot
  - z_near/z_far how close/far the camera can perceive stuff
  - width/height of images

- trajectory.dat
  POSE_ID (1) ODOMETRY_POSE (2:4) GROUNDTRUTH_POSE (5:7)
  the ODOMETRY_POSE is obtained by adding Gaussian Noise (0; 0.001) to the actual robot commands

- meas-XXXX.dat
  Every measurement contains a sequence number, groundtruth (of the robot) and odometry pose and measurement information:
  - point POINT_ID_CURRENT_MESUREMENT (1) ACTUAL_POINT_ID (2) IMAGE_POINT (3:4)

  The Image_Point represents the pair [col;row] where the landmark is observed in the image

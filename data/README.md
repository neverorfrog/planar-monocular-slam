# Dataset

Dataset is composed by the following files:

- world.dat (you need this only for evaluation) It contains information about the map.
  Every row contains: LANDMARK_ID (0) POSITION (1:3)

- camera.dat
  It contains information about the camera used to gather data:
  - camera matrix
  - cam_transform: pose of the camera w.r.t. robot
  - z_near/z_far how close/far the camera can perceive stuff
  - width/height of images

- trajectory.dat
  POSE_ID (0) ODOMETRY_POSE (1:3) GROUNDTRUTH_POSE (4:6)
  the ODOMETRY_POSE is obtained by adding Gaussian Noise (0; 0.001) to the actual robot commands

- meas-XXXX.dat
  Every measurement contains a sequence number, groundtruth (of the robot) and odometry pose and measurement information:
  - point POINT_ID_CURRENT_MESUREMENT (0) ACTUAL_POINT_ID (1) IMAGE_POINT (2:3)
  - Image_Point represents the pair [col;row] where the landmark is observed in the image

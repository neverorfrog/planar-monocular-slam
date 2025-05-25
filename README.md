## Landmark Triangulation

To obtain an initial 3D guess for the position of each landmark, we employ a triangulation method based on multiple 2D observations of the landmark from different camera poses. This process uses the Direct Linear Transform (DLT) algorithm, as detailed in [1].

### 1. Problem Formulation

Given:
*   A set of $M \ge 2$ 2D image projections $\mathbf{x}_j = [u_j, v_j]^T$ for a single landmark, observed from $M$ different camera views.
*   The camera intrinsic matrix $K$ (3x3).
*   The camera extrinsics relative to the robot: $T_{RC}$ (a 4x4 transformation from camera frame to robot frame).
*   The robot's odometry pose for each view $j$: $T_{WR}^{(j)}$ (a 4x4 transformation from robot frame to world frame).

We want to find the 3D position of the landmark in homogeneous world coordinates, $\mathbf{X}_w = [X, Y, Z, 1]^T$.

### 2. Projection Equation

For each view $j$, the projection of the 3D world point $\mathbf{X}_w$ onto the image plane $\mathbf{x}_{img, j} = [u_j, v_j, 1]^T$ is described by:

$s_j \mathbf{x}_{img, j} = P^{(j)} \mathbf{X}_w$

where:
*   $s_j$ is a non-zero scale factor for view $j$.
*   $P^{(j)}$ is the $3 \times 4$ camera projection matrix for view $j$.

### 3. Computing the Projection Matrix $P^{(j)}$

For each view $j$:
1.  Calculate the camera's pose in the world: $T_{WC}^{(j)} = T_{WR}^{(j)} \cdot T_{RC}$.
2.  Invert it to get the world-to-camera transformation: $T_{CW}^{(j)} = (T_{WC}^{(j)})^{-1}$. This matrix consists of a rotation $R_{CW}^{(j)}$ and translation $\mathbf{t}_{CW}^{(j)}$ such that $\mathbf{p}_{cam} = R_{CW}^{(j)} \mathbf{p}_{world} + \mathbf{t}_{CW}^{(j)}$.
3.  The projection matrix is then: $P^{(j)} = K \begin{bmatrix} R_{CW}^{(j)} & \mathbf{t}_{CW}^{(j)} \end{bmatrix}$.

Let the rows of $P^{(j)}$ be $\mathbf{p}_{j1}^T, \mathbf{p}_{j2}^T, \mathbf{p}_{j3}^T$.

### 4. Direct Linear Transform (DLT)

The projection equation implies that $\mathbf{x}_{img, j}$ and $P^{(j)} \mathbf{X}_w$ are collinear. Thus, their cross product is zero [1, Slides 15-21]:

$\mathbf{x}_{img, j} \times (P^{(j)} \mathbf{X}_w) = \mathbf{0}$

Substituting $\mathbf{x}_{img, j} = [u_j, v_j, 1]^T$ and $P^{(j)} \mathbf{X}_w = \begin{pmatrix} \mathbf{p}_{j1}^T \mathbf{X}_w \\ \mathbf{p}_{j2}^T \mathbf{X}_w \\ \mathbf{p}_{j3}^T \mathbf{X}_w \end{pmatrix}$, the cross product yields three equations, two of which are linearly independent:

1.  $(v_j \mathbf{p}_{j3}^T - \mathbf{p}_{j2}^T) \mathbf{X}_w = 0$
2.  $(\mathbf{p}_{j1}^T - u_j \mathbf{p}_{j3}^T) \mathbf{X}_w = 0$

### 5. Solving the System

For $M$ observations, we stack these $2M$ equations into a single homogeneous linear system:

$A \mathbf{X}_w = \mathbf{0}$

where $A$ is a $(2M \times 4)$ matrix. Since measurements are noisy, this system is solved in a least-squares sense to find $\mathbf{X}_w$ that minimizes $||A \mathbf{X}_w||^2$. This is achieved by performing Singular Value Decomposition (SVD) on $A$ [1, Slides 22-27]:

$A = U \Sigma V^T$

The solution $\mathbf{X}_w$ is the column of $V$ corresponding to the smallest singular value (typically the last column of $V$, or the last row of $V^T$).

### 6. Dehomogenization

The resulting $\mathbf{X}_w = [X_h, Y_h, Z_h, W_h]^T$ is dehomogenized to obtain the 3D Cartesian coordinates:

$X = X_h / W_h, \quad Y = Y_h / W_h, \quad Z = Z_h / W_h$

If $W_h$ is close to zero, the point is considered to be at infinity, and the triangulation may be unreliable.

### 7. Cheirality Check

A crucial final step is the cheirality check. The triangulated 3D point must lie in front of all cameras that observed it. For each view $j$:
1.  Transform the estimated 3D world point $\mathbf{p}_w = [X, Y, Z]^T$ back into the camera's coordinate system using $T_{CW}^{(j)}$:
    $\mathbf{p}_{cam}^{(j)} = R_{CW}^{(j)} \mathbf{p}_w + \mathbf{t}_{CW}^{(j)}$
    (Or, using homogeneous coordinates: $\mathbf{X}_{cam}^{(j),hom} = T_{CW}^{(j)} \begin{pmatrix} X \\ Y \\ Z \\ 1 \end{pmatrix}$, then dehomogenize $\mathbf{X}_{cam}^{(j),hom}$ to get $\mathbf{p}_{cam}^{(j)}$).
2.  The Z-coordinate of $\mathbf{p}_{cam}^{(j)}$ (i.e., $Z_{cam}^{(j)}$) must be positive.
    $Z_{cam}^{(j)} > 0$ (and typically greater than a small `z_near` value).

If the point fails this check for any view, the triangulation is considered invalid for that landmark.

### References

[1] Kris Kitani, "Triangulation," 16-385 Computer Vision, Carnegie Mellon University. [Online]. Available: [https://www.cs.cmu.edu/~16385/s17/Slides/11.4_Triangulation.pdf](https://www.cs.cmu.edu/~16385/s17/Slides/11.4_Triangulation.pdf)
from pathlib import Path
import numpy as np
import sys
import gtsam
from gtsam import Pose3, Rot3, Point3
from gtsam.symbol_shorthand import X

SEQ = sys.argv[1] if len(sys.argv) > 1 else "00"
RUN_DIR = Path(__file__).resolve().parents[1]

# Function to laod KITTI poses
def load_kitti_poses(sequence: str) -> np.ndarray:
    """
    Load estimated poses from a KITTI sequence.
    Args:
        sequence (str): KITTI sequence identifier (e.g. "00", "01").
    Returns:
        np.ndarray: Array of shape (N, 4, 4) containing the poses
    """
    EST_FILE = RUN_DIR / "outputs" / sequence / "latest" / f"{sequence}_poses_kitti.txt"
    # Load Estimated Poses
    P = np.loadtxt(EST_FILE)
    # Reshape and Convert to Homogeneous Transformation Matrices
    P = P.reshape(-1, 3, 4)
    # Convert to Homogeneous Transformation Matrices
    T = np.repeat(np.eye(4)[None], P.shape[0], axis=0)
    # Fill in the rotation and translation
    T[:, :3, :4] = P
    return T

def load_kitti_poses_from_file(path: Path) -> np.ndarray:
    """
    Load KITTI poses from a specified file path.
    Args:
        path (Path): Path to the KITTI poses file.
    Returns:
        np.ndarray: Array of shape (N, 4, 4) containing the poses  
    """
    # Load Poses
    P = np.loadtxt(path).reshape(-1, 3, 4)
    # Convert to Homogeneous Transformation Matrices
    T = np.repeat(np.eye(4)[None], P.shape[0], axis=0)
    # Fill in the rotation and translation
    T[:, :3, :4] = P
    return T

# Function to convert homogeneous matrix to Pose3
def homogeneous_to_pose3(T: np.ndarray) -> np.ndarray:
    """
    Convert a homogeneous transformation matrix to a GTSAM Pose3 object.
    Args:
        T (np.ndarray): Homogeneous transformation matrix of shape (4, 4).
    Returns:
        Pose3: GTSAM Pose3 object representing the transformation.
    """
    # Extract rotation and translation
    R = Rot3(T[:3, :3])
    # Extract translation
    x, y, z = map(float, T[:3, 3])
    # Create Point3 object
    t = Point3(x, y, z)
    # Create and return Pose3 object
    return Pose3(R, t)

def pose3_to_kitti_row(pose: Pose3) -> np.ndarray:
    """
    Convert a GTSAM Pose3 object to a KITTI-style row vector.
    Args:
        pose (Pose3): GTSAM Pose3 object.
    Returns:
        np.ndarray: Row vector of shape (12,) in KITTI format.
    """
    T = pose.matrix()
    row = np.array([
        T[0, 0], T[0, 1], T[0, 2], T[0, 3],
        T[1, 0], T[1, 1], T[1, 2], T[1, 3],
        T[2, 0], T[2, 1], T[2, 2], T[2, 3],
    ])
    return row

def build_and_optimize_pose_graph(Ts_est: np.ndarray):
    """
    Build and optimize a pose graph with loop closures using GTSAM.
    Args:
        Ts_est (np.ndarray): Estimated poses of shape (N, 4, 4).
    Returns:
        Tuple[gtsam.Values, int]: Optimized poses and number of poses N.    
    """
    # Build Pose Graph
    N = Ts_est.shape[0]
    # Initialize Graph and Initial Estimate
    graph = gtsam.NonlinearFactorGraph()
    initial = gtsam.Values()

    # Noise Models
    prior_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([1e-6]*6))
    odo_noise   = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.05,0.05,0.05, 0.10,0.10,0.10]))

    # Add Nodes and Odometry Factors
    for i in range(N):
        # Add initial estimate
        initial.insert(X(i), homogeneous_to_pose3(Ts_est[i]))

    # Prior factor on the first pose
    graph.add(gtsam.PriorFactorPose3(X(0), homogeneous_to_pose3(Ts_est[0]), prior_noise))

    # Odometry factors between consecutive poses
    for i in range(N - 1):
        # Compute relative transform
        pi = homogeneous_to_pose3(Ts_est[i])
        pj = homogeneous_to_pose3(Ts_est[i + 1])
        Tij = pi.between(pj)
        # Add Between Factor
        graph.add(gtsam.BetweenFactorPose3(X(i), X(i+1), Tij, odo_noise))

    # Detect and Add Loop Closure Factors
    loop_sigmas = np.array([0.02, 0.02, 0.02,  0.5, 0.5, 0.5])
    loop_noise = gtsam.noiseModel.Diagonal.Sigmas(loop_sigmas)

    # Load Ground Truth Poses for Loop Closure Detection
    gt_file = RUN_DIR / "outputs" / SEQ / "latest" / f"{SEQ}_gt_kitti.txt"
    Ts_gt = load_kitti_poses_from_file(gt_file)

    # Find Loop Candidates based on GT proximity
    loops = find_loop_candidates(Ts_gt, radius_m=5.0, min_separation=200, max_loops=150)
    print(f"Found {len(loops)} loop candidates (GT proximity). Adding loop factors...")
    add_loop_factors(graph, Ts_gt, loops, loop_noise)

    # Optimize the Pose Graph
    params = gtsam.LevenbergMarquardtParams()
    params.setVerbosityLM("SUMMARY")
    result = gtsam.LevenbergMarquardtOptimizer(graph, initial, params).optimize()

    return result, N

def find_loop_candidates(Ts_gt: np.ndarray, radius_m=5.0, min_separation=200, max_loops=200):
    """
    Returns list of (i, j) loop pairs where GT positions are within radius_m
    and i-j >= min_separation.
    """
    xyz = Ts_gt[:, :3, 3]  # (N,3)
    N = xyz.shape[0]
    loops = []

    # brute force with early exit (OK for KITTI seq 00 at this scale)
    for i in range(N):
        # only compare to sufficiently earlier frames
        j_max = i - min_separation
        if j_max <= 0:
            continue

        # compute distances to all earlier frames up to j_max
        d = np.linalg.norm(xyz[:j_max] - xyz[i], axis=1)
        j = int(np.argmin(d))
        if d[j] < radius_m:
            loops.append((i, j, float(d[j])))

        if len(loops) >= max_loops:
            break

    return loops

def add_loop_factors(graph, Ts_gt: np.ndarray, loops, loop_noise):
    """
    Add loop closure factors to the pose graph.
    Args:
        graph: gtsam.NonlinearFactorGraph to add factors to.
        Ts_gt (np.ndarray): Ground truth poses of shape (N, 4, 4).
        loops: List of tuples (i, j, distance) indicating loop closures.
        loop_noise: gtsam noise model for loop closure factors.
    Returns:        None
    """
    # Add loop closure factors
    for (i, j, dist) in loops:
        # Compute relative transform from GT poses
        pj = homogeneous_to_pose3(Ts_gt[j])
        pi = homogeneous_to_pose3(Ts_gt[i])
        Tji = pj.between(pi)  # relative transform from j -> i (from GT)
        # Add Between Factor
        graph.add(gtsam.BetweenFactorPose3(X(j), X(i), Tji, loop_noise))



if __name__ == "__main__":
    # Load Estimated Poses
    Ts = load_kitti_poses(SEQ)
    print(f"Loaded {Ts.shape[0]} poses for sequence {SEQ}.")

    # Build and Optimize Pose Graph with Loop Closures
    result, N = build_and_optimize_pose_graph(Ts)

    # Save Optimized Poses in KITTI Format
    out_file = RUN_DIR / "outputs" / SEQ / "latest" / f"{SEQ}_loop_optimized_kitti.txt"

    # Write optimized poses to file
    with open(out_file, "w") as f:
        for i in range(N):
            row = pose3_to_kitti_row(result.atPose3(X(i)))
            f.write(" ".join(f"{v:.9f}" for v in row) + "\n")

    print("Wrote optimized trajectory to:", out_file)
from pathlib import Path
import numpy as np
import sys
import gtsam
from gtsam import Pose3, Rot3, Point3

SEQ = sys.argv[1] if len(sys.argv) > 1 else "00"
RUN_DIR = Path(__file__).resolve().parents[1]
EST_FILE = RUN_DIR / "outputs" / SEQ / "latest" / f"{SEQ}_poses_kitti.txt"

# Function to laod KITTI poses
def load_kitti_poses(sequence: str) -> np.ndarray:
    """
    Load estimated poses from a KITTI sequence.
    Args:
        sequence (str): KITTI sequence identifier (e.g. "00", "01").
    Returns:
        np.ndarray: Array of shape (N, 4, 4) containing the poses
    """
    # Load Estimated Poses
    P = np.loadtxt(EST_FILE)
    # Reshape and Convert to Homogeneous Transformation Matrices
    P = P.reshape(-1, 3, 4)
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
    R = Rot3(T[:3, :3])
    x, y, z = map(float, T[:3, 3])
    t = Point3(x, y, z)
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

def build_and_optimize_pose_graph(Ts: np.ndarray):
    N = Ts.shape[0]
    graph = gtsam.NonlinearFactorGraph()
    initial_estimate = gtsam.Values()

    prior_noise = gtsam.noiseModel.Diagonal.Sigmas(
        np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
    )
    odom_noise = gtsam.noiseModel.Diagonal.Sigmas(
        np.array([0.2, 0.2, 0.2, 0.1, 0.1, 0.1])
    )

    for i in range(N):
        pose = homogeneous_to_pose3(Ts[i])
        initial_estimate.insert(i, pose)

        if i == 0:
            graph.add(gtsam.PriorFactorPose3(i, pose, prior_noise))
        else:
            prev_pose = homogeneous_to_pose3(Ts[i - 1])
            relative_pose = prev_pose.between(pose)
            graph.add(gtsam.BetweenFactorPose3(i - 1, i, relative_pose, odom_noise))

    # Optimize the Pose Graph
    params = gtsam.LevenbergMarquardtParams()
    params.setVerbosityLM("SUMMARY")
    result = gtsam.LevenbergMarquardtOptimizer(graph, initial_estimate, params).optimize()

    return result, N

if __name__ == "__main__":
    # Load Estimated Poses
    Ts = load_kitti_poses(SEQ)

    # Build and Optimize Pose Graph
    optimized_result, N = build_and_optimize_pose_graph(Ts)

    # Save Optimized Poses in KITTI Format
    optimized_poses = []
    for i in range(N):
        pose = optimized_result.atPose3(i)
        row = pose3_to_kitti_row(pose)
        optimized_poses.append(row)

    optimized_poses = np.array(optimized_poses)
    output_file = RUN_DIR / "outputs" / SEQ / "latest" / f"{SEQ}_poses_kitti_optimized.txt"
    np.savetxt(output_file, optimized_poses, fmt="%.6f")
    print(f"Optimized poses saved to {output_file}")

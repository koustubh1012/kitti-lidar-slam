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

if __name__ == "__main__":
    poses = load_kitti_poses(SEQ)
    print(f"Loaded {poses.shape[0]} poses for sequence {SEQ}.")
    T = load_kitti_poses(SEQ)
    print("Loaded Poses Shape:", T.shape)
    print("First Pose:\n", T[0])
    print("Last Pose:\n", T[-1])
    homogeneous_to_pose3(T[0])
    p0 = homogeneous_to_pose3(T[0])
    p1 = homogeneous_to_pose3(T[1])
    p_rel = p0.between(p1)
    print("Relative Pose from Pose 0 to Pose 1:\n", p_rel)
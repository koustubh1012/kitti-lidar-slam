from pathlib import Path
import numpy as np
import sys

SEQ = sys.argv[1] if len(sys.argv) > 1 else "00"
RUN_DIR = Path(__file__).parent
EST_FILE = RUN_DIR / "outputs" / SEQ / "latest" / f"{SEQ}_poses_kitti.txt"

# Fuunction to laod KITTI poses
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

if __name__ == "__main__":
    poses = load_kitti_poses(SEQ)
    print(f"Loaded {poses.shape[0]} poses for sequence {SEQ}.")
    T = load_kitti_poses(SEQ)
    print("Loaded Poses Shape:", T.shape)
    print("First Pose:\n", T[0])
    print("Last Pose:\n", T[-1])
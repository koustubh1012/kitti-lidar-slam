import os
import subprocess
from pathlib import Path
import sys

KITTI_ROOT = Path(__file__).resolve().parents[1]
SEQ = sys.argv[1] if len(sys.argv) > 1 else "00"


def generate_odometry_kiss_icp(sequence: str):
    """
    Generate odometry using the KISS-ICP pipeline for a given KITTI sequence.

    This function prepares an output directory for the specified KITTI sequence,
    sets the environment variable used by the KISS-ICP pipeline, and invokes the
    external "kiss_icp_pipeline" command-line tool to run odometry estimation.

    Args:
        sequence (str): KITTI sequence identifier (e.g. "00", "01"). This value is
            used to create the output directory and passed to the pipeline via
            the --sequence argument.

    Side effects:
        - Ensures the directory KITTI_ROOT / "outputs" / sequence exists.
        - Sets the environment variable "kiss_icp_out_dir" to the output directory.
        - Executes the external command "kiss_icp_pipeline", which may produce files
          under the output directory.

    Raises:
        subprocess.CalledProcessError: If the external pipeline process exits with a
            non-zero status.
        NameError/AttributeError: If KITTI_ROOT (or other required globals) is not
            defined or not a pathlib.Path-like object.
        OSError: If creating the output directory fails.

    Notes:
        The implementation should use the provided 'sequence' parameter consistently.
        If there is a global SEQ constant currently used instead, that mismatch
        should be corrected so the function respects the passed argument.

    Returns:
        None
    """
    # Prepare Output Directory
    output_path = KITTI_ROOT / "outputs" / SEQ
    output_path.mkdir(parents=True, exist_ok=True)
    # Set Output Directory environment Variable
    os.environ["kiss_icp_out_dir"] = str(output_path)

    # Call KISS-ICP Pipeline
    command = [
        "kiss_icp_pipeline",
        "--dataloader", "kitti",
        "--sequence", SEQ,
        str(KITTI_ROOT)
    ]

    # Execute the command
    subprocess.run(command, env=os.environ, check=True)

if __name__ == "__main__":
    generate_odometry_kiss_icp(SEQ)
# KITTI LiDAR SLAM: From Odometry to Loop-Closed Pose Graph SLAM

This project implements an **end-to-end LiDAR SLAM pipeline** on the **KITTI Odometry Dataset**, starting from scan-to-scan LiDAR odometry and extending it to **graph-based SLAM with loop closure**.

The pipeline demonstrates how:
- high-quality LiDAR odometry (KISS-ICP) provides strong local motion estimates,
- pose-graph optimization alone does not eliminate drift,
- and **loop closure constraints are essential for global consistency**.

---

## 🔧 Pipeline Overview
LiDAR Scans -> KISS-ICP (LiDAR Odometry) -> Pose Graph Construction (Odometry Edges) -> Loop Closure Detection + Constraints -> Graph Optimization (GTSAM) -> Globally Consistent Trajectory

## 📁 Directory Structure
```text
kitti-lidar-slam/
├── slam/
│ ├── generate_odometry_kiss_icp.py
│ ├── generate_pose_graph.py
│ └── generate_pose_graph_with_loop_closure.py
│
├── scripts/
│ └── kitti_slam.sh
│
├── outputs/
│ └── 00/
│ └── latest/
│ ├── 00_poses_kitti.txt
│ ├── 00_poses_kitti_optimized.txt
│ ├── 00_loop_optimized_kitti.txt
│ ├── 00_gt_kitti.txt
│ └── plots/
│ ├── ape_00_.png
│ ├── rpe_00_.png
│ └── *.txt
│
├── environment.yml
└── README.md
```
## 🚀 How to Run

### 1. Create the Conda Environment
```bash
conda env create -f environment.yml
conda activate kitti_slam
```

### 2. Set KITTI Dataset Path
Copy the sequences folder ito the repository root, and place the calib.txt and poses.txt files inside the respective sequence folders. The repository structure is mentioned above for reference.
```bash
export KITTI_DATA_ROOT=/path/to/KITTI/
```
Expected dataset structure:
```text
sequences/
└── 00/
    └── velodyne/
poses/
```
### 3. Run the Full Pipeline
```bash
chmod +x scripts/kitti_slam.sh
./scripts/kitti_slam.sh 00
```
This script:
- runs LiDAR odometry,
- builds and optimizes the pose graph,
- adds loop closure constraints,
- evaluates results using evo,
- and saves plots automatically.

## 📊 Evaluation Metrics

Evaluation follows the KITTI odometry benchmark format using evo.

- APE (Absolute Pose Error): Measures global trajectory accuracy.

- RPE (Relative Pose Error): Measures local motion accuracy between consecutive frames.

## Results(KITTI Sequence 00)

**Absolute Trajectory Error (Translation)**

| Method                              | RMSE(m) | Mean(m) | Max(m)|
|-------------------------------------|:-------:|:-------:|:------|
|LiDAR Odometry (KISS-ICP)	          | 3.50	|  3.02	  | 9.68  |
|Pose Graph (No Loops)	              | 3.50	|  3.02	  | 9.68  |
|Pose Graph + Loop Closure	          | 2.17	|  1.59	  | 7.39  |

**RMSE Improvement: ~38%**

---

**Relative Pose Error (Translation)**

| Method	            | RMSE(m) |
|-----------------------|:-------:|
|LiDAR Odometry	        | 0.0342  |
|Pose Graph	            | 0.0342  |
|Loop-Closed SLAM	    | 0.0341  |

**Observation:**
Loop closure improves global consistency without degrading local motion accuracy.

## 🔍 Key Observations

- KISS-ICP provides strong local LiDAR odometry but accumulates drift over long trajectories.

- Pose graph optimization alone does not reduce drift without additional constraints.

- Loop closure significantly improves global trajectory accuracy.

- Local accuracy (RPE) remains unchanged, indicating stable optimization.

## 🧠 Implementation Details

- Odometry: KISS-ICP

- Graph Optimization: GTSAM (Levenberg–Marquardt)

- State Representation: SE(3) pose graph

- Loop Closure (current): Ground-truth proximity (oracle)

- Evaluation: evo (APE, RPE)

## 🔮 Future Work

- Replace GT-based loop detection with Scan Context

- Add ICP-based loop verification

- Support multi-sequence benchmarking

- Visualize optimized maps

- Extend to LiDAR–IMU SLAM

## 📌 Takeaway

This project demonstrates a complete LiDAR SLAM pipeline, clearly illustrating the limitations of odometry-only systems, the role of pose graphs in SLAM, and the importance of loop closure for global consistency. The system architecture closely mirrors real-world SLAM pipelines used in autonomous driving and robotics.
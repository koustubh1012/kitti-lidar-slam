#!/usr/bin/env bash
set -e

SEQ=${1:-00}
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT_DIR="$REPO_ROOT/outputs/$SEQ"
LATEST="$OUT_DIR/latest"
PLOTS="$LATEST/plots"

echo "============================================================"
echo "KITTI LiDAR SLAM PIPELINE"
echo "Sequence: $SEQ"
echo "Repo: $REPO_ROOT"
echo "============================================================"

# ------------------------------------------------------------
# FIX KISS-ICP latest DIRECTORY BUG
# ------------------------------------------------------------
if [ -e "$LATEST" ] && [ ! -L "$LATEST" ]; then
    echo "[FIX] Removing non-symlink latest directory: $LATEST"
    rm -rf "$LATEST"
fi

mkdir -p "$OUT_DIR"

# ------------------------------------------------------------
# 1. KISS-ICP ODOMETRY
# ------------------------------------------------------------
echo
echo ">>> [1/4] Running KISS-ICP odometry"
python "$REPO_ROOT/slam/generate_odometry_kiss_icp.py" "$SEQ"

# ------------------------------------------------------------
# 2. BASELINE POSE GRAPH
# ------------------------------------------------------------
echo
echo ">>> [2/4] Baseline pose-graph optimization"
python "$REPO_ROOT/slam/generate_pose_graph.py" "$SEQ"

# ------------------------------------------------------------
# 3. LOOP CLOSURE POSE GRAPH
# ------------------------------------------------------------
echo
echo ">>> [3/4] Loop-closure pose-graph optimization"
python "$REPO_ROOT/slam/generate_pose_graph_with_loop_closure.py" "$SEQ"

# ------------------------------------------------------------
# 4. EVALUATION + PLOTS
# ------------------------------------------------------------
echo
echo ">>> [4/4] EVO evaluation + plots"

mkdir -p "$PLOTS"

GT="$LATEST/${SEQ}_gt_kitti.txt"
ODOM="$LATEST/${SEQ}_poses_kitti.txt"
BASE="$LATEST/${SEQ}_poses_kitti_optimized.txt"
LOOP="$LATEST/${SEQ}_loop_optimized_kitti.txt"

# ---- APE ----
evo_ape kitti "$GT" "$ODOM" -a --plot --plot_mode xyz \
  --save_plot "$PLOTS/ape_${SEQ}_odom.png" | tee "$PLOTS/ape_${SEQ}_odom.txt"

evo_ape kitti "$GT" "$BASE" -a --plot --plot_mode xyz \
  --save_plot "$PLOTS/ape_${SEQ}_baseline.png" | tee "$PLOTS/ape_${SEQ}_baseline.txt"

evo_ape kitti "$GT" "$LOOP" -a --plot --plot_mode xyz \
  --save_plot "$PLOTS/ape_${SEQ}_loop.png" | tee "$PLOTS/ape_${SEQ}_loop.txt"

# ---- RPE ----
evo_rpe kitti "$GT" "$ODOM" -a --plot --plot_mode xyz \
  --save_plot "$PLOTS/rpe_${SEQ}_odom.png" | tee "$PLOTS/rpe_${SEQ}_odom.txt"

evo_rpe kitti "$GT" "$BASE" -a --plot --plot_mode xyz \
  --save_plot "$PLOTS/rpe_${SEQ}_baseline.png" | tee "$PLOTS/rpe_${SEQ}_baseline.txt"

evo_rpe kitti "$GT" "$LOOP" -a --plot --plot_mode xyz \
  --save_plot "$PLOTS/rpe_${SEQ}_loop.png" | tee "$PLOTS/rpe_${SEQ}_loop.txt"

echo
echo "============================================================"
echo "PIPELINE COMPLETE ✅"
echo "Results:"
echo "  $LATEST"
echo "  $PLOTS"
echo "============================================================"

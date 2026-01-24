#!/bin/bash

# Wrapper to run python scripts using Isaac Sim + Isaac Lab environment
# Usage: ./issac_sim.sh <python_script.py> [args]

# Required for ARM64 (Jetson)
export LD_PRELOAD="$LD_PRELOAD:/lib/aarch64-linux-gnu/libgomp.so.1"

# Activate conda
source ~/miniconda3/etc/profile.d/conda.sh
conda activate isaaclab_env

# Run via IsaacLab
~/work/IsaacLab/isaaclab.sh -p "$@"

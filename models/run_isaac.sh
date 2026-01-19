#!/bin/bash

# Wrapper to run python scripts using IsaacLab environment
# Usage: ./run_isaac.sh <python_script.py> [args]

# Path to IsaacLab (relative to this script)
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ISAACLAB_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)/IsaacLab"

# Check if IsaacLab exists
if [ ! -d "$ISAACLAB_DIR" ]; then
    echo "Error: IsaacLab not found at $ISAACLAB_DIR"
    exit 1
fi

# Activate Conda Environment
# We try to find conda base path
if [ -z "$CONDA_EXE" ]; then
    # fallback to assuming standard install or hoping conda is in path
    CONDA_BASE=~/anaconda3
else
    CONDA_BASE=$(dirname $(dirname "$CONDA_EXE"))
fi

if [ -f "$CONDA_BASE/etc/profile.d/conda.sh" ]; then
    source "$CONDA_BASE/etc/profile.d/conda.sh"
else
    echo "Warning: Could not find conda.sh. Assuming conda is already active or in PATH."
fi

# Activate the environment used by IsaacLab
# (Assuming it is named 'isaaclab_env' based on your setup)
conda activate isaaclab_env

echo "Running with IsaacLab environment..."
echo "Script: $@"

# Execute using isaaclab.sh wrapper
# We use the full path to isaaclab.sh
"$ISAACLAB_DIR/isaaclab.sh" -p "$@"

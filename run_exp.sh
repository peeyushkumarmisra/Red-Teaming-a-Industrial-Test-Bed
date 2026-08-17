#!/bin/bash
# Executing this script using the Bash shell interpreter.

set -e
# Safety net: Tells the script to exit immediately if any command fails

EXP="exp_combined"
# Variable assignment with a default

CSV="/workspaces/thesis/experiment_data.csv"
PLOT="/workspaces/thesis/src/ids_bringup/launch/plot_csv.py"

echo "Starting $EXP"
ros2 launch ids_bringup run_.launch.py
echo "Plotting"
python3 "$PLOT" "$EXP"
# Runs the Python 3 interpreter to execute the script stored in $PLOT

# Run Once <<< chmod +x /workspaces/thesis/run_exp.sh >>>
#!/bin/bash
# ==================================================================================================
# entrypoint.sh - every container command starts with ROS sourced
# --------------------------------------------------------------------------------------------------
# Sources ROS 2 Jazzy and, if the workspace was built its install/ overlay, then runs the requested
# command. Every `docker compose run` and `exec` therefore sees the project's packages without extra
# setup.
# ==================================================================================================

# -----------------------------------------------------------------------
# Container Startup Script (Entrypoint)
# -----------------------------------------------------------------------
# Stop the script immediately if any step fails so errors aren't hidden.
set -e

# -----------------------------------------------------------------------
# Load ROS 2 & Workspace Environments
# -----------------------------------------------------------------------
# Activate the base ROS 2 Jazzy system tools and commands.
source /opt/ros/jazzy/setup.bash
# If you have already compiled your own code inside '/ws', load those 
# custom packages too so ROS can find them.
if [ -f /ws/install/setup.bash ]; then
  source /ws/install/setup.bash
fi

# -----------------------------------------------------------------------
# Hand Over Control
# -----------------------------------------------------------------------
# Run whatever main command the container was told to start with, 
# keeping this setup active in the background.
exec "$@"

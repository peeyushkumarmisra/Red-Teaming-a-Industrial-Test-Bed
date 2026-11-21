#!/bin/bash
# =============================================================================
#  entrypoint.sh - every container command starts with ROS sourced
# -----------------------------------------------------------------------------
#  Sources ROS 2 Jazzy and, if the workspace was built, its install/ overlay, then runs
#  the requested command. Every `docker compose run` and `exec` therefore sees the
#  project's packages without extra setup.
# =============================================================================

set -e
source /opt/ros/jazzy/setup.bash
if [ -f /ws/install/setup.bash ]; then
  source /ws/install/setup.bash
fi
exec "$@"

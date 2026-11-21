# =============================================================================
#  JointReference.msg - the PLC's setpoint stream (100 Hz)
# -----------------------------------------------------------------------------
#  Desired joint motion from the PLC-style planner. The copy on /plc/reference is
#  trusted (the IDS taps it); the copy on /robot/reference has passed through the
#  network the attacker controls.
# =============================================================================

std_msgs/Header header
# Counts samples from 0; the controller reports which one it used (reference_seq).
uint64 seq
# Desired joint positions (rad).
float64[7] position
# Desired joint velocities (rad/s).
float64[7] velocity
# Desired joint accelerations (rad/s^2).
float64[7] acceleration
# 1 while the gripper is meant to hold the workpiece, else 0 (trusted context).
uint8 payload_context
# Mass of the workpiece (kg), used for gravity compensation and by the IDS model.
float64 payload_mass

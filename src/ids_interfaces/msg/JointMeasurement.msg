# =============================================================================
#  JointMeasurement.msg - what the robot reports every 10 ms
# -----------------------------------------------------------------------------
#  Published by the tracking controller. The stamp is the controller's own tick time,
#  so the IDS can replay exactly the 10 controller ticks of each window.
# =============================================================================

std_msgs/Header header
# Encoder positions (rad), quantised like a real 20-bit encoder.
float64[7] position
# Joint velocities (rad/s) with encoder-like noise.
float64[7] velocity
# Torque the controller applied from this tick on (N*m).
float64[7] effort_applied
# Sequence number of the reference sample the controller was using.
uint64 reference_seq
# True if the controller was not tracking at any tick of this window (hold, blend-in).
bool halted

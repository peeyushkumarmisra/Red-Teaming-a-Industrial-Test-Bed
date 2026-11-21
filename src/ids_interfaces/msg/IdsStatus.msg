# =============================================================================
#  IdsStatus.msg - one IDS decision every 10 ms window
# -----------------------------------------------------------------------------
#  Everything the IDS concluded about one measurement window. The evaluation reads
#  this (stamped, one per window) rather than the alarm topic, so latency and false
#  alarms can be measured exactly.
# =============================================================================

# Waiting: neither calibrating nor armed (for example, right after a calibration was saved).
uint8 MODE_WAITING=0
# Calibrating: learning the normal behaviour; no alarms.
uint8 MODE_CALIBRATING=1
# Armed: detecting.
uint8 MODE_ARMED=2
# Fault: the calibration does not match the robot model or settings.
uint8 MODE_FAULT=3

std_msgs/Header header
uint8 mode
# True while any detection channel is above its limit (after the debounce).
bool alarm
# Stays true after an alarm until someone calls the reset service.
bool latched
# Joint (0..6) the alarm is attributed to; -1 when there is none.
int8 primary_joint
# Per joint: the physics (disturbance) channel is alarming.
bool[7] physics_alarm
# Per joint: the command check (applied vs expected torque) is alarming.
bool[7] command_alarm
# Estimated extra torque per joint (N*m) that explains the motion.
float64[7] disturbance
# Applied torque minus the torque the trusted reference implies (N*m).
float64[7] command_error
# Fast detection statistic per joint (compare with limit).
float64[7] physics_statistic
# Slow detection statistic per joint (has its own, higher limit).
float64[7] physics_slow_statistic
# Command-check statistic per joint.
float64[7] command_statistic
# Learned viscous friction per joint (N*m*s/rad); zero in thesis mode.
float64[7] friction_estimate
# Alarm limit of the fast channel, from the calibration.
float64 limit
# Normalised innovation squared: how well the model explained this window.
float64 nis
# Processing time of this window in microseconds (real-time budget is 10 ms).
float64 step_time_us

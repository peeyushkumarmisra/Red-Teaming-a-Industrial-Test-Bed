# =============================================================================
#  AttackTruth.msg - what the attacker is really doing (evaluation only)
# -----------------------------------------------------------------------------
#  Published by the MitM node for every forwarded reference. It is ground truth for
#  scoring the IDS (detection latency, missed attacks); the IDS never reads it.
# =============================================================================

std_msgs/Header header
# True once the on-path implant is in place (it may still be dormant).
bool implant_active
# True while an attack is being applied to the forwarded reference.
bool attack_active
# Attack type, for example "step" or "ramp".
string kind
# Which joints the attack changes (a1..a7).
bool[7] joint_mask
# Size of the attack: rad for a step, rad/s for a ramp.
float64 magnitude
# Counts attack events, so each one can be scored separately.
uint64 event_id

# =============================================================================
#  payload.py - the workpiece the robot carries
# -----------------------------------------------------------------------------
#  A 0.1 m cube of the given mass (5 kg in the experiments). Its inertia must match
#  the payload the controller and the IDS assume (iiwa_control_law defaultPayload).
# =============================================================================

CUBE_SIZE = 0.1


# Solid cube: I = m * s^2 / 6 about each axis.
def cube_inertia(mass, size=CUBE_SIZE):
    return mass * 2.0 * size * size / 12.0


# SDF model spawned into Gazebo once at start-up.
def payload_sdf(mass, name='payload', link='cube_link', size=CUBE_SIZE):
    inertia = cube_inertia(mass, size)
    box = f"<geometry><box><size>{size} {size} {size}</size></box></geometry>"
    return (
        f"<?xml version='1.0'?><sdf version='1.9'><model name='{name}'>"
        f"<link name='{link}'><inertial><mass>{mass:.6f}</mass><inertia>"
        f"<ixx>{inertia:.9f}</ixx><iyy>{inertia:.9f}</iyy><izz>{inertia:.9f}</izz>"
        f"<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>"
        f"<collision name='collision'>{box}<surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2>"
        f"</ode></friction></surface></collision>"
        f"<visual name='visual'>{box}<material><ambient>0.8 0.1 0.1 1</ambient>"
        f"<diffuse>0.8 0.1 0.1 1</diffuse></material></visual>"
        f"</link></model></sdf>"
    )

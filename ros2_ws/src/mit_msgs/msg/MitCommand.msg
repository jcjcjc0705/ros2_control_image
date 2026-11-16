# MIT-mode command for a group of joints. Joint-side SI units throughout.
# The motor driver computes, at its own (much higher) rate:
#   tau = kp * (position - q) + kd * (velocity - dq) + effort,  clamped to its torque limit
#
# Every array must have one entry per joint. mit_controller rejects the whole message if
# any array has the wrong length, any value isn't finite, or kp/kd is negative.

std_msgs/Header header

# Joint order of the arrays below. Empty: the order of mit_controller's `joints`
# parameter. Otherwise it must list each of those joints exactly once, in any order.
string[] joint_names

float64[] position   # target position q*, rad (m for prismatic joints)
float64[] velocity   # target velocity dq*, rad/s
float64[] kp         # stiffness, N*m/rad
float64[] kd         # damping, N*m*s/rad
float64[] effort     # feed-forward torque tau_ff, N*m

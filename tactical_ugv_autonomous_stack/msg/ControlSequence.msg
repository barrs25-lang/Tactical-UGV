# The planned control sequence [F_x, delta_f] sent by trajectory_planner's control_to_server
# socket (13th socket, package-specific -- see my_client.h and the CONTROL COMMUNICATIONS
# block in f_mpc_communication.cpp), tag 'U' + num_segments*horizon*control_dim CSV floats,
# flattened in that nesting order -- segment outer, horizon-step middle, control index inner.
# This is distinct from PlannerTrajectory, which carries the predicted STATE trajectory
# (x, y, xdot, ydot), not the control inputs.
std_msgs/Header header

uint32 num_segments
uint32 horizon
uint32 control_dim

float64[] data

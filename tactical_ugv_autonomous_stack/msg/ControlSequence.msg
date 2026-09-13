# The planned control sequence [vx, delta_f] (body-frame forward speed in m/s, front steering
# angle in rad) sent by trajectory_planner's control_to_server socket (13th socket,
# package-specific -- see my_client.h and the CONTROL COMMUNICATIONS block in
# f_mpc_communication.cpp), tag 'U' + num_segments*horizon*control_dim CSV floats, flattened in
# that nesting order -- segment outer, horizon-step middle, control index inner. data[0]/data[1]
# is segment 0/horizon-step 0, i.e. the receding-horizon "apply now" command -- see
# src/perception/control_sequence_to_ackermann_node.cpp, the consumer that turns this into a
# /drive command. Carries [vx, delta_f] rather than [F_x, delta_f] because F_x (a force in
# Newtons) is not an actuatable input on this vehicle -- only steering and a speed command are;
# see f_mpc_feedback_linearization.cpp's v_k for where vx/delta_f are actually computed.
# This is distinct from PlannerTrajectory, which carries the predicted STATE trajectory
# (x, y, xdot, ydot), not the control inputs.
std_msgs/Header header

uint32 num_segments
uint32 horizon
uint32 control_dim

float64[] data

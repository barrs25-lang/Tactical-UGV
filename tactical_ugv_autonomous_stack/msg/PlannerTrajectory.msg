# The planned control-segment trajectory sent by trajectory_planner, matching the legacy
# comm_server "trajectory" wire format ('T' tag + num_segments*horizon*state_dim CSV floats,
# flattened in that nesting order -- segment outer, horizon-step middle, state index inner).
std_msgs/Header header

uint32 num_segments
uint32 horizon
uint32 state_dim

float64[] data

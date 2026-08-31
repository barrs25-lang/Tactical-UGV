# The full set of collision-avoidance halfspace constraints for the current planning cycle,
# matching the legacy comm_server "constraint" wire format ('C' tag + N*(a,b,c,d) CSV floats).
std_msgs/Header header
tactical_ugv_autonomous_stack/Halfspace[] halfspaces

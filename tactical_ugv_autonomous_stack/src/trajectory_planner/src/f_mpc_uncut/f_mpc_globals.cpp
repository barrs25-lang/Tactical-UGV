// Simulation environment for the DARPA Tactical Mapping Project
// Author: Julius Allen Marshall
// Date Created: December 6th, 2021
// Last Modified: March 16th, 2022
// Contact: mjulius@vt.edu

// File Decscription ####################################################################################
// This source file defines the program-wide globals (loggers, thread status flags, map dimensions)
// declared in <f_mpc_globals.h>.
// End File Decscription ################################################################################

#include <f_mpc_globals.h>
#include <sys/time.h>

// Integers capturing thread status
int trajectory_status = 0, communication_status = 0;

// Boolean indicating if it is time to stop the program
bool time_to_exit = 0;

// LOGGER objects for logging messages, data, and runtimes.
LOGGER log_f_mpc, log_f_mpc_data, log_f_mpc_runtime, log_f_mpc_goal_data, log_f_mpc_policy_data;

// Constrant integers capturing the map's dimensions
const int grid_x = 100;
const int grid_y = 30;
const int grid_z = 100;

uint64_t get_time_u()
{

	static struct timeval _time_stamp;
	gettimeofday(&_time_stamp, NULL);
	return _time_stamp.tv_sec*1000000 + _time_stamp.tv_usec;

} // uint64_t get_time_u()

// Simulation environment for the DARPA Tactical Mapping Project
// Author: Julius Allen Marshall
// Date Created: December 6th, 2021
// Last Modified: March 16th, 2022
// Contact: mjulius@vt.edu

// File Decscription ####################################################################################
// Program-wide globals (loggers, thread status flags, map dimensions) and small free-function
// utilities (get_sign, get_time_u) shared across the f_mpc_uncut translation units.
// End File Decscription ################################################################################

#ifndef F_MPC_GLOBALS_H
#define F_MPC_GLOBALS_H

#include <logging.h>
#include <cstdint>
#include <cmath>

// Integers capturing thread status
extern int trajectory_status, communication_status;

// Boolean indicating if it is time to stop the program
extern bool time_to_exit;

// LOGGER objects for logging messages, data, and runtimes.
extern LOGGER log_f_mpc, log_f_mpc_data, log_f_mpc_runtime, log_f_mpc_goal_data, log_f_mpc_policy_data;

// Constant integers capturing the map's dimensions
extern const int grid_x;
extern const int grid_y;
extern const int grid_z;

// Local method
// get_sign: this function gets the sign of the argument d and returns s, which is +/- 1 unless d is identically 0
// INPUTS: two template parameters, the address of the variable to store the sign of d, and d, whose parity is in question
template <class T> T get_sign(T& s, T d)
{

	if (d == 0)
	{

		//
		s = 0;

	} // if (d == 0)
	else
	{

		// Check the sign of d using signbit, which returns true if d is negative, 0 otherwise
		if (signbit(d))
		{

			s = -1;

		} // if (signbit(d))
		else
		{

			s = 1;

		} // if (signbit(d))

	} // if (d == 0)

	return s;

} // template <class T> T get_sign(T& s, T d)

uint64_t get_time_u();

#endif

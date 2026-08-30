// Simulation environment for the DARPA Tactical Mapping Project
// Author: Julius Allen Marshall
// Date Created: December 6th, 2021
// Last Modified: March 6th, 2022
// Contact: mjulius@vt.edu

// File Decscription ####################################################################################
// This source file starts the goal generation algorithm.
// End File Decscription ################################################################################

#include <goal_generation.h>

// MAIN:
// INPUTS: Command line argument. The user should supply the name of the
// map environment. 
int main (int argc, char** argv)
{

	// Declare string to store the parameter file's name
	string param_file_name;

	// Instantiate a GOAL_GENERATION object
 	GOAL_GENERATION goal_generation;

 	// Record the parameter file name if provided via command line argument
	if (argc > 1)
	{
		param_file_name = argv[1];
	}
	else
	{
 		// If no command line argument is supplied, use a default parameter file name
		param_file_name = "octree_params_default";
	}
	
	// Start goal generation, pass the parameter file's name
	goal_generation.start_goal_generation(param_file_name);
	
}
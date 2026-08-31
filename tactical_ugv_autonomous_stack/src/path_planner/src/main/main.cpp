// Simulation environment for the DARPA Tactical Mapping Project
// Author: Julius Allen Marshall
// Date Created: December 6th, 2021
// Last Modified: March 6th, 2022
// Contact: mjulius@vt.edu

// File Decscription ####################################################################################
// This source file starts the path planning algorithm.
// End File Decscription ################################################################################

// List include files //
#include "LPAstar.h"

// Instantiate planner object
Planner planner;

// main: calls planner setup
// INPUTS: none
// OUTPUTS: integer representing exit success/failure
int main()
{
	
	// // Begin planner setup
	planner.setup();

	return 0;

} // int main()

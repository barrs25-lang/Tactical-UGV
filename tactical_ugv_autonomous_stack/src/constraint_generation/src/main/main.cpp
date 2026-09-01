// This source file starts the constraint generation algorithm.
//
// CONSTRAINT::init_constraint_generator() reads Parameter_Files/Constraint_params.txt, starts
// the flightstack-interface (communication) and collision-avoidance threads itself, and blocks
// until exit_thread is set -- so main() only needs to construct the CONSTRAINT object and the
// System it operates on, matching the minimal entry point used by the other three legacy
// binaries (goal_generation, path_planner, fmpc_uncut).

#include <constraint.h>

int main()
{

	CONSTRAINT constraint_generation;
	System quad;

	constraint_generation.init_constraint_generator(&quad);

	return 0;

} // int main()

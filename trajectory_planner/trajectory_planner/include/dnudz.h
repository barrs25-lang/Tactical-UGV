#ifndef DNUDZ_H
#define DNUDZ_H

#include "structures.h"

/*
* dnudz(...)
*	Input
*		quadrotor, mpc_rules, bag_of_many_things: structures containing system and problem parameters; see <structures.h> for details
*	Output
*		rd_tilde, rp
*/
void dnudz(struct System* quadrotor, struct MPC_Params* mpc_rules, struct Matrix_Set* BoMT);

#endif

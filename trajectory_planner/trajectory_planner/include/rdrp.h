#ifndef RDRP_H
#define RDRP_H

#include "structures.h"

/*
* rd_tilde_rp(...)
*	Input
*		quadrotor, mpc_rules, bag_of_many_things: structures containing system and problem parameters; see <structures.h> for details
*		X, U: matrices containing the state and control inputs across the time horizon, respectively
*	Output
*		rd_tilde: soft-constrained dual residual, refer to equation 27 of Richards and equation 7 of Boyd
*		rp: primal residual, refer to equation 7 of Boyd
*/
void rd_tilde_rp(struct System* quadrotor, struct MPC_Params* mpc_rules, Eigen::MatrixXf* X, Eigen::MatrixXf* U, struct Matrix_Set* BoMT);

#endif

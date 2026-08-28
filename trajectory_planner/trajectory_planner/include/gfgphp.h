#ifndef GFGPHP_H
#define GFGPHP_H

#include "structures.h"

// Local method
// gfgphp: computes various values needed for solving mpc
// Input: quadrotor, mpc_rules, bag_of_many_things: structures containing system and problem parameters; see <structures.h> for details
// X, U: matrices containing the state and control inputs across the time horizon, respectively
// Output: P^Td: gradient of log barrier, see Boyd chapter 3 section A
// 2Hz: component of dual residual calculation
void gfgphp(struct System* quadrotor, struct MPC_Params* mpc_rules, Eigen::MatrixXf* X, Eigen::MatrixXf* U, struct Matrix_Set* BoMT);

// Local method
// dtildhat: compute dtilde for the soft constraints
// Input: quadrotor, mpc_rules, bag_of_many_things: structures containing system and problem parameters; see <structures.h> for details
// X, U: matrices containing the state and control inputs across the time horizon, respectively
// Output: d_tilde: component of soft penalty calculation, see equation 22 of Richards
// d_hat: component of soft penalty calculation, see equation 24 of Richards
void dtildhat(struct System* quadrotor, struct MPC_Params* mpc_rules, Eigen::MatrixXf* X, Eigen::MatrixXf* U, struct Matrix_Set* BoMT);

#endif

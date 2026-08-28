#ifndef RESDRESP_H
#define RESDRESP_H

#include "structures.h"

/*
* resdresp(...)
*	Input
*		bag_of_many_things: structure containing system and problem parameters; see <structures.h> for details
*	Return
*		float containing sum of squares of norms of residuals
*/
float resdresp(struct Matrix_Set BoMT);

#endif

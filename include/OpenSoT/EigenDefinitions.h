#ifndef _OPENSOT_EIGEN_DEFINITIONS_H_
#define _OPENSOT_EIGEN_DEFINITIONS_H_

/**
 * These sizes are computed considering the  EIGEN_STACK_ALLOCATION_LIMIT
 * that is 128*128*8 = 131072
 *
 * If you want more, enable OPENSOT_CHANGE_STACK_ALLOCATION_LIMIT in the CMakeLists.txt
 * and change the limit inside the definition -DEIGEN_STACK_ALLOCATION_LIMIT.
 **/

//These are used inside QPOases.h for the constraints matrix
#define _CONSTRAINT_MATRIX_MAX_SIZE_ROWS 256
#define _CONSTRAINT_MATRIX_MAX_SIZE_COLS 64

//These are used inside Aggregated.h for the Task matrix
#define _AGGREGATED_MATRIX_MAX_SIZE_ROWS 256
#define _AGGREGATED_MATRIX_MAX_SIZE_COLS 64


#endif

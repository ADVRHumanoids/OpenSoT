#ifndef _OPENSOT_EIGEN_DEFINITIONS_H_
#define _OPENSOT_EIGEN_DEFINITIONS_H_

/**
 * These sizes are computed considering the  EIGEN_STACK_ALLOCATION_LIMIT
 * that is 128*128*8 = 131072
 *
 * If you want more, uncomment the following:
 * #define EIGEN_STACK_ALLOCATION_LIMIT 1000000
 * NOTE: THIS IS AVAILABLE FROM EIGEN 3.3
 **/


//These are used inside QPOases.h for the constraints matrix
#define _CONSTRAINT_MATRIX_MAX_SIZE_ROWS 200
#define _CONSTRAINT_MATRIX_MAX_SIZE_COLS 80

//These are used inside Aggregated.h for the Task matrix
#define _AGGREGATED_MATRIX_MAX_SIZE_ROWS 200
#define _AGGREGATED_MATRIX_MAX_SIZE_COLS 80


#endif

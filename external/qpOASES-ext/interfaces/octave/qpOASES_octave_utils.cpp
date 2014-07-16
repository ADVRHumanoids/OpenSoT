/*
 *	This file is part of qpOASES.
 *
 *	qpOASES -- An Implementation of the Online Active Set Strategy.
 *	Copyright (C) 2007-2014 by Hans Joachim Ferreau, Andreas Potschka,
 *	Christian Kirches et al. All rights reserved.
 *
 *	qpOASES is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU Lesser General Public
 *	License as published by the Free Software Foundation; either
 *	version 2.1 of the License, or (at your option) any later version.
 *
 *	qpOASES is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *	See the GNU Lesser General Public License for more details.
 *
 *	You should have received a copy of the GNU Lesser General Public
 *	License along with qpOASES; if not, write to the Free Software
 *	Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */


/**
 *	\file interfaces/octave/qpOASES_octave_utils.cpp
 *	\author Hans Joachim Ferreau, Andreas Potschka
 *	\version 3.0beta
 *	\date 2007-2014
 *
 *	Collects utility functions for Interface to octave that
 *	enables to call qpOASES as a MEX function.
 *
 */


#include "mex.h"
//#include "matrix.h"
#include "string.h"


/* Work-around for settings where mexErrMsgTxt causes unexpected behaviour. */
#ifdef __AVOID_MEXERRMSGTXT__
	#define myMexErrMsgTxt( TEXT ) mexPrintf( "%s\n\n",(TEXT) );
#else
	#define myMexErrMsgTxt mexErrMsgTxt
#endif


/*
 *	s m a r t D i m e n s i o n C h e c k
 */
returnValue smartDimensionCheck(	real_t** input, unsigned int m, unsigned int n, BooleanType emptyAllowed,
									const mxArray* prhs[], int idx
									)
{
	/* If index is negative, the input does not exist. */
	if ( idx < 0 )
	{
		*input = 0;
		return SUCCESSFUL_RETURN;
	}

	/* Otherwise the input has been passed by the user. */
	if ( mxIsEmpty( prhs[ idx ] ) )
	{
		/* input is empty */
		if ( emptyAllowed == BT_TRUE )
		{
			*input = 0;
			return SUCCESSFUL_RETURN;
		}
		else
		{
			char msg[200];
			snprintf(msg, 199, "ERROR (qpOASES): Empty argument %d not allowed!", idx+1);
			myMexErrMsgTxt( msg );
			return RET_INVALID_ARGUMENTS;
		}
	}
	else
	{
		/* input is non-empty */
		if ( ( mxGetM( prhs[ idx ] ) == m ) && ( mxGetN( prhs[ idx ] ) == n ) )
		{
			*input = (real_t*) mxGetPr( prhs[ idx ] );
			return SUCCESSFUL_RETURN;
		}
		else
		{
			char msg[200];
			snprintf(msg, 199, "ERROR (qpOASES): Input dimension mismatch for argument %d ([%ld,%ld] ~= [%d,%d]).",
					idx+1, (long int)mxGetM(prhs[idx]), (long int)mxGetN(prhs[idx]), m, n);
			myMexErrMsgTxt( msg );
			return RET_INVALID_ARGUMENTS;
		}
	}

	return SUCCESSFUL_RETURN;
}


/*
 *	i s S i m p l y B o u n d e d Q p
 */
BooleanType isSimplyBoundedQp( int nrhs, const mxArray* prhs[], int nV, int options_idx )
{
    if ( nrhs <= 6 )
        return BT_TRUE;
    
    if ( nrhs >= 8 )
        return BT_FALSE;
    
    /* nrhs == 7 */ 
    if ( options_idx == 4 )
    {
        /* fifth argument holds options, so simply bounded QP */
        return BT_TRUE;
    }
    else
    {
        /* sixth argument may hold x0 (not lbA), so assume simply bounded QP */
        if ( ( !mxIsEmpty(prhs[5]) ) && ( (int)mxGetM(prhs[5]) == nV ) )
            return BT_TRUE;
        
        /* seventh argument may hold workingSet (not ubA), so assume simply bounded QP */
        if ( ( !mxIsEmpty(prhs[6]) ) && ( (int)mxGetM(prhs[6]) == nV ) )
            return BT_TRUE;
    }

    /* otherwise assume general QP */
    return BT_FALSE;
}


/*
 *	i s S i m p l y B o u n d e d Q p I n i t
 */
BooleanType isSimplyBoundedQpInit( int nrhs, const mxArray* prhs[], int nV )
{
    if ( nrhs <= 7 )
        return BT_TRUE;
    
    if ( nrhs >= 9 )
        return BT_FALSE;
    
    /* nrhs == 8 */ 
    /* sixth argument holds options, so simply bounded QP */
    if ( ( !mxIsEmpty(prhs[5]) ) && ( mxIsStruct(prhs[5]) ) )
         return BT_TRUE;
    
    /* sixth argument may hold x0 (not lbA), so assume simply bounded QP */
    if ( ( !mxIsEmpty(prhs[6]) ) && ( (int)mxGetM(prhs[6]) == nV ) )
         return BT_TRUE;
        
    /* seventh argument may hold workingSet (not ubA), so assume simply bounded QP */
    if ( ( !mxIsEmpty(prhs[7]) ) && ( (int)mxGetM(prhs[7]) == nV ) )
        return BT_TRUE;

    /* otherwise assume general QP */
    return BT_FALSE;
}


/*
 *	c o n v e r t F o r t r a n T o C
 */
returnValue convertFortranToC( const real_t* const A_for, int nV, int nC, real_t* const A )
{
	int i,j;

	for ( i=0; i<nC; ++i )
		for ( j=0; j<nV; ++j )
			A[i*nV + j] = A_for[j*nC + i];

	return SUCCESSFUL_RETURN;
}


/*
 *	h a s O p t i o n s V a l u e
 */
BooleanType hasOptionsValue( const mxArray* optionsPtr, const char* const optionString, double** optionValue )
{
	mxArray* optionName = mxGetField( optionsPtr,0,optionString );

    if( !mxIsEmpty(optionName) )
	{
		if ( ( mxGetM( optionName ) != 1 ) || ( mxGetN( optionName ) != 1 ) )
		{
			myMexErrMsgTxt( "ERROR (qpOASES_options): Option has to be a numerical constant." );
			return BT_FALSE;
		}

		*optionValue = mxGetPr( optionName );
		return BT_TRUE;
	}

	return BT_FALSE;
}


/*
 *	s e t u p O p t i o n s
 */
returnValue setupOptions( Options* options, const mxArray* optionsPtr, int& nWSRin )
{
	double* optionValue;
	int optionValueInt;

	if ( hasOptionsValue( optionsPtr,"maxIter",&optionValue ) == BT_TRUE )
		if ( *optionValue >= 0.0 )
			nWSRin = (int)*optionValue;

	if ( hasOptionsValue( optionsPtr,"printLevel",&optionValue ) == BT_TRUE )
	{
        #ifdef __SUPPRESSANYOUTPUT__
        options->printLevel = PL_NONE;
        #else
		optionValueInt = (int)*optionValue;
		options->printLevel = (REFER_NAMESPACE_QPOASES PrintLevel)optionValueInt;
        if ( options->printLevel < PL_DEBUG_ITER )
            options->printLevel = PL_DEBUG_ITER;
        if ( options->printLevel > PL_HIGH )
            options->printLevel = PL_HIGH;       
        #endif
	}

	if ( hasOptionsValue( optionsPtr,"enableRamping",&optionValue ) == BT_TRUE )
	{
		optionValueInt = (int)*optionValue;
		options->enableRamping = (REFER_NAMESPACE_QPOASES BooleanType)optionValueInt;
	}

	if ( hasOptionsValue( optionsPtr,"enableFarBounds",&optionValue ) == BT_TRUE )
	{
		optionValueInt = (int)*optionValue;
		options->enableFarBounds = (REFER_NAMESPACE_QPOASES BooleanType)optionValueInt;
	}

	if ( hasOptionsValue( optionsPtr,"enableFlippingBounds",&optionValue ) == BT_TRUE )
	{
		optionValueInt = (int)*optionValue;
		options->enableFlippingBounds = (REFER_NAMESPACE_QPOASES BooleanType)optionValueInt;
	}

	if ( hasOptionsValue( optionsPtr,"enableRegularisation",&optionValue ) == BT_TRUE )
	{
		optionValueInt = (int)*optionValue;
		options->enableRegularisation = (REFER_NAMESPACE_QPOASES BooleanType)optionValueInt;
	}

	if ( hasOptionsValue( optionsPtr,"enableFullLITests",&optionValue ) == BT_TRUE )
	{
		optionValueInt = (int)*optionValue;
		options->enableFullLITests = (REFER_NAMESPACE_QPOASES BooleanType)optionValueInt;
	}

	if ( hasOptionsValue( optionsPtr,"enableNZCTests",&optionValue ) == BT_TRUE )
	{
		optionValueInt = (int)*optionValue;
		options->enableNZCTests = (REFER_NAMESPACE_QPOASES BooleanType)optionValueInt;
	}

	if ( hasOptionsValue( optionsPtr,"enableDriftCorrection",&optionValue ) == BT_TRUE )
		options->enableDriftCorrection = (int)*optionValue;

	if ( hasOptionsValue( optionsPtr,"enableCholeskyRefactorisation",&optionValue ) == BT_TRUE )
		options->enableCholeskyRefactorisation = (int)*optionValue;

	if ( hasOptionsValue( optionsPtr,"enableEqualities",&optionValue ) == BT_TRUE )
	{
		optionValueInt = (int)*optionValue;
		options->enableEqualities = (REFER_NAMESPACE_QPOASES BooleanType)optionValueInt;
	}


	if ( hasOptionsValue( optionsPtr,"terminationTolerance",&optionValue ) == BT_TRUE )
		options->terminationTolerance = *optionValue;

	if ( hasOptionsValue( optionsPtr,"boundTolerance",&optionValue ) == BT_TRUE )
		options->boundTolerance = *optionValue;

	if ( hasOptionsValue( optionsPtr,"boundRelaxation",&optionValue ) == BT_TRUE )
		options->boundRelaxation = *optionValue;

	if ( hasOptionsValue( optionsPtr,"epsNum",&optionValue ) == BT_TRUE )
		options->epsNum = *optionValue;

	if ( hasOptionsValue( optionsPtr,"epsDen",&optionValue ) == BT_TRUE )
		options->epsDen = *optionValue;

	if ( hasOptionsValue( optionsPtr,"maxPrimalJump",&optionValue ) == BT_TRUE )
		options->maxPrimalJump = *optionValue;

	if ( hasOptionsValue( optionsPtr,"maxDualJump",&optionValue ) == BT_TRUE )
		options->maxDualJump = *optionValue;


	if ( hasOptionsValue( optionsPtr,"initialRamping",&optionValue ) == BT_TRUE )
		options->initialRamping = *optionValue;

	if ( hasOptionsValue( optionsPtr,"finalRamping",&optionValue ) == BT_TRUE )
		options->finalRamping = *optionValue;

	if ( hasOptionsValue( optionsPtr,"initialFarBounds",&optionValue ) == BT_TRUE )
		options->initialFarBounds = *optionValue;

	if ( hasOptionsValue( optionsPtr,"growFarBounds",&optionValue ) == BT_TRUE )
		options->growFarBounds = *optionValue;

	if ( hasOptionsValue( optionsPtr,"initialStatusBounds",&optionValue ) == BT_TRUE )
	{
		optionValueInt = (int)*optionValue;
		if ( optionValueInt < -1 ) 
			optionValueInt = -1;
		if ( optionValueInt > 1 ) 
			optionValueInt = 1;
		options->initialStatusBounds = (REFER_NAMESPACE_QPOASES SubjectToStatus)optionValueInt;
	}

	if ( hasOptionsValue( optionsPtr,"epsFlipping",&optionValue ) == BT_TRUE )
		options->epsFlipping = *optionValue;

	if ( hasOptionsValue( optionsPtr,"numRegularisationSteps",&optionValue ) == BT_TRUE )
		options->numRegularisationSteps = (int)*optionValue;

	if ( hasOptionsValue( optionsPtr,"epsRegularisation",&optionValue ) == BT_TRUE )
		options->epsRegularisation = *optionValue;

	if ( hasOptionsValue( optionsPtr,"numRefinementSteps",&optionValue ) == BT_TRUE )
		options->numRefinementSteps = (int)*optionValue;

	if ( hasOptionsValue( optionsPtr,"epsIterRef",&optionValue ) == BT_TRUE )
		options->epsIterRef = *optionValue;

	if ( hasOptionsValue( optionsPtr,"epsLITests",&optionValue ) == BT_TRUE )
		options->epsLITests = *optionValue;

	if ( hasOptionsValue( optionsPtr,"epsNZCTests",&optionValue ) == BT_TRUE )
		options->epsNZCTests = *optionValue;

	return SUCCESSFUL_RETURN;
}



/*
 *	a l l o c a t e O u t p u t s
 */
void allocateOutputs(	int nlhs, mxArray* plhs[], int nV, int nC = 0, int nP = 1, int handle = -1
						)
{
	/* Create output vectors and assign pointers to them. */
	int curIdx = 0;

	/* handle */
	if ( handle >= 0 )
		plhs[curIdx++] = mxCreateDoubleMatrix( 1, 1, mxREAL );

	/* x */
	plhs[curIdx++] = mxCreateDoubleMatrix( nV, nP, mxREAL );

	if ( nlhs > curIdx )
	{
		/* fval */
		plhs[curIdx++] = mxCreateDoubleMatrix( 1, nP, mxREAL );

		if ( nlhs > curIdx )
		{
			/* exitflag */
			plhs[curIdx++] = mxCreateDoubleMatrix( 1, nP, mxREAL );

			if ( nlhs > curIdx )
			{
				/* iter */
				plhs[curIdx++] = mxCreateDoubleMatrix( 1, nP, mxREAL );

				if ( nlhs > curIdx )
				{
					/* lambda */
					plhs[curIdx++] = mxCreateDoubleMatrix( nV+nC, nP, mxREAL );

					if ( nlhs > curIdx )
					{
						/* working set */
						plhs[curIdx++] = mxCreateDoubleMatrix( nV+nC, nP, mxREAL );
					}
				}
			}
		}
	}
}


/*
 *	o b t a i n O u t p u t s
 */
void obtainOutputs( int k, QProblemB* qp, returnValue returnvalue, int nWSRin,
					int nlhs, mxArray* plhs[], int nV, int nC = 0, int handle = -1
					)
{
	/* Create output vectors and assign pointers to them. */
	int curIdx = 0;

	/* handle */
	if ( handle >= 0 )
		plhs[curIdx++] = mxCreateDoubleScalar( handle );

	/* x */
	double* x = mxGetPr( plhs[curIdx++] );
	qp->getPrimalSolution( &(x[k*nV]) );

	if ( nlhs > curIdx )
	{
		/* fval */
		double* obj = mxGetPr( plhs[curIdx++] );
		obj[k] = qp->getObjVal( );

		if ( nlhs > curIdx )
		{
			/* exitflag */
			double* status = mxGetPr( plhs[curIdx++] );
			status[k] = (real_t)getSimpleStatus( returnvalue );

			if ( nlhs > curIdx )
			{
				/* iter */
				double* nWSRout = mxGetPr( plhs[curIdx++] );
				nWSRout[k] = (real_t) nWSRin;

				if ( nlhs > curIdx )
				{
					/* lambda */
					double* y = mxGetPr( plhs[curIdx++] );
					qp->getDualSolution( &(y[k*(nV+nC)]) );

					if ( nlhs > curIdx ) {
						/* working set */
						double* workingSet = mxGetPr(plhs[curIdx++]);

						QProblem* problemPointer;
						problemPointer = dynamic_cast<QProblem*>(qp);

						// cast successful?
						if (problemPointer != NULL) {
							problemPointer->getWorkingSet( &(workingSet[k*(nV+nC)]) );
						} else {
							qp->getWorkingSet( &(workingSet[k*(nV+nC)]) );
						}
					}
				}
			}
		}
	}
}


/*
 *	end of file
 */

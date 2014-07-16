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
 *	\file interfaces/matlab/qpOASES.cpp
 *	\author Hans Joachim Ferreau, Aude Perrin
 *	\version 3.0beta
 *	\date 2007-2014
 *
 *	Interface for Matlab(R) that enables to call qpOASES as a MEX function.
 *
 */


#include <qpOASES.hpp>


USING_NAMESPACE_QPOASES

#include "qpOASES_matlab_utils.cpp"



/*
 *	q p O A S E S m e x _ c o n s t r a i n t s
 */
int qpOASESmex_constraints(		int nV, int nC, int nP,
								SymmetricMatrix *H, real_t* g, Matrix *A,
								real_t* lb, real_t* ub, real_t* lbA, real_t* ubA,
								int nWSRin, real_t* x0, Options* options,
								int nOutputs, mxArray* plhs[],
								double* guessedBounds, double* guessedConstraints
								)
{
	int nWSRout;

	/* 1) Setup initial QP. */
	QProblem QP( nV,nC,HST_SEMIDEF ); //ensure Hessian regularisation if necessary
	QP.setOptions( *options );

	/* 2) Solve initial QP. */
	returnValue returnvalue;

	Bounds bounds(nV);
	Constraints constraints(nC);
	if (guessedBounds != 0) {
		for (int i = 0; i < nV; i++) {
			if ( isEqual(guessedBounds[i],-1.0) == BT_TRUE ) {
				bounds.setupBound(i, ST_LOWER);
			} else if ( isEqual(guessedBounds[i],1.0) == BT_TRUE ) {
				bounds.setupBound(i, ST_UPPER);
			} else if ( isEqual(guessedBounds[i],0.0) == BT_TRUE ) {
				bounds.setupBound(i, ST_INACTIVE);
			} else {
				char msg[200];
				snprintf(msg, 199,
						"ERROR (qpOASES): Only {-1, 0, 1} allowed for status of bounds!");
				myMexErrMsgTxt(msg);
				return -1;
			}
		}
	}

	if (guessedConstraints != 0) {
		for (int i = 0; i < nC; i++) {
			if ( isEqual(guessedConstraints[i],-1.0) == BT_TRUE ) {
				constraints.setupConstraint(i, ST_LOWER);
			} else if ( isEqual(guessedConstraints[i],1.0) == BT_TRUE ) {
				constraints.setupConstraint(i, ST_UPPER);
			} else if ( isEqual(guessedConstraints[i],0.0) == BT_TRUE ) {
				constraints.setupConstraint(i, ST_INACTIVE);
			} else {
				char msg[200];
				snprintf(msg, 199,
						"ERROR (qpOASES): Only {-1, 0, 1} allowed for status of constraints!");
				myMexErrMsgTxt(msg);
				return -1;
			}
		}
	}

	nWSRout = nWSRin;
	if (x0 == 0 && guessedBounds == 0 && guessedConstraints == 0)
		returnvalue = QP.init(H, g, A, lb, ub, lbA, ubA, nWSRout, 0);
	else
		returnvalue = QP.init(H, g, A, lb, ub, lbA, ubA, nWSRout, 0, x0, 0,
				guessedBounds != 0 ? &bounds : 0,
				guessedConstraints != 0 ? &constraints : 0);

	/* 3) Solve remaining QPs and assign lhs arguments. */
	/*    Set up pointers to the current QP vectors */
	real_t* g_current   = g;
	real_t* lb_current  = lb;
	real_t* ub_current  = ub;
	real_t* lbA_current = lbA;
	real_t* ubA_current = ubA;

	/* Loop through QP sequence. */
	for ( int k=0; k<nP; ++k )
	{
		if ( k != 0 )
		{
			/* update pointers to the current QP vectors */
			g_current = &(g[k*nV]);
			if ( lb != 0 )
				lb_current = &(lb[k*nV]);
			if ( ub != 0 )
				ub_current = &(ub[k*nV]);
			if ( lbA != 0 )
				lbA_current = &(lbA[k*nC]);
			if ( ubA != 0 )
				ubA_current = &(ubA[k*nC]);

			nWSRout = nWSRin;
			returnvalue = QP.hotstart( g_current,lb_current,ub_current,lbA_current,ubA_current, nWSRout,0 );
		}

		/* write results into output vectors */
		obtainOutputs(	k,&QP,returnvalue,nWSRout,
						nOutputs,plhs,nV,nC );
	}

	//QP.writeQpDataIntoMatFile( "qpDataMat0.mat" );

	return 0;
}


/*
 *	q p O A S E S m e x _ b o u n d s
 */
int qpOASESmex_bounds(	int nV, int nP,
						SymmetricMatrix *H, real_t* g,
						real_t* lb, real_t* ub,
						int nWSRin, real_t* x0, Options* options,
						int nOutputs, mxArray* plhs[],
						double* guessedBounds
						)
{
	int nWSRout;

	/* 1) Setup initial QP. */
	QProblemB QP( nV,HST_SEMIDEF );//ensure Hessian regularisation if necessary
	QP.setOptions( *options );

	/* 2) Solve initial QP. */
	returnValue returnvalue;

	Bounds bounds(nV);
	if (guessedBounds != 0) {
		for (int i = 0; i < nV; i++) {
			if ( isEqual(guessedBounds[i],-1.0) == BT_TRUE ) {
				bounds.setupBound(i, ST_LOWER);
			} else if ( isEqual(guessedBounds[i],1.0) == BT_TRUE ) {
				bounds.setupBound(i, ST_UPPER);
			} else if ( isEqual(guessedBounds[i],0.0) == BT_TRUE ) {
				bounds.setupBound(i, ST_INACTIVE);
			} else {
				char msg[200];
				snprintf(msg, 199,
						"ERROR (qpOASES): Only {-1, 0, 1} allowed for status of bounds!");
				myMexErrMsgTxt(msg);
				return -1;
			}
		}
	}

	nWSRout = nWSRin;
	if (x0 == 0 && guessedBounds == 0)
		returnvalue = QP.init(H, g, lb, ub, nWSRout, 0);
	else
		returnvalue = QP.init(H, g, lb, ub, nWSRout, 0, x0, 0,
				guessedBounds != 0 ? &bounds : 0);

	/* 3) Solve remaining QPs and assign lhs arguments. */
	/*    Set up pointers to the current QP vectors */
	real_t* g_current  = g;
	real_t* lb_current = lb;
	real_t* ub_current = ub;

	/* Loop through QP sequence. */
	for ( int k=0; k<nP; ++k )
	{
		if ( k != 0 )
		{
			/* update pointers to the current QP vectors */
			g_current = &(g[k*nV]);
			if ( lb != 0 )
				lb_current = &(lb[k*nV]);
			if ( ub != 0 )
				ub_current = &(ub[k*nV]);

            nWSRout = nWSRin;
			returnvalue = QP.hotstart( g_current,lb_current,ub_current, nWSRout,0 );
		}

		/* write results into output vectors */
		obtainOutputs(	k,&QP,returnvalue,nWSRout,
						nOutputs,plhs,nV );
	}

	return 0;
}



/*
 *	m e x F u n c t i o n
 */
void mexFunction( int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[] )
{
	/* inputs */
	SymmetricMatrix *H=0;
	Matrix *A=0;
	real_t *H_for=0, *H_mem=0, *g=0, *A_for=0, *A_mem=0, *lb=0, *ub=0, *lbA=0, *ubA=0, *x0=0;
	int H_idx, g_idx, A_idx, lb_idx, ub_idx, lbA_idx, ubA_idx, x0_idx=-1, options_idx=-1;

	double *guessedBoundsAndConstraints = 0;
	double *guessedBounds = 0, *guessedConstraints = 0;
	int guessedBoundsAndConstraintsIndex = -1;

    /* Setup default options */
	Options options;
	options.printLevel = PL_LOW;
	#ifdef __DEBUG__
	options.printLevel = PL_HIGH;
	#endif
	#ifdef __SUPPRESSANYOUTPUT__
	options.printLevel = PL_NONE;
	#endif

	/* dimensions */
	unsigned int nV=0, nC=0, nP=0;

	/* sparse matrix indices and values */
	sparse_int_t *Hdiag = 0, *Hir = 0, *Hjc = 0, *Air = 0, *Ajc = 0;
	real_t *Hv = 0, *Av = 0;

	/* I) CONSISTENCY CHECKS: */
	/* 1a) Ensure that qpOASES is called with a feasible number of input arguments. */
	if ( ( nrhs < 4 ) || ( nrhs > 10 ) )
	{
		myMexErrMsgTxt( "ERROR (qpOASES): Invalid number of input arguments!\nType 'help qpOASES' for further information." );
		return;
	}
    
    /* 1b) Warn when call might be ambiguous. */
    if ( ( nrhs == 7 ) && ( mxIsEmpty(prhs[4]) ) && ( mxIsEmpty(prhs[5]) ) && ( mxIsEmpty(prhs[6]) ) )
    {
        mexWarnMsgTxt( "Consider skipping empty input arguments to make call unambiguous!\n         Type 'help qpOASES' for further information." );
    }

	/* 2) Check for proper number of output arguments. */
	if ( nlhs > 6 )
	{
		myMexErrMsgTxt( "ERROR (qpOASES): At most six output arguments are allowed: \n    [x,fval,exitflag,iter,lambda,workingSet]!" );
		return;
	}
	if ( nlhs < 1 )
	{
		myMexErrMsgTxt( "ERROR (qpOASES): At least one output argument is required: [x,...]!" );
		return;
	}


	/* II) PREPARE RESPECTIVE QPOASES FUNCTION CALL: */
	/*     Choose between QProblem and QProblemB object and assign the corresponding
	 *     indices of the input pointer array in to order to access QP data correctly. */
	H_idx = 0;
	g_idx = 1;
	nV = (int)mxGetM( prhs[ H_idx ] ); /* row number of Hessian matrix */
	nP = (int)mxGetN( prhs[ g_idx ] ); /* number of columns of the gradient matrix (vectors series have to be stored columnwise!) */

	/* 0) Check whether options are specified .*/
	for (int i = 4; i < nrhs; i++) {
		if ((!mxIsEmpty(prhs[i])) && (mxIsStruct(prhs[i]))) {
			options_idx = i;
			break;
		}
	}

	// Is the third argument constraint Matrix A?
	int numberOfColumns = (int)mxGetN(prhs[2]);

	/* 1) Simply bounded QP. */
	if ( ( isSimplyBoundedQp( nrhs,prhs,nV,options_idx ) == BT_TRUE ) ||
		 ( ( numberOfColumns == 1 ) && ( nV != 1 ) ) )
	{
		lb_idx   = 2;
		ub_idx   = 3;

		if ( nrhs >= 6 ){ /* x0 specified */
			x0_idx = 5;
			if ( (nrhs >= 7) && (!mxIsEmpty(prhs[6])) ){
				guessedBoundsAndConstraintsIndex = 6;
			} else {
				guessedBoundsAndConstraintsIndex = -1;
			}
		}
		else {
			x0_idx = -1;
			guessedBoundsAndConstraintsIndex = -1;
		}
	}
	else
	{
		A_idx = 2;

		/* If constraint matrix is empty, use a QProblemB object! */
		if ( mxIsEmpty( prhs[ A_idx ] ) )
		{
			lb_idx   = 3;
			ub_idx   = 4;

			if ( nrhs >= 9 ) /* x0 specified */
				x0_idx = 8;
			else
				x0_idx = -1;

			/* guesses for bounds and constraints specified? */
			if ( (nrhs >= 10) && (!mxIsEmpty(prhs[9])) ) {
				guessedBoundsAndConstraintsIndex = 9;
			} else {
				guessedBoundsAndConstraintsIndex = -1;
			}
		}
		else
		{
			lb_idx   = 3;
			ub_idx   = 4;
			lbA_idx  = 5;
			ubA_idx  = 6;

			if ( nrhs >= 9 ) /* x0 specified */
				x0_idx = 8;
			else
				x0_idx = -1;

			/* guesses for bounds and constraints specified? */
			if ( (nrhs >= 10) && (!mxIsEmpty(prhs[9])) ){
				guessedBoundsAndConstraintsIndex = 9;
			} else {
				guessedBoundsAndConstraintsIndex = -1;
			}

			nC = (int)mxGetM( prhs[ A_idx ] ); /* row number of constraint matrix */
		}
	}


	/* III) ACTUALLY PERFORM QPOASES FUNCTION CALL: */
	int nWSRin = 5*(nV+nC);
	if ( options_idx > 0 )
		setupOptions( &options,prhs[options_idx],nWSRin );

	/* ensure that data is given in real_t precision */
	if ( ( mxIsDouble( prhs[ H_idx ] ) == 0 ) ||
		 ( mxIsDouble( prhs[ g_idx ] ) == 0 ) )
	{
		myMexErrMsgTxt( "ERROR (qpOASES): All data has to be provided in double precision!" );
		return;
	}

	/* Check inputs dimensions and assign pointers to inputs. */
	if ( mxGetN( prhs[ H_idx ] ) != nV )
	{
		char msg[200]; 
		snprintf(msg, 199, "ERROR (qpOASES): Hessian matrix input dimension mismatch (%ld != %d)!", 
				(long int)mxGetN(prhs[H_idx]), nV);
		//fprintf(stderr, "%s\n", msg);
		myMexErrMsgTxt(msg);
		return;
	}

	/* check for sparsity */
	if ( mxIsSparse( prhs[ H_idx ] ) != 0 )
	{
		mwIndex *mat_ir = mxGetIr(prhs[H_idx]);
		mwIndex *mat_jc = mxGetJc(prhs[H_idx]);
		double *v = (double*)mxGetPr(prhs[H_idx]);
		sparse_int_t nfill = 0;
		mwIndex i, j;

		/* copy indices to avoid 64/32-bit integer confusion */
		/* also add explicit zeros on diagonal for regularization strategy */
		/* copy values, too */
		Hir = new sparse_int_t[mat_jc[nV] + nV];
		Hjc = new sparse_int_t[nV+1];
		Hv = new real_t[mat_jc[nV] + nV];
		for (j = 0; j < nV; j++) 
		{
			Hjc[j] = (sparse_int_t)(mat_jc[j]) + nfill;
			/* fill up to diagonal */
			for (i = mat_jc[j]; i < mat_jc[j+1] && mat_ir[i] <= j; i++) 
			{
				Hir[i + nfill] = (sparse_int_t)(mat_ir[i]);
				Hv[i + nfill] = (real_t)(v[i]);
			}
			/* possibly add zero diagonal element */
			if (i >= mat_jc[j+1] || mat_ir[i] > j)
			{
				Hir[i + nfill] = (sparse_int_t)j;
				Hv[i + nfill] = 0.0;
				nfill++;
			}
			/* fill up to diagonal */
			for (; i < mat_jc[j+1]; i++) 
			{
				Hir[i + nfill] = (sparse_int_t)(mat_ir[i]);
				Hv[i + nfill] = (real_t)(v[i]);
			}
		}
		Hjc[nV] = (sparse_int_t)(mat_jc[nV]) + nfill;

		SymSparseMat *sH;
		H = sH = new SymSparseMat(nV, nV, Hir, Hjc, Hv);
		Hdiag = sH->createDiagInfo();
	}
	else
	{
		/* make a hard-copy in order to avoid modifying input data */
		H_for = (real_t*) mxGetPr( prhs[H_idx] );
		H_mem = new real_t[nV*nV];
		memcpy( H_mem,H_for, nV*nV*sizeof(real_t) );

		H = new SymDenseMat( nV,nV,nV, H_mem );
		H->doFreeMemory( );
	}

	if ( smartDimensionCheck( &g,nV,nP, BT_FALSE,prhs,g_idx ) != SUCCESSFUL_RETURN )
		return;

	if ( smartDimensionCheck( &lb,nV,nP, BT_TRUE,prhs,lb_idx ) != SUCCESSFUL_RETURN )
		return;

	if ( smartDimensionCheck( &ub,nV,nP, BT_TRUE,prhs,ub_idx ) != SUCCESSFUL_RETURN )
		return;

	if ( smartDimensionCheck( &x0,nV,1, BT_TRUE,prhs,x0_idx ) != SUCCESSFUL_RETURN )
		return;

	if (guessedBoundsAndConstraintsIndex != -1) {
		if (smartDimensionCheck(&guessedBoundsAndConstraints, nV + nC, 1,
				BT_TRUE, prhs, guessedBoundsAndConstraintsIndex)
				!= SUCCESSFUL_RETURN) {
			return;
		}
	}


	if (guessedBoundsAndConstraintsIndex != -1) {
		guessedBounds = new double[nV];
		for (unsigned int i = 0; i < nV; i++) {
			guessedBounds[i] = guessedBoundsAndConstraints[i];
		}

		if ( nC > 0 )
		{
			guessedConstraints = new double[nC];
			for (unsigned int i = 0; i < nC; i++) {
				guessedConstraints[i] = guessedBoundsAndConstraints[i + nV];
			}
		}
	}


	if ( nC > 0 )
	{
		/* ensure that data is given in real_t precision */
		if ( mxIsDouble( prhs[ A_idx ] ) == 0 )
		{
			myMexErrMsgTxt( "ERROR (qpOASES): All data has to be provided in real_t precision!" );
			return;
		}

		/* Check inputs dimensions and assign pointers to inputs. */
		if ( mxGetN( prhs[ A_idx ] ) != nV )
		{
			char msg[200]; 
			snprintf(msg, 199, "ERROR (qpOASES): Constraint matrix input dimension mismatch (%ld != %d)!", 
					(long int)mxGetN(prhs[A_idx]), nV);
			//fprintf(stderr, "%s\n", msg);
			myMexErrMsgTxt(msg);
			return;
		}

		A_for = (real_t*) mxGetPr( prhs[ A_idx ] );

		if ( smartDimensionCheck( &lbA,nC,nP, BT_TRUE,prhs,lbA_idx ) != SUCCESSFUL_RETURN )
			return;

		if ( smartDimensionCheck( &ubA,nC,nP, BT_TRUE,prhs,ubA_idx ) != SUCCESSFUL_RETURN )
			return;

		/* Check for sparsity. */
		if ( mxIsSparse( prhs[ A_idx ] ) != 0 )
		{
			mwIndex i;
			long j;

			mwIndex *mat_ir = mxGetIr(prhs[A_idx]);
			mwIndex *mat_jc = mxGetJc(prhs[A_idx]);
			double *v = (double*)mxGetPr(prhs[A_idx]);

			/* copy indices to avoid 64/32-bit integer confusion */
			Air = new sparse_int_t[mat_jc[nV]];
			Ajc = new sparse_int_t[nV+1];
			for (i = 0; i < mat_jc[nV]; i++) Air[i] = (sparse_int_t)(mat_ir[i]);
			for (i = 0; i < nV + 1; i++) Ajc[i] = (sparse_int_t)(mat_jc[i]);

			/* copy values, too */
			Av = new real_t[Ajc[nV]];
			for (j = 0; j < Ajc[nV]; j++) Av[j] = (real_t)(v[j]);

			A = new SparseMatrix(nC, nV, Air, Ajc, Av);
		}
		else
		{
			/* Convert constraint matrix A from FORTRAN to C style
			 * (not necessary for H as it should be symmetric!). */
			A_mem = new real_t[nC*nV];
			convertFortranToC( A_for,nV,nC, A_mem );
			A = new DenseMatrix(nC, nV, nV, A_mem );
            A->doFreeMemory();
		}
	}

	allocateOutputs( nlhs,plhs,nV,nC,nP );

	if ( nC == 0 )
	{
		/* call qpOASES (using QProblemB class). */
		qpOASESmex_bounds(	nV,nP,
							H,g,
							lb,ub,
							nWSRin,x0,&options,
							nlhs,plhs,
							guessedBounds
							);

		if (guessedBounds) delete[] guessedBounds;
        delete H;
		if (Hdiag) delete[] Hdiag;
		if (Hv) delete[] Hv;
		if (Hjc) delete[] Hjc;
		if (Hir) delete[] Hir;
		return;
	}
	else
	{
		/* Call qpOASES (using QProblem class). */
		qpOASESmex_constraints(	nV,nC,nP,
								H,g,A,
								lb,ub,lbA,ubA,
								nWSRin,x0,&options,
								nlhs,plhs,
								guessedBounds, guessedConstraints
								);

		if (guessedBounds != 0) delete[] guessedBounds;
		if (guessedConstraints != 0) delete[] guessedConstraints;
		delete A;
		delete H;
		if (Av != 0) delete[] Av;
		if (Ajc != 0) delete[] Ajc;
		if (Air != 0) delete[] Air;
		if (Hdiag != 0) delete[] Hdiag;
		if (Hv != 0) delete[] Hv;
		if (Hjc != 0) delete[] Hjc;
		if (Hir != 0) delete[] Hir;
		return;
	}
}

/*
 *	end of file
 */

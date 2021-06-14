/* -------------------------------------------------------------------------- *
 *
 * Compute the COD  A = U [ L 0 ] V
 *                        [ 0 0 ]
 * and provide the ls solver x = A^+ b.
 *
 * -------------------------------------------------------------------------- */

#ifndef __SOTH_COD__
#define __SOTH_COD__

#include <Eigen/SVD>
#include "../include/soth/Algebra.hpp"
#include "../include/soth/Givens.hpp"
#include "../include/soth/debug.hpp"

namespace soth {
/* ---------------------------------------------------------------------------
 */
/* --- COD SOLVER ------------------------------------------------------------
 */
/* ---------------------------------------------------------------------------
 */
/* Compute the decomposition A=U.L.V', with U and L rotation matrices, and
 * L = [ L0 0 ; 0 0 ] with L0 triangular inf, with non zero diagonal.
 * Use this decompo to solve min||Ax-b||.
 */
struct COD {
  MatrixXd L, Ainit;
  ColPivHouseholderQR<MatrixXd> qrv;
  int rank, NC, NR;
  MatrixXd U, V;
  bool m_computeFullU, m_computeThinU, m_computeFullV, m_computeThinV;

  COD(unsigned int computationOptions = 0) { options(computationOptions); }
  void options(unsigned int computationOptions = 0) {
    m_computeFullU = (computationOptions & ComputeFullU) != 0;
    m_computeThinU = (computationOptions & ComputeThinU) != 0;
    m_computeFullV = (computationOptions & ComputeFullV) != 0;
    m_computeThinV = (computationOptions & ComputeThinV) != 0;
  }

  /* Nullify row k of L using the first rows as a lower triangle, supposing that
   * LX = [ L ; X ], L of size MxM and lower triangular and k>M. */
  void leftGivens(const int k) {
    for (int j = rank - 1; j >= 0; --j) {
      Givens G1(L.col(j), j, k);
      G1.transpose() >> L;
      if (m_computeFullU || m_computeThinU) {
        U << G1;
      }
      sotDEBUG(5) << "LX" << j << " = " << (MATLAB)L << std::endl << std::endl;
    }
  }

  int computeRank(int rank, const double EPSILON = 1e-8) {
    if (rank > 0) return rank;

    const MatrixXd& QR = qrv.matrixQR();
    const int R0 = std::min(NR, NC);
    for (rank = R0; rank > 0; --rank) {
      if (fabs(QR(rank - 1, rank - 1)) > EPSILON) break;
    }
    return rank;
  }

  void compute(const MatrixXd& A, const int rank_ = -1,
               const double EPSILON = 1e-8) {
    NC = (int)A.cols();
    NR = (int)A.rows();
#ifdef DEBUG
    Ainit = A;
#endif

    qrv.compute(A.transpose());
    sotDEBUG(5) << "QR= " << (MATLAB)qrv.matrixQR() << std::endl;

    rank = computeRank(rank_, EPSILON);
    L = qrv.matrixQR().topRows(rank).triangularView<Upper>().transpose();
    sotDEBUG(5) << "L = " << (MATLAB)L << std::endl;

    if (m_computeFullV) {
      V.setIdentity(NC, NC);
      V.applyOnTheRight(qrv.householderQ());
      sotDEBUG(5) << "V = " << (MATLAB)V << std::endl;
    } else if (m_computeThinV) {
      V.resize(NC, rank);
      V.setIdentity(NC, rank);
      V.applyOnTheLeft(qrv.householderQ());
    }

    if (m_computeFullU || m_computeThinU) {
      U = qrv.colsPermutation();
      sotDEBUG(5) << "Pi = " << (MATLAB)U << std::endl;
    }

    for (int k = rank; k < NR; ++k) leftGivens(k);

    sotDEBUG(5) << "L = " << (MATLAB)L << std::endl;
    sotDEBUG(5) << "U = " << (MATLAB)U << std::endl;
  }

  /* Compute M:=M*V */
  template <typename D>
  void applyMatrixV(MatrixBase<D>& M) {
    M.applyOnTheRight(qrv.householderQ());
  }

  MatrixXd& matrixU() { return U; }
  MatrixXd& matrixV() { return V; }
  MatrixXd::ColsBlockXpr matrixUr() { return U.leftCols(rank); }
  MatrixXd::ColsBlockXpr matrixVr() { return V.leftCols(rank); }
  MatrixXd::ColsBlockXpr matrixUo() { return U.rightCols(NR - rank); }
  MatrixXd::ColsBlockXpr matrixVo() { return V.rightCols(NC - rank); }
  TriangularView<Block<MatrixXd>, Lower> matrixL() {
    return L.topRows(rank).triangularView<Lower>();
  }

  /* Solve min||Ax-b|| for a matrix A whose rank is given. */
  VectorXd solve(const VectorXd& b, bool inY = false) {
    if (rank == 0) return VectorXd::Zero(NC);

    /* Approximate solution using no basis transfo (result is meanigless
     * appart from the computation time pov. */
    /*
      VectorXd sol = b.head(rank);
      matrixL().solveInPlace( sol );
      VectorXd res; res.setZero(NC);
      res.head(rank)=sol; return res;
    */

    /* With plain matrices. */
    /*
      VectorXd sol = matrixUr().transpose()*b;
      matrixL().solveInPlace( sol );
      return matrixVr()*sol;
    */

    /* Using the HH representation of V. */
    assert(m_computeThinU || m_computeFullU);
    VectorXd sol;
    if (inY)
      sol.setZero(rank);
    else
      sol.setZero(NC);
    sol.head(rank) = matrixUr().transpose() * b;
    matrixL().solveInPlace(sol.head(rank));
    if (!inY) sol.applyOnTheLeft(qrv.householderQ());
    return sol;
  }
};

}  // namespace soth

#endif  // #ifndef __SOTH_COD__

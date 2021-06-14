/* -------------------------------------------------------------------------- *
 *
 * Simple test of the givens class, with assertion.
 *
 * -------------------------------------------------------------------------- */

#define SOTH_DEBUG
#define SOTH_DEBUG_MODE 50

#include "soth/debug.hpp"

#include <iostream>
#include "soth/Algebra.hpp"
#include "soth/Givens.hpp"

using namespace Eigen;
using namespace soth;

typedef Matrix<MatrixXd::Index, Dynamic, 1> IndexType;

void testSimpleGivens() {
  using std::cout;
  using std::endl;

  MatrixXd M = MatrixXd::Random(4, 6);
  const double n0 = M.col(0).norm();
  UNUSED(n0);
  sotDEBUG(1) << "M = " << (MATLAB)M << endl;

  sotDEBUG(1) << "Nullify col 0 ...";
  Givens G1(M.col(0), 2, 3);
  sotDEBUG(1) << "G1 = " << G1.G.c() << "," << G1.G.s() << endl;
  G1.applyTransposeOnTheRight(M);
  // G1.applyTransposeOnTheLeft(M);
  sotDEBUG(1) << "G1'*M = " << (MATLAB)M << endl;

  Givens G2(M(1, 0), M(2, 0), 1, 2);
  G2.applyTransposeOnTheRight(M);
  sotDEBUG(1) << "G2'*G1'*M = " << (MATLAB)M << endl;

  Givens G3(M(0, 0), M(1, 0), 0, 1);
  G3.transpose() >> M;
  sotDEBUG(1) << "G3'*G2'*G1'*M = " << (MATLAB)M << endl;

  const double n1 = M.row(0).tail(M.cols() - 1).norm();
  UNUSED(n1);
  sotDEBUG(1) << endl << "Nullify row 0 ..." << endl;
  for (int i = (int)M.cols() - 1; i > 1; --i) {
    // Givens G(M.row(0), i-1, i);
    M << Givens(M.row(0), i - 1, i);
    sotDEBUG(1) << "G1:3'*M*Prod(G)" << (MATLAB)M << endl;
  }

  cout << "Checking the application on the cols of MatrixXd ... " << std::flush;
  assert(M.col(0).tail(M.rows() - 1).norm() < 1e-6);
  assert(std::abs(M.col(0).norm() - n0) < 1e-6);
  cout << " ... OK! " << endl;
  cout << "Checking the application on the rows of MatrixXd ... " << std::flush;
  assert(M.row(0).tail(M.rows() - 2).norm() < 1e-6);
  assert(std::abs(M.row(0).tail(M.cols() - 1).norm() - n1) < 1e-6);
  cout << " ... OK! " << endl;
}

#include "soth/SubMatrix.hpp"
void testSubMatrixGivens() {
  using std::cout;
  using std::endl;

  MatrixXd A = MatrixXd::Random(4, 6);
  sotDEBUG(1) << "A = " << (MATLAB)A << endl;

  IndexType idxr(4);
  idxr << 3, 1, 0, 2;
  IndexType idxc(6);
  idxc << 3, 5, 0, 1, 2, 4;
  SubMatrix<MatrixXd> M(A, idxr, idxc);
  sotDEBUG(1) << "M = " << (MATLAB)M << endl;

  const double n0 = M.col(0).norm();
  UNUSED(n0);

  sotDEBUG(1) << "Nullify col 0 ..." << endl;
  Givens G1(M.col(0), 2, 3);
  G1.applyTransposeOnTheRight(M);
  sotDEBUG(1) << "G1'*M = " << (MATLAB)M << endl;

  Givens G2(M.col(0), 1, 2);
  G2.applyTransposeOnTheRight(M);
  sotDEBUG(1) << "G2'*G1'*M = " << (MATLAB)M << endl;

  Givens G3(M(0, 0), M(1, 0), 0, 1);
  G3.applyTransposeOnTheRight(M);
  sotDEBUG(1) << "G3'*G2'*G1'*M = " << (MATLAB)M << endl;

  const double n1 = M.row(0).tail(M.cols() - 1).norm();
  UNUSED(n1);
  sotDEBUG(1) << endl << "Nullify row 0 ..." << endl;
  for (int i = (int)M.cols() - 1; i > 1; --i) {
    M << Givens(M.row(0), i - 1, i);
    // Givens (M.row(0), i-1, i).applyThisOnTheLeft(M);
    sotDEBUG(1) << "G1:3'*M*Prod(G)" << (MATLAB)M << endl;
  }

  cout << "Checking the application on the cols of a submatrix ... "
       << std::flush;
  assert(M.col(0).tail(M.rows() - 1).norm() < 1e-6);
  assert(std::abs(M.col(0).norm() - n0) < 1e-6);
  cout << " ... OK! " << endl;
  cout << "Checking the application on the rows of a submatrix ... "
       << std::flush;
  assert(M.row(0).tail(M.rows() - 2).norm() < 1e-6);
  assert(std::abs(M.row(0).tail(M.cols() - 1).norm() - n1) < 1e-6);
  cout << " ... OK! " << endl;
}

void testSequenceSub() {
  using std::cout;
  using std::endl;

  MatrixXd A = MatrixXd::Random(4, 6);
  sotDEBUG(1) << "A = " << (MATLAB)A << endl;

  IndexType idxr(4);
  idxr << 3, 1, 0, 2;
  IndexType idxc(6);
  idxc << 3, 5, 0, 1, 2, 4;
  SubMatrix<MatrixXd> M(A, idxr, idxc);
  sotDEBUG(1) << "M = " << (MATLAB)M << endl;
  MatrixXd Msav = M;
  GivensSequence U, V;

  Givens G1(M.col(0), 2, 3);
  G1.applyTransposeOnTheRight(M);
  U.push(G1);

  Givens G2(M.col(0), 1, 2);
  G2.applyTransposeOnTheRight(M);
  U.push(G2);

  Givens G3(M(0, 0), M(1, 0), 0, 1);
  G3.applyTransposeOnTheRight(M);
  U.push(G3);

  Givens Gr;
  for (int i = (int)M.cols() - 1; i > 1; --i) {
    Gr.makeGivens(M.row(0), i - 1, i);
    Gr.applyThisOnTheLeft(M);
    V.push(Gr);
  }

  U.transpose() >> Msav;
  Msav << V;
  sotDEBUG(1) << "Msav = " << (MATLAB)Msav << endl;

  cout << "Checking the sequence of givens ... " << std::flush;
  assert((M - Msav).norm() < 1e-6);
  cout << " ... OK! " << endl;
}

int main() {
  sotDebugTrace::openFile();
  testSimpleGivens();
  testSubMatrixGivens();
  testSequenceSub();
}

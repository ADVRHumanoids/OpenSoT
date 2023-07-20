/* -------------------------------------------------------------------------- *
 *
 * Simplification and measure of cone volum.
 *
 * -------------------------------------------------------------------------- */
#define SOTH_DEBUG
#define SOTH_DEBUG_MODE 45
#include "../include/soth/HCOD.hpp"
#include "../include/soth/debug.hpp"

#ifndef WIN32
#include <sys/time.h>
#endif  // WIN32

#include <iostream>
#include <vector>
#include "../unitTesting/gettimeofday.hpp"

using namespace soth;
using std::cerr;
using std::cout;
using std::endl;

/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */

#include <boost/program_options.hpp>

/* Solve an optimisation problem to check if the column <index> of A is linked
 * to the rest of A and B, under the constraint of positivity of A
 * coefficients. Return true if the column is linked, false otherwise. */

template <typename D>
bool checkColumn(const Eigen::MatrixBase<D> &A, const Eigen::MatrixBase<D> &B,
                 const int index, int sign = +1) {
  const int NC = (int)A.rows(), NA = (int)A.cols(), NB = (int)B.cols(),
            NX = (int)(NA + NB);

  std::vector<Eigen::MatrixXd> J(3);
  std::vector<soth::VectorBound> b(3);

  /* SOTH structure construction. */
  soth::HCOD hcod(NX, 3);

  J[0].resize(NA, NX);
  b[0].resize(NA);
  J[0].leftCols(NA).setIdentity();
  J[0].rightCols(NB).fill(0);
  b[0].fill(soth::Bound(0, soth::Bound::BOUND_INF));
  b[0][index] = soth::Bound(-10, soth::Bound::BOUND_INF);
  hcod.pushBackStage(J[0], b[0]);

  J[1].resize(NC, NA + NB);
  b[1].resize(NC);
  J[1].leftCols(NA) = A;
  J[1].rightCols(NB) = B;
  b[1].fill(0);
  hcod.pushBackStage(J[1], b[1]);

  J[2].resize(1, NA + NB);
  b[2].resize(1);
  J[2].fill(0);
  J[2](0, index) = 1;
  b[2].fill(-sign);
  hcod.pushBackStage(J[2], b[2]);

  hcod.setNameByOrder("stage_");

  hcod.useDamp(false);
  hcod.setInitialActiveSet();

  VectorXd solution(NX);
  hcod.activeSearch(solution);
  if (sotDEBUGFLOW.outputbuffer.good()) hcod.show(sotDEBUGFLOW.outputbuffer);
  // cout << "x = " << (MATLAB)solution << endl;
  // cout << "actset = "; hcod.showActiveSet(std::cout);
  // cout << "res = " << solution[index]+1 << endl;

  cout << "Checking const " << (sign > 0 ? "+" : "-") << index << "... ";
  if (std::abs(solution[index] + sign) < 1e-6)
    cout << "     \tuseless" << endl;
  else
    cout << "      \tneeded" << endl;

  return std::abs(solution[index] + sign) < 1e-6;
}

typedef SubMatrix<MatrixXd>::ColIndices Indirect;

int checkAndModify(SubMatrix<MatrixXd, ColPermutation> &A,
                   SubMatrix<MatrixXd, ColPermutation> &B, int index) {
  // cout << "A = " << (MATLAB)A << endl;
  // cout << "B = " << (MATLAB)B << endl;

  if (checkColumn(A, B, index, +1)) {
    A.removeCol(index);
    cout << "remove ... " << endl;
    return 0;
  }
  if (checkColumn(A, B, index, -1)) {
    int ref = (int)A.removeCol(index);
    B.pushColFront(ref);
    cout << "transfert ... " << endl;
    return 0;
  }

  return 1;
}

/* Check if all AB can be expressed as X (ie conservative solution). */
bool checkOut(const MatrixXd &X, const SubMatrix<MatrixXd, ColPermutation> &A,
              const SubMatrix<MatrixXd, ColPermutation> &B) {
  /* shoot ab = rand, with a>0 and target=AB*ab
   * solve X*x=target, with x>0. */

  const int NC = (int)A.rows(), NA = (int)A.cols(), NB = (int)B.cols(),
            NX = (int)X.cols();
  VectorXd ab = VectorXd::Random(NA + NB);
  ab.head(NA) += VectorXd::Ones(NA);
  ab.head(NA) /= 2;
  VectorXd target = A * ab.head(NA) + B * ab.tail(NB);
  cout << "ab = " << (MATLAB)ab << endl;
  cout << "target = " << (MATLAB)target << endl;

  std::vector<Eigen::MatrixXd> J(2);
  std::vector<soth::VectorBound> b(2);

  /* SOTH structure construction. */
  soth::HCOD hcod(NX, 2);

  J[0].resize(NX, NX);
  b[0].resize(NX);
  J[0].setIdentity();
  b[0].fill(soth::Bound(0, soth::Bound::BOUND_INF));
  hcod.pushBackStage(J[0], b[0]);

  J[1].resize(NC, NX);
  b[1].resize(NC);
  J[1] = X;
  for (int i = 0; i < NC; ++i) b[1][i] = target[i];
  hcod.pushBackStage(J[1], b[1]);

  hcod.setNameByOrder("stage_");

  hcod.useDamp(false);
  hcod.setInitialActiveSet();

  VectorXd solution(NX);
  hcod.activeSearch(solution);
  if (sotDEBUGFLOW.outputbuffer.good()) hcod.show(sotDEBUGFLOW.outputbuffer);

  cout << "ab = " << (MATLAB)solution << std::endl;
  cout << "res = " << (MATLAB)(VectorXd)(X * solution - target) << std::endl;

  return (X * solution - target).norm() < 1e-6;
}

/* Check if all X can be expressed as AB. */
bool checkIn(const MatrixXd &X, const SubMatrix<MatrixXd, ColPermutation> &A,
             const SubMatrix<MatrixXd, ColPermutation> &B) {
  /* shoot x = rand>0, and target=X*x
   * solve Aa+Bb = x, with a>0. */

  const int NC = (int)A.rows(), NA = (int)A.cols(), NB = (int)B.cols(),
            NX = (int)X.cols();
  VectorXd x = VectorXd::Random(NX);
  x += VectorXd::Ones(NX);
  x /= 2;
  VectorXd target = X * x;
  cout << "x = " << (MATLAB)x << endl;

  std::vector<Eigen::MatrixXd> J(2);
  std::vector<soth::VectorBound> b(2);

  /* SOTH structure construction. */
  soth::HCOD hcod(NA + NB, 2);

  J[0].resize(NA, NA + NB);
  b[0].resize(NA);
  J[0].leftCols(NA).setIdentity();
  J[0].rightCols(NB).fill(0);
  b[0].fill(soth::Bound(0, soth::Bound::BOUND_INF));
  hcod.pushBackStage(J[0], b[0]);

  J[1].resize(NC, NA + NB);
  b[1].resize(NC);
  J[1].leftCols(NA) = A;
  J[1].rightCols(NB) = B;
  for (int i = 0; i < NC; ++i) b[1][i] = target[i];
  hcod.pushBackStage(J[1], b[1]);

  hcod.setNameByOrder("stage_");

  hcod.useDamp(false);
  hcod.setInitialActiveSet();

  VectorXd solution(NA + NB);
  hcod.activeSearch(solution);
  if (sotDEBUGFLOW.outputbuffer.good()) hcod.show(sotDEBUGFLOW.outputbuffer);

  cout << "ab = " << (MATLAB)solution << std::endl;
  cout << "res = " << (MATLAB)(VectorXd)(J[1] * solution - target) << std::endl;

  return (J[1] * solution - target).norm() < 1e-6;
}

int main(int, char **) {
#ifndef NDEBUG
  sotDebugTrace::openFile();
#endif

  const int NC = 6, NX = 40;

  // Eigen::MatrixXd X = MatrixXd::Random(NC,NX);
  //   Eigen::MatrixXd X(NC,NX);
  //   X <<    1.00000, 1.00000, 1.00000, 1.00000, 0.00000, 0.00000, 0.00000,
  //   0.00000, -1.00000, -1.00000, -1.00000, -1.00000, -0.00000, -0.00000,
  //   -0.00000, -0.00000, 0.00000, 0.00000, 0.00000, 0.00000
  // , 0.00000, 0.00000, 0.00000, 0.00000, 1.00000, 1.00000, 1.00000, 1.00000,
  // -0.00000, -0.00000, -0.00000, -0.00000, -1.00000, -1.00000, -1.00000,
  // -1.00000, 0.00000, 0.00000, 0.00000, 0.00000 , 0.00000, 0.00000, 0.00000,
  // 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, -0.00000, -0.00000, -0.00000,
  // -0.00000, -0.00000, -0.00000, -0.00000, -0.00000, 1.00000,
  // 1.00000, 1.00000, 1.00000 , 0.00000, 0.00000, 0.00000, 0.00000, -0.10500,
  // -0.10500, -0.10500, -0.10500, -0.00000, -0.00000, -0.00000, -0.00000,
  // 0.10500, 0.10500, 0.10500, 0.10500, 0.04500, 0.04500, -0.07000, -0.07000 ,
  // 0.10500, 0.10500, 0.10500, 0.10500, 0.00000, 0.00000, 0.00000, 0.00000,
  // -0.10500, -0.10500, -0.10500, -0.10500, -0.00000, -0.00000, -0.00000,
  // -0.00000, 0.11000, -0.08000, -0.08000, 0.11000 , -0.04500, -0.04500,
  // 0.07000, 0.07000, -0.11000, 0.08000, 0.08000, -0.11000, 0.04500, 0.04500,
  // -0.07000, -0.07000, 0.11000, -0.08000, -0.08000, 0.11000, 0.00000, 0.00000,
  // 0.00000, 0.00000;

  Eigen::MatrixXd X(NC, NX);
  X << 8.0902e-01, 3.0902e-01, -3.0902e-01, -8.0902e-01, -1.0000e+00,
      -8.0902e-01, -3.0902e-01, 3.0902e-01, 8.0902e-01, 1.0000e+00, 8.0902e-01,
      3.0902e-01, -3.0902e-01, -8.0902e-01, -1.0000e+00, -8.0902e-01,
      -3.0902e-01, 3.0902e-01, 8.0902e-01, 1.0000e+00, 8.0902e-01, 3.0902e-01,
      -3.0902e-01, -8.0902e-01, -1.0000e+00, -8.0902e-01, -3.0902e-01,
      3.0902e-01, 8.0902e-01, 1.0000e+00, 8.0902e-01, 3.0902e-01, -3.0902e-01,
      -8.0902e-01, -1.0000e+00, -8.0902e-01, -3.0902e-01, 3.0902e-01,
      8.0902e-01, 1.0000e+00, 5.8779e-01, 9.5106e-01, 9.5106e-01, 5.8779e-01,
      1.2246e-16, -5.8779e-01, -9.5106e-01, -9.5106e-01, -5.8779e-01,
      -2.4492e-16, 5.8779e-01, 9.5106e-01, 9.5106e-01, 5.8779e-01, 1.2246e-16,
      -5.8779e-01, -9.5106e-01, -9.5106e-01, -5.8779e-01, -2.4492e-16,
      5.8779e-01, 9.5106e-01, 9.5106e-01, 5.8779e-01, 1.2246e-16, -5.8779e-01,
      -9.5106e-01, -9.5106e-01, -5.8779e-01, -2.4492e-16, 5.8779e-01,
      9.5106e-01, 9.5106e-01, 5.8779e-01, 1.2246e-16, -5.8779e-01, -9.5106e-01,
      -9.5106e-01, -5.8779e-01, -2.4492e-16, 1.0000e+00, 1.0000e+00, 1.0000e+00,
      1.0000e+00, 1.0000e+00, 1.0000e+00, 1.0000e+00, 1.0000e+00, 1.0000e+00,
      1.0000e+00, 1.0000e+00, 1.0000e+00, 1.0000e+00, 1.0000e+00, 1.0000e+00,
      1.0000e+00, 1.0000e+00, 1.0000e+00, 1.0000e+00, 1.0000e+00, 1.0000e+00,
      1.0000e+00, 1.0000e+00, 1.0000e+00, 1.0000e+00, 1.0000e+00, 1.0000e+00,
      1.0000e+00, 1.0000e+00, 1.0000e+00, 1.0000e+00, 1.0000e+00, 1.0000e+00,
      1.0000e+00, 1.0000e+00, 1.0000e+00, 1.0000e+00, 1.0000e+00, 1.0000e+00,
      1.0000e+00, -1.6717e-02, -5.4861e-02, -5.4861e-02, -1.6717e-02,
      4.5000e-02, 1.0672e-01, 1.4486e-01, 1.4486e-01, 1.0672e-01, 4.5000e-02,
      -1.6717e-02, -5.4861e-02, -5.4861e-02, -1.6717e-02, 4.5000e-02,
      1.0672e-01, 1.4486e-01, 1.4486e-01, 1.0672e-01, 4.5000e-02, -1.3172e-01,
      -1.6986e-01, -1.6986e-01, -1.3172e-01, -7.0000e-02, -8.2825e-03,
      2.9861e-02, 2.9861e-02, -8.2825e-03, -7.0000e-02, -1.3172e-01,
      -1.6986e-01, -1.6986e-01, -1.3172e-01, -7.0000e-02, -8.2825e-03,
      2.9861e-02, 2.9861e-02, -8.2825e-03, -7.0000e-02, 1.9495e-01, 1.4245e-01,
      7.7553e-02, 2.5053e-02, 5.0000e-03, 2.5053e-02, 7.7553e-02, 1.4245e-01,
      1.9495e-01, 2.1500e-01, 4.9468e-03, -4.7553e-02, -1.1245e-01, -1.6495e-01,
      -1.8500e-01, -1.6495e-01, -1.1245e-01, -4.7553e-02, 4.9468e-03,
      2.5000e-02, 4.9468e-03, -4.7553e-02, -1.1245e-01, -1.6495e-01,
      -1.8500e-01, -1.6495e-01, -1.1245e-01, -4.7553e-02, 4.9468e-03,
      2.5000e-02, 1.9495e-01, 1.4245e-01, 7.7553e-02, 2.5053e-02, 5.0000e-03,
      2.5053e-02, 7.7553e-02, 1.4245e-01, 1.9495e-01, 2.1500e-01, -1.0106e-01,
      -1.1852e-01, -9.0710e-02, -2.8251e-02, 4.5000e-02, 1.0106e-01, 1.1852e-01,
      9.0710e-02, 2.8251e-02, -4.5000e-02, 1.0617e-02, 6.2179e-02, 8.9990e-02,
      8.3429e-02, 4.5000e-02, -1.0617e-02, -6.2179e-02, -8.9990e-02,
      -8.3429e-02, -4.5000e-02, 1.0365e-01, 9.7716e-02, 5.4453e-02, -9.6084e-03,
      -7.0000e-02, -1.0365e-01, -9.7716e-02, -5.4453e-02, 9.6084e-03,
      7.0000e-02, -8.0252e-03, -8.2985e-02, -1.2625e-01, -1.2129e-01,
      -7.0000e-02, 8.0252e-03, 8.2985e-02, 1.2625e-01, 1.2129e-01, 7.0000e-02;

  SubMatrix<MatrixXd, ColPermutation> A(X, true);
  SubMatrix<MatrixXd, ColPermutation> B(X);

  cout << "Ao = " << (MATLAB)A << endl;
  cout << "Bo = " << (MATLAB)B << endl;

  int ref = 0;
  for (int i = 0; i < NX; ++i) ref += checkAndModify(A, B, ref);

  cout << "A = " << (MATLAB)A << endl;
  cout << "B = " << (MATLAB)B << endl;

  // if( A.cols()>0 ) for( int i=0;i<1000;i++ ) checkIn(X,A,B);
  for (int i = 0; i < 1000; i++) checkOut(X, A, B);

  return 0;
}

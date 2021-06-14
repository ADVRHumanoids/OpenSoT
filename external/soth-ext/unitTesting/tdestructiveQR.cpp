/* -------------------------------------------------------------------------- *
 *
 * Exhaustive test of the DestructiveColPivQR class. Assertion should be added.
 *
 * -------------------------------------------------------------------------- */

#include <Eigen/Core>
#include <Eigen/Householder>
#include <Eigen/QR>
#include <iostream>
#include "soth/DestructiveColPivQR.hpp"

using namespace Eigen;

void testColPivForCod() {
  const int n = 7;
  const int m = 6;
  MatrixXd M;
  M = MatrixXd::Random(m, m - 2) * MatrixXd::Random(m - 2, n);
  MatrixXd Y = MatrixXd::Zero(n, n);

  std::cout << "initial matrix M: " << std::endl;
  std::cout << M << std::endl << std::endl << std::endl;

  std::cout << "***** Reference Decomposition of M' *****" << std::endl;
  ColPivHouseholderQR<MatrixXd> ref(M.transpose());
  MatrixXd resR = ref.matrixQR().triangularView<Upper>();
  MatrixXd resQ = ref.matrixQR().triangularView<StrictlyLower>();
  std::cout << "R_ref = " << std::endl;
  std::cout << resR << std::endl << std::endl;
  std::cout << "householder essential H_ref= " << std::endl;
  std::cout << resQ << std::endl << std::endl;

  std::cout << "***** Tested Decomposition of M' *****" << std::endl;
  Transpose<MatrixXd> Mt = M.transpose();
  Block<MatrixXd> Yse = Y.block(0, 0, n, m);
  DestructiveColPivQR<Transpose<MatrixXd>, Block<MatrixXd> > qr(Mt, Yse);
  std::cout << "R (in place, no transposition) = " << std::endl;
  std::cout << M.transpose() << std::endl << std::endl;
  std::cout << "permuted R = " << std::endl;
  std::cout << M.transpose() * qr.colsPermutation() << std::endl << std::endl;
  std::cout << "householder essential H= " << std::endl;
  std::cout << Y << std::endl << std::endl;
  std::cout << "QR = " << std::endl;
  std::cout << qr.householderQ() * M.transpose() << std::endl << std::endl;

  std::cout << std::endl << "***** Check *****" << std::endl;
  std::cout
      << "correct R (R_ref*P_ref' = R): "
      << ((resR * ref.colsPermutation().transpose() - M.transpose()).isZero()
              ? "true"
              : "false")
      << std::endl;
  std::cout << "correct householder (Href = H): "
            << ((resQ - Y.block(0, 0, n, m)).isZero() ? "true" : "false")
            << std::endl;
  std::cout << "correct permutations (P_ref*P = I): "
            << ((ref.colsPermutation().transpose() * qr.colsPermutation())
                        .toDenseMatrix()
                        .isIdentity()
                    ? "true"
                    : "false")
            << std::endl;
}

int main() {
  testColPivForCod();
  return 0;
}

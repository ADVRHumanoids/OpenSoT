/* -------------------------------------------------------------------------- *
 *
 * Unittest of the COD class, with time comparison with the Jacobi class. Could
 * be asserted.
 *
 * -------------------------------------------------------------------------- */
#define SOTH_DEBUG
#define SOTH_DEBUG_MODE 45
#include "soth/debug.hpp"

#include <Eigen/SVD>
#include <iostream>
#include "COD.hpp"
#include "gettimeofday.hpp"

using namespace std;
using namespace Eigen;
using namespace soth;

int main(void) {
#ifndef NDEBUG
  sotDebugTrace::openFile();
#endif
  {
    // int NR=90,NC=60,RANK=30;
    // int NR=100,NC=100,RANK=100;
    int NR = 20, NC = 40, RANK = 6;

    MatrixXd Xhi = MatrixXd::Random(NR, RANK);
    MatrixXd R = MatrixXd::Random(RANK, NC);
    MatrixXd A;
    A = Xhi * R;

    COD Acod;
    Acod.compute(A);

#ifndef NDEBUG
    cout << "A =" << (MATLAB)A << endl;
    std::cout << "rank = " << Acod.rank << std::endl;
    if (Acod.matrixU().cols() != 0)
      std::cout << "U = " << (MATLAB)Acod.matrixUr() << std::endl;
    else
      std::cout << "U = " << (MATLAB)Acod.matrixU() << std::endl;
    std::cout << "L = " << (MATLAB)(MatrixXd)Acod.matrixL() << std::endl;
    if (Acod.matrixV().cols() != 0)
      std::cout << "V = " << (MATLAB)Acod.matrixVr() << std::endl;
    else
      std::cout << "V = " << (MATLAB)Acod.matrixV() << std::endl;

    if ((Acod.matrixU().cols() != 0) && (Acod.matrixV().cols() != 0)) {
      std::cout << "ULV = "
                << (MATLAB)(MatrixXd)(Acod.matrixUr() * Acod.matrixL() *
                                      Acod.matrixVr().transpose())
                << std::endl;
      std::cout << "ERR = "
                << (A - Acod.matrixUr() * Acod.matrixL() *
                            Acod.matrixVr().transpose())
                       .norm()
                << std::endl;
    }
#endif

    /* --- CHRONO --- */
    struct timeval t0, t1;
    double totalTime = 0;
    double totalTimeSVD = 0;

    for (int shoot = 0; shoot < 1000; ++shoot) {
      Xhi = MatrixXd::Random(NR, RANK);
      R = MatrixXd::Random(RANK, NC);
      A = Xhi * R;

      gettimeofday(&t0, NULL);
      COD Acod;
      Acod.compute(A, RANK);
      gettimeofday(&t1, NULL);
      double time = (double)(t1.tv_sec - t0.tv_sec) +
                    (double)(t1.tv_usec - t0.tv_usec) / 1.0e6;
      totalTime += time;

      gettimeofday(&t0, NULL);
      // JacobiSVD<MatrixXd> Asvd(A, ComputeThinU | ComputeThinV);
      gettimeofday(&t1, NULL);

      double timeSVD = (double)(t1.tv_sec - t0.tv_sec) +
                       (double)(t1.tv_usec - t0.tv_usec) / 1.0e6;
      totalTimeSVD += timeSVD;
      if (!(shoot % 100))
        std::cout << time * 1000 << "\t" << timeSVD * 1000 << std::endl;
    }
    std::cout << totalTime << "\t" << totalTimeSVD << std::endl;
  }
  return 0;
}

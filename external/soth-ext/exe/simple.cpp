/* -------------------------------------------------------------------------- *
 *
 * IJRR 2013 tests: compare the cost of the HQP with the QP cascade of Kanoun
 * and DeLassa.
 *
 * -------------------------------------------------------------------------- */
#define SOTH_DEBUG
#define SOTH_DEBUG_MODE 45
#include "../unitTesting/COD.hpp"
#include "../unitTesting/RandomGenerator.hpp"
#include "../include/soth/DestructiveColPivQR.hpp"
#include "../include/soth/HCOD.hpp"
#include "../include/soth/Random.hpp"
#include "../include/soth/debug.hpp"

#include <boost/assign/std/vector.hpp>  // for 'operator+=()'
using namespace boost::assign;          // bring 'operator+=()' into scope

#ifndef WIN32
#include <sys/time.h>
#endif  // WIN32

#include <iostream>
#include <sstream>
#include "../unitTesting/gettimeofday.hpp"

using namespace soth;
using std::cerr;
using std::cout;
using std::endl;

/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */

void ehqp(HCOD& hsolver) {
  hsolver.initialize();
  // hsolver.Y.computeExplicitly();
  // hsolver.computeSolution();
  // hsolver.showActiveSet(std::cout);
}

/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */

int main() {
  unsigned int NB_STAGE, NC;
  std::vector<unsigned int> NR, RANKLINKED, RANKFREE;
  std::vector<Eigen::MatrixXd> J;
  std::vector<soth::VectorBound> b;

  soth::Random::setSeed(704819);
  //  const int size = 100;

  // generateFixedSizeRandomProfile(size,
  //                               1,0.99,0.99,NB_STAGE,RANKFREE,RANKLINKED,NR,NC);
  if (1) {
    NB_STAGE = 3;
    RANKFREE += 30, 40, 30;
    RANKLINKED += 0, 0, 0;
    NR = RANKFREE;
    NC = 100;
  }
  if (0) {
    NB_STAGE = 1;
    NC = 100;
    RANKFREE += NC;
    RANKLINKED += 0, 0, 0;
    NR = RANKFREE;
  }
  if (0) {
    RANKFREE += 6, 3, 7, 2, 5, 5, 44, 6, 2, 4, 5, 3, 7, 1, 50;
    RANKLINKED.resize(RANKFREE.size(), 0);
    NB_STAGE = (unsigned int)RANKFREE.size();
    NR = RANKFREE;
    NC = 150;
  }

  std::cout << "nVar \t= " << NC << std::endl;
  std::cout << "nLevels \t= " << NB_STAGE << std::endl;
  std::cout << "LevelDim \t= [ ";
  for (int i = 0; i < (int)NB_STAGE; ++i) std::cout << NR[i] << " ";
  std::cout << "]" << std::endl;

  generateDeficientDataSet(J, b, NB_STAGE, RANKFREE, RANKLINKED, NR, NC);

  HCOD hsolver(NC, NB_STAGE);
  VectorXd solution(NC);
  for (unsigned int i = 0; i < NB_STAGE; ++i) {
    for (int j = 0; j < b[i].size(); ++j) b[i][j] = Bound(rand());
    hsolver.pushBackStage(J[i], b[i]);
    hsolver.setNameByOrder("level");
  }
  hsolver.setDamping(0.0);
  hsolver.setInitialActiveSet();

  struct timeval t0, t1;
  double time;

  gettimeofday(&t0, NULL);
  for (int i = 0; i < 1000; ++i) ehqp(hsolver);
  gettimeofday(&t1, NULL);
  time = (double)(t1.tv_sec - t0.tv_sec) +
         (double)(t1.tv_usec - t0.tv_usec) / 1.0e6;
  cout << "ehqp = " << time << " ms " << std::endl;

  MatrixXd A = J[0];
  MatrixXd Y(NC, NC);

  gettimeofday(&t0, NULL);
  for (int i = 0; i < 1000; ++i)
    Eigen::DestructiveColPivQR<MatrixXd, MatrixXd> mQR(A, Y, 1e-6);
  // Eigen::DestructiveColPivQR<SubMatrixXd,MatrixXd > mQR(Ar,Y, 1e-6);
  gettimeofday(&t1, NULL);
  time = (double)(t1.tv_sec - t0.tv_sec) +
         (double)(t1.tv_usec - t0.tv_usec) / 1.0e6;
  cout << "qr = " << time << " ms " << std::endl;
}

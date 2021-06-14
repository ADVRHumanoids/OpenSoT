/* -------------------------------------------------------------------------- *
 *
 * Active search unittest. Could be asserted.
 *
 * -------------------------------------------------------------------------- */

#include "RandomGenerator.hpp"
#include "soth/HCOD.hpp"
#include "soth/debug.hpp"

int main(int, char**) {
  soth::sotDebugTrace::openFile();
  const unsigned int NB_STAGE = 3;
  const int RANKFREE[] = {5, 2, 5, 5, 3};
  // const int RANKFREE[]   = { 5, 4, 5,     5, 3 };
  const int RANKLINKED[] = {0, 2, 1, 5, 3};
  const int NR[] = {5, 4, 5, 5, 8};
  const unsigned int NC = 15;

  /* Initialize J and b. */
  std::vector<Eigen::MatrixXd> J(NB_STAGE);
  std::vector<soth::VectorBound> b(NB_STAGE);
  soth::generateDeficientDataSet(J, b, NB_STAGE, RANKFREE, RANKLINKED, NR, NC);

  b[0][2] = std::make_pair(-0.3, .3);
  b[0][3] = std::make_pair(-0.4, .4);
  b[0][4] = std::make_pair(-0.5, .5);

  for (unsigned int i = 0; i < NB_STAGE; ++i) {
    std::cout << "J" << i + 1 << " = " << (soth::MATLAB)J[i] << std::endl;
    std::cout << "e" << i + 1 << " = " << b[i] << ";" << std::endl;
  }
  //  assert( std::abs(J[0](0,0)-(-1.1149))<1e-5 );

  /* SOTH structure construction. */
  soth::HCOD hcod(NC, NB_STAGE);
  for (unsigned int i = 0; i < NB_STAGE; ++i) {
    hcod.pushBackStage(J[i], b[i]);
    assert(NR[i] > 0);
  }
  hcod.setInitialActiveSet();
  hcod.setNameByOrder("stage_");

  Eigen::VectorXd solution;
  hcod.activeSearch(solution);
  hcod.show(std::cout);
}

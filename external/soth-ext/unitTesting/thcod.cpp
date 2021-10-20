/* -------------------------------------------------------------------------- *
 *
 * Simple test of the HCOD class with assertion.
 *
 * -------------------------------------------------------------------------- */
#define SOTH_DEBUG
#define SOTH_DEBUG_MODE 10
#define SOTH_TEMPLATE_DEBUG_MODE 10

#include "RandomGenerator.hpp"
#include "soth/HCOD.hpp"
#include "soth/Random.hpp"
#include "soth/debug.hpp"

using namespace soth;

int main(int, char**) {
  soth::sotDebugTrace::openFile();
  soth::Random::setSeed(7);
  const int NB_STAGE = 3;
  const int RANKFREE[] = {3, 4, 3, 5, 3};
  const int RANKLINKED[] = {2, 2, 1, 5, 3};
  const int NR[] = {5, 4, 5, 5, 8};
  const int NC = 12;

  /* Initialize J and b. */
  std::vector<Eigen::MatrixXd> J(NB_STAGE);
  std::vector<soth::VectorBound> b(NB_STAGE);
  soth::generateDeficientDataSet(J, b, NB_STAGE, RANKFREE, RANKLINKED, NR, NC);
  b[0][1] = std::make_pair(-0.1, 2.37);
  for (int i = 0; i < NB_STAGE; ++i) {
    sotDEBUG(0) << "J" << i + 1 << " = " << (soth::MATLAB)J[i] << std::endl;
    sotDEBUG(0) << "e" << i + 1 << " = " << b[i] << ";" << std::endl;
  }
  std::cout << J[0](0, 0) << std::endl;
  assert(std::abs(J[0](0, 0) - 0.544092) < 1e-5);

  /* SOTH structure construction. */
  soth::HCOD hcod(NC, NB_STAGE);
  for (int i = 0; i < NB_STAGE; ++i) {
    hcod.pushBackStage(J[i], b[i]);
    assert(NR[i] > 0);
  }
  hcod.setInitialActiveSet();
  hcod.setNameByOrder("stage_");

  hcod.initialize();
  hcod.Y.computeExplicitly();
  hcod.computeSolution(true);
  std::cout << hcod.rank() << " " << hcod[0].rank() << " " << hcod[1].rank()
            << " " << hcod[2].rank() << std::endl;
  assert((hcod.rank() == 4) && (hcod[0].rank() == 0) && (hcod[1].rank() == 2) &&
         (hcod[2].rank() == 2));

  double tau = hcod.computeStepAndUpdate();
  hcod.makeStep(tau);
  std::cout << "tau:" << tau << " " << std::abs(tau - 0.486522) << " "
            << soth::Stage::EPSILON << std::endl;
  assert((std::abs(tau - 0.486522) <= 50 * soth::Stage::EPSILON) &&
         "Check bound test failed.");

  hcod.computeLagrangeMultipliers(hcod.nbStages());
  bool testL = hcod.testLagrangeMultipliers(hcod.nbStages(), std::cout);
  sotDEBUG(5) << "Test multipliers: " << ((testL) ? "Passed!" : "Failed...")
              << std::endl;
  assert(testL && "Lagrange Multipliers test failed.");
}

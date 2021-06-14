/* -------------------------------------------------------------------------- *
 *
 * Simple test of the Stage class.
 * No assertion, assert could be added on the final test.
 *
 * -------------------------------------------------------------------------- */

#include <Eigen/Jacobi>
#include <Eigen/QR>
#include <boost/smart_ptr.hpp>
#include <iostream>
#include "RandomGenerator.hpp"
#include "soth/BaseY.hpp"
#include "soth/Stage.hpp"

int main(int, char**) {
  // const int NB_STAGE = 5;
  // const int RANK[] = { 3, 5, 3, 5, 3 };
  // const int NR[] = { 5, 5, 5, 5, 8 };
  // const int NC = 20;

  const unsigned int NB_STAGE = 3;
  const int RANK[] = {3, 4, 3, 5, 3};
  const int RANKLINKED[] = {0, 0, 0, 0, 0};
  const int NR[] = {5, 4, 5, 5, 8};
  const unsigned int NC = 12;
  /* Initialize J and b. */
  std::vector<Eigen::MatrixXd> J(NB_STAGE);
  std::vector<soth::VectorBound> b(NB_STAGE);

  soth::generateDeficientDataSet(J, b, NB_STAGE, RANK, RANKLINKED, NR, NC);

  for (unsigned int i = 0; i < NB_STAGE; ++i) {
    std::cout << "J" << i << " = " << (soth::MATLAB)J[i] << std::endl;
  }

  /* SOTH structure construction. */
  soth::BaseY Y(NC);
  typedef boost::shared_ptr<soth::Stage> stage_ptr_t;
  typedef std::vector<stage_ptr_t> stage_list_t;
  stage_list_t stages(NB_STAGE);
  for (unsigned int i = 0; i < NB_STAGE; ++i) {
    std::cout << " --- STAGE " << i << " --------------------------------- "
              << std::endl;
    /* Compute the initial COD for each stage. */
    stages[i] = stage_ptr_t(new soth::Stage(J[i], b[i], Y));
#ifndef NDEBUG
    stages[i]->reset();
#endif
    stages[i]->setInitialActiveSet();
    stages[i]->computeInitialCOD(Y);
    Eigen::MatrixXd Jrec;
    stages[i]->recompose(Jrec);
    std::cout << "Jrec" << i << " = " << (soth::MATLAB)Jrec << std::endl;
  }

  Eigen::MatrixXd rec;
  stages[0]->recompose(rec);
}

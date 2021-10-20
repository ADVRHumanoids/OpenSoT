/* -------------------------------------------------------------------------- *
 *
 * Exhaustive tests of the HCOD, initial decompo, update and downdate, with
 * assertion.
 *
 * -------------------------------------------------------------------------- */

#include "soth/HCOD.hpp"
#include "soth/Random.hpp"
#include "soth/debug.hpp"

#ifndef WIN32
#include <sys/time.h>
#endif  // WIN32

#include <iomanip>
#include "gettimeofday.hpp"

using namespace soth;

void generateDataSet(std::vector<Eigen::MatrixXd> &J,
                     std::vector<soth::VectorBound> &b, const int NB_STAGE,
                     const int RANK[], const int NR[], const int NC) {
  /* Initialize J and b. */
  J.resize(NB_STAGE);
  b.resize(NB_STAGE);

  for (int s = 0; s < NB_STAGE; ++s) {
    Eigen::MatrixXd Xhi(NR[s], RANK[s]);
    Eigen::MatrixXd Jfr(RANK[s], NC);
    b[s].resize(NR[s]);

    soth::MatrixRnd::randomize(Xhi);
    soth::MatrixRnd::randomize(Jfr);
    sotDEBUG(5) << "Xhi: " << Xhi << std::endl;
    sotDEBUG(5) << "Jfr: " << Jfr << std::endl;
    J[s] = Xhi * Jfr;

    for (int i = 0; i < NR[s]; ++i) b[s][i] = (double)(i + 1);
  }
}

void generateDeficientDataSet(std::vector<Eigen::MatrixXd> &J,
                              std::vector<soth::VectorBound> &b,
                              const int NB_STAGE, const int RANKFREE[],
                              const int RANKLINKED[], const int NR[],
                              const int NC) {
  /* Initialize J and b. */
  J.resize(NB_STAGE);
  b.resize(NB_STAGE);

  for (int s = 0; s < NB_STAGE; ++s) {
    b[s].resize(NR[s]);

    assert((RANKFREE[s] > 0) || (RANKLINKED[s] > 0));

    J[s].resize(NR[s], NC);
    J[s].setZero();
    if (RANKFREE[s] > 0) {
      Eigen::MatrixXd Xhifree(NR[s], RANKFREE[s]);
      Eigen::MatrixXd Jfr(RANKFREE[s], NC);
      soth::MatrixRnd::randomize(Xhifree);
      soth::MatrixRnd::randomize(Jfr);
      if (Xhifree.cols() > 0) J[s] += Xhifree * Jfr;
    }
    if (RANKLINKED[s] > 0) {
      Eigen::MatrixXd Xhilinked(NR[s], RANKLINKED[s]);
      soth::MatrixRnd::randomize(Xhilinked);
      for (int sb = 0; sb < s; ++sb) {
        Eigen::MatrixXd Alinked(RANKLINKED[s], NR[sb]);
        soth::MatrixRnd::randomize(Alinked);
        J[s] += Xhilinked * Alinked * J[sb];
      }
    }

    for (int i = 0; i < NR[s]; ++i) b[s][i] = (double)(i + 1);
  }
}

bool clearIteralively(soth::HCOD &hcod) {
  bool exitOk = true;
  for (unsigned int s = 0; s < hcod.nbStages(); ++s) {
    while (hcod[s].sizeA() > 0) {
      hcod.downdate(s, 0);
      exitOk &= hcod.testRecomposition(&std::cout);
      assert(exitOk);
    }
  }
  return exitOk;
}

int main(int, char **) {
  bool exitOk = true;
  const int executeAll = 1;
  sotDebugTrace::openFile();

  Eigen::MatrixXd Massert(5, 9);
  soth::MatrixRnd::randomize(Massert);
  std::cout << "Massert(1,2): " << Massert(1, 2) << std::endl;

  assert(std::abs(Massert(1, 2) + 0.368673) < 1e-5);

  {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    int seed = (int)(tv.tv_usec % 7919);
    std::cout << "seed = " << seed << std::endl;
    soth::Random::setSeed(seed);
  }

  /* --- INITIALIZATION TESTS ----------------------------------------------- */
  /* --- INITIALIZATION TESTS ----------------------------------------------- */
  /* --- INITIALIZATION TESTS ----------------------------------------------- */

  if (executeAll) {
    // sotDebugTrace::openFile();
    /* All matrices full rank and independant, but the last one due to
     * lack of DOF.
     */
    const int NB_STAGE = 3;
    const int RANK[] = {3, 3, 6};
    const int NR[] = {3, 3, 6};
    const int NC = 10;

    std::vector<Eigen::MatrixXd> J(NB_STAGE);
    std::vector<soth::VectorBound> b(NB_STAGE);
    generateDataSet(J, b, NB_STAGE, RANK, NR, NC);

    soth::HCOD hcod(NC, NB_STAGE);
    hcod.pushBackStages(J, b);

    hcod.initialize();
    exitOk &= hcod.testRecomposition(&std::cout);
    // if( sotDEBUGFLOW.outputbuffer.good() ) hcod.show(
    // sotDEBUGFLOW.outputbuffer );
    assert(exitOk);
  }

  if (executeAll) {
    // sotDebugTrace::openFile();
    /* All matrices full rank, updating rows until the last rank saturates
     * du to the matrix size. Then remove all the lines from the first.
     */
    const int NB_STAGE = 3;
    const int RANK[] = {3, 3, 6};
    const int NR[] = {3, 3, 6};
    const int NC = 10;

    std::vector<Eigen::MatrixXd> J(NB_STAGE);
    std::vector<soth::VectorBound> b(NB_STAGE);
    generateDataSet(J, b, NB_STAGE, RANK, NR, NC);
    for (int i = 2; i < NR[2]; ++i) b[2][i] = std::make_pair(-i - 1, i + 1);

    soth::HCOD hcod(NC, NB_STAGE);
    hcod.pushBackStages(J, b);

    hcod.initialize();

    for (int i = 2; i < NR[2]; ++i) {
      hcod.update(2, ConstraintRef(i, soth::Bound::BOUND_INF));
    }

    exitOk &= hcod.testRecomposition(&std::cout);
    //    if( sotDEBUGFLOW.outputbuffer.good() ) hcod.show(
    //    sotDEBUGFLOW.outputbuffer );

    exitOk &= clearIteralively(hcod);
    assert(exitOk);
  }

  if (executeAll) {
    // sotDebugTrace::openFile();
    /* Stage 1 has 1 rows, Stage 2 is rank 1, Stage 3 projected as rank 1,
     * Stage 4 has 0 rows, Stage 5 projected is rank 0.
     */
    const int NB_STAGE = 6;
    const int RANKFREE[] = {3, 1, 1, 1, 0, 0};
    const int RANKLINKED[] = {0, 0, 0, 2, 1, 3};
    const int NR[] = {3, 1, 3, 3, 1, 3};
    const int NC = 10;

    std::vector<Eigen::MatrixXd> J(NB_STAGE);
    std::vector<soth::VectorBound> b(NB_STAGE);
    generateDeficientDataSet(J, b, NB_STAGE, RANKFREE, RANKLINKED, NR, NC);
    b[4][0] = std::make_pair(-1, +1);

    soth::HCOD hcod(NC, NB_STAGE);
    hcod.pushBackStages(J, b);

    hcod.initialize();

    exitOk &= hcod.testRecomposition(&std::cout);
    if (sotDEBUGFLOW.outputbuffer.good()) hcod.show(sotDEBUGFLOW.outputbuffer);

    assert(hcod.rank() == 6);
    for (int i = 0; i < 4; ++i) {
      assert(hcod[i].rank() == RANKFREE[i]);
    }
    assert(hcod[4].rank() == 0);
    assert(hcod[5].rank() == 0);

    exitOk &= clearIteralively(hcod);
    assert(exitOk);
  }

  /* --- UPDATE TESTS ------------------------------------------------------- */
  /* --- UPDATE TESTS ------------------------------------------------------- */
  /* --- UPDATE TESTS ------------------------------------------------------- */

  /* :CHECK LIST:
   *  ==========
   * UPDATE:
   *  1. PRE on current stage:
   *   1a. Stage is empty.
   *   1b. Stage is not empty.
   *    1ba. Stage is rank-null.
   *    1bb. Stage is not rank-null.
   *     1bba. Stage is full rank.
   *      1bbaa. Stage is overshoot (ie sizeM+sizeL==nc)
   *      1bbab. Stage is not overshoot (ie sizeM+sizeL<nc)
   *     1bbb. Stage is rank deficient.
   *      1bbba. Stage is overshoot (ie sizeM+sizeL==nc)
   *      1bbbb. Stage is not overshoot (ie sizeM+sizeL<nc)
   *
   *  2. POST on current stage:
   *   2a. Rank didn't increase.
   *   2b. Rank increase.
   *    2ba. Total rank increase.
   *    2bb. Total rank did not increase.
   *     2bba. Rank increase caused an overshoot (ie, at last non-void stage,
   * sM+sL==nc). 2bbb. No overshoot. 2bbba. Rank increase caused a rank closure
   * (ie rank==0 on a upper stage). 2bbbb. No rank closure.
   */

  if (executeAll) {
    // sotDebugTrace::openFile();
    /* Insertion of a full rank line, increasing the total rank, on stages
     * first, middle, last.
     */
    const int NB_STAGE = 3;
    const int RANK[] = {3, 3, 3};
    const int NR[] = {3, 3, 3};
    const int NC = 10;

    std::vector<Eigen::MatrixXd> J(NB_STAGE);
    std::vector<soth::VectorBound> b(NB_STAGE);
    generateDataSet(J, b, NB_STAGE, RANK, NR, NC);
    for (int s = 0; s < NB_STAGE; ++s) b[s][2] = std::make_pair(-3, 3);

    soth::HCOD hcod(NC, NB_STAGE);
    hcod.pushBackStages(J, b);

    hcod.initialize();

    exitOk &= hcod.testRecomposition(&std::cout);
    // if( sotDEBUGFLOW.outputbuffer.good() ) hcod.show(
    // sotDEBUGFLOW.outputbuffer );

#ifndef NDEBUG
    int rank = hcod.rank();
#endif

    for (int i = 0; i < NB_STAGE; ++i) {
      hcod.update(i, ConstraintRef(2, soth::Bound::BOUND_INF));
      exitOk &= hcod.testRecomposition(&std::cout);
      assert(hcod.rank() == ++rank);
    }

    exitOk &= clearIteralively(hcod);
    assert(exitOk);
  }

  if (executeAll) {
    // sotDebugTrace::openFile();
    /* Insertion of a rank-def line, preserving the total rank, on stages first,
     * middle, last.
     */
    const int NB_STAGE = 3;
    const int RANK[] = {2, 2, 2};
    const int NR[] = {3, 3, 3};
    const int NC = 10;

    std::vector<Eigen::MatrixXd> J(NB_STAGE);
    std::vector<soth::VectorBound> b(NB_STAGE);
    generateDataSet(J, b, NB_STAGE, RANK, NR, NC);
    for (int s = 0; s < NB_STAGE; ++s) b[s][2] = std::make_pair(-3, 3);

    soth::HCOD hcod(NC, NB_STAGE);
    hcod.pushBackStages(J, b);

    hcod.initialize();

    exitOk &= hcod.testRecomposition(&std::cout);
    // if( sotDEBUGFLOW.outputbuffer.good() ) hcod.show(
    // sotDEBUGFLOW.outputbuffer );

#ifndef NDEBUG
    int rank = hcod.rank();
#endif

    for (int i = 0; i < NB_STAGE; ++i) {
#ifndef NDEBUG
      const int rankStage = (int)hcod[i].rank();
#endif
      hcod.update(i, ConstraintRef(2, soth::Bound::BOUND_INF));
      exitOk &= hcod.testRecomposition(&std::cout);
      assert(hcod.rank() == rank);
      assert(rankStage == hcod[i].rank());
    }

    exitOk &= clearIteralively(hcod);
    assert(exitOk);
  }

  if (executeAll) {
    // sotDebugTrace::openFile();
    /* Insertion of a full rank line, increasing stage rank but not the total
     * rank, on stages first, and middle (not last, this case is not possible).
     */
    const int NB_STAGE = 3;
    const int RANK[] = {3, 3, 3};
    const int NR[] = {3, 3, 3};
    const int NC = 10;

    std::vector<Eigen::MatrixXd> J(NB_STAGE);
    std::vector<soth::VectorBound> b(NB_STAGE);
    generateDataSet(J, b, NB_STAGE, RANK, NR, NC);
    for (int s = 0; s < NB_STAGE - 1; ++s) {
      const int LAST = NR[s] - 1;
      b[s][LAST] = std::make_pair(-LAST - 1, LAST - 1);
    }
    {  // Link last line of 0 to first 2 lines of 1.
      const int s = 0, LAST = NR[s] - 1;
      Eigen::MatrixXd Xhi(1, NR[s + 1] - 1);
      soth::MatrixRnd::randomize(Xhi);
      J[s].row(LAST) = Xhi * J[s + 1].topRows(NR[s + 1] - 1);
    }
    {  // Link last line of 1 to 2.
      const int s = 1, LAST = NR[s] - 1;
      Eigen::MatrixXd Xhi(1, NR[s + 1]);
      soth::MatrixRnd::randomize(Xhi);
      J[s].row(LAST) = Xhi * J[s + 1];
    }

    soth::HCOD hcod(NC, NB_STAGE);
    hcod.pushBackStages(J, b);

    hcod.initialize();

    exitOk &= hcod.testRecomposition(&std::cout);
    // if( sotDEBUGFLOW.outputbuffer.good() ) hcod.show(
    // sotDEBUGFLOW.outputbuffer );

#ifndef NDEBUG
    int rank = hcod.rank();
#endif
    for (int i = 0; i < NB_STAGE - 1; ++i) {
#ifndef NDEBUG
      const int rankStage = (int)hcod[i].rank();
#endif
      hcod.update(i, ConstraintRef(2, soth::Bound::BOUND_INF));
      exitOk &= hcod.testRecomposition(&std::cout);
      assert(hcod.rank() == rank);
      assert(rankStage + 1 == hcod[i].rank());
    }

    exitOk &= clearIteralively(hcod);
    assert(exitOk);
  }

  /* --- Overshoot --- */
  if (executeAll) {
    // sotDebugTrace::openFile();
    /* Insertion at the last stage when rank==nc (direct overshoot possible).
     */
    const int NB_STAGE = 5;
    const int RANK[] = {3, 3, 3, 3, 5};
    const int NR[] = {4, 4, 4, 4, 5};
    const int NC = 15;

    std::vector<Eigen::MatrixXd> J(NB_STAGE);
    std::vector<soth::VectorBound> b(NB_STAGE);
    generateDataSet(J, b, NB_STAGE, RANK, NR, NC);

    const int LS = NB_STAGE - 1, LR = NR[LS] - 1;
    b[LS][LR] = std::make_pair(-LR - 1, LR + 1);

    soth::HCOD hcod(NC, NB_STAGE);
    hcod.pushBackStages(J, b);

    hcod.initialize();

    exitOk &= hcod.testRecomposition(&std::cout);
    // if( sotDEBUGFLOW.outputbuffer.good() ) hcod.show(
    // sotDEBUGFLOW.outputbuffer );

#ifndef NDEBUG
    const int rank = hcod.rank();
    const int rankStage = (int)hcod[LS].rank();
#endif
    assert(rank == NC);
    hcod.update(LS, ConstraintRef(LR, soth::Bound::BOUND_INF));
    exitOk &= hcod.testRecomposition(&std::cout);
    assert(hcod.rank() == rank);
    assert(rankStage == hcod[LS].rank());

    exitOk &= clearIteralively(hcod);
    assert(exitOk);
  }

  if (executeAll) {
    // sotDebugTrace::openFile();
    /* Insertion at the last stage when rank==nc (indirect overshoot possible).
     */
    const int NB_STAGE = 5;
    const int RANK[] = {3, 3, 3, 3, 5};
    const int NR[] = {4, 4, 4, 4, 5};
    const int NC = 15;

    std::vector<Eigen::MatrixXd> J(NB_STAGE);
    std::vector<soth::VectorBound> b(NB_STAGE);
    generateDataSet(J, b, NB_STAGE, RANK, NR, NC);

    const int LS = NB_STAGE - 3, LR = NR[LS] - 1;
    b[LS][LR] = std::make_pair(-LR - 1, LR + 1);

    soth::HCOD hcod(NC, NB_STAGE);
    hcod.pushBackStages(J, b);

    hcod.initialize();

    exitOk &= hcod.testRecomposition(&std::cout);
// if( sotDEBUGFLOW.outputbuffer.good() ) hcod.show( sotDEBUGFLOW.outputbuffer
// );
#ifndef NDEBUG
    const int rank = (int)hcod.rank();
    const int rankStage = (int)hcod[LS].rank();
#endif
    assert(rank == NC);
    hcod.update(LS, ConstraintRef(LR, soth::Bound::BOUND_INF));
    exitOk &= hcod.testRecomposition(&std::cout);
    assert(hcod.rank() == rank);
    assert(rankStage == hcod[LS].rank());

    exitOk &= clearIteralively(hcod);
    assert(exitOk);
  }

  /* --- Closure --- */
  if (executeAll) {
    // sotDebugTrace::openFile();
    /* Insertion of a full rank line that causes the closure of a stage above.
     *  1.   Rank distribution is 2-2-2, with the 2 last stages being link to
     * the 4 first inactive constraints of stage 0. 2.a  Update Stage 0,
     * decrease stage 2 rank. 2.b  Update Stage 0, closing stage 2 to rank 0.
     *  2.c  Update Stage 0, decrease stage 2 rank.
     *  2.d  Update Stage 0, closing stage 1 to rank 0.
     *  2.e  Update Stage 0, total rank increase, no rank modif on the closed
     * stages.
     */
    const int NB_STAGE = 3;
    const int RANKFREE[] = {6, 0, 0};
    const int RANKLINKED[] = {0, 2, 3};
    const int NR[] = {7, 2, 3};
    const int NC = 12;

    std::vector<Eigen::MatrixXd> J(NB_STAGE);
    std::vector<soth::VectorBound> b(NB_STAGE);
    generateDeficientDataSet(J, b, NB_STAGE, RANKFREE, RANKLINKED, NR, NC);
    for (int i = 2; i < NR[0]; ++i) b[0][i] = std::make_pair(-i - 1, i + 1);
    {  // Stage 1 and 2 depends on the N-1 first lines of J[0], last line is
       // free.
      MatrixXd Jl(1, NC);
      soth::MatrixRnd::randomize(Jl);
      J[0].row(NR[0] - 1) = Jl;
    }

    soth::HCOD hcod(NC, NB_STAGE);
    hcod.pushBackStages(J, b);

    /*1.*/ hcod.initialize();
    exitOk &= hcod.testRecomposition(&std::cout);
    assert(hcod.rank() == 6);
    assert(hcod[0].rank() == 2);
    assert(hcod[1].rank() == 2);
    assert(hcod[2].rank() == 2);
    // if( sotDEBUGFLOW.outputbuffer.good() ) hcod.show(
    // sotDEBUGFLOW.outputbuffer );

    /*2.a*/ hcod.update(0, ConstraintRef(2, soth::Bound::BOUND_INF));
    exitOk &= hcod.testRecomposition(&std::cout);
    assert(hcod.rank() == 6);
    assert(hcod[0].rank() == 3);
    assert(hcod[1].rank() == 2);
    assert(hcod[2].rank() == 1);

    /*2.b*/ hcod.update(0, ConstraintRef(3, soth::Bound::BOUND_INF));
    exitOk &= hcod.testRecomposition(&std::cout);
    assert(hcod.rank() == 6);
    assert(hcod[0].rank() == 4);
    assert(hcod[1].rank() == 2);
    assert(hcod[2].rank() == 0);

    /*2.c*/ hcod.update(0, ConstraintRef(4, soth::Bound::BOUND_INF));
    exitOk &= hcod.testRecomposition(&std::cout);
    assert(hcod.rank() == 6);
    assert(hcod[0].rank() == 5);
    assert(hcod[1].rank() == 1);
    assert(hcod[2].rank() == 0);

    /*2.d*/ hcod.update(0, ConstraintRef(5, soth::Bound::BOUND_INF));
    exitOk &= hcod.testRecomposition(&std::cout);
    assert(hcod.rank() == 6);
    assert(hcod[0].rank() == 6);
    assert(hcod[1].rank() == 0);
    assert(hcod[2].rank() == 0);

    /*2.e*/ hcod.update(0, ConstraintRef(6, soth::Bound::BOUND_INF));
    exitOk &= hcod.testRecomposition(&std::cout);
    assert(hcod.rank() == 7);
    assert(hcod[0].rank() == 7);
    assert(hcod[1].rank() == 0);
    assert(hcod[2].rank() == 0);

    // if( sotDEBUGFLOW.outputbuffer.good() ) hcod.show(
    // sotDEBUGFLOW.outputbuffer );

    exitOk &= clearIteralively(hcod);
    assert(exitOk);
  }

  /* --- Ouverture --- */
  if (executeAll) {
    // sotDebugTrace::openFile();
    /* Insertion in a stage with 0 rank, then removal (back to 0, closure),
     * then reintro (opening).
     */
    const int NB_STAGE = 2;
    const int RANK[] = {3, 1};
    const int NR[] = {4, 1};
    const int NC = 10;

    std::vector<Eigen::MatrixXd> J(NB_STAGE);
    std::vector<soth::VectorBound> b(NB_STAGE);
    generateDataSet(J, b, NB_STAGE, RANK, NR, NC);
    b[1][0] = std::make_pair(-1, 1);

    soth::HCOD hcod(NC, NB_STAGE);
    hcod.pushBackStages(J, b);

    hcod.initialize();

    exitOk &= hcod.testRecomposition(&std::cout);
    assert(hcod.rank() == 3);
    assert(hcod[0].rank() == 3);
    assert(hcod[1].rank() == 0);
    // if( sotDEBUGFLOW.outputbuffer.good() ) hcod.show(
    // sotDEBUGFLOW.outputbuffer );

    hcod.update(1, ConstraintRef(0, soth::Bound::BOUND_INF));
    exitOk &= hcod.testRecomposition(&std::cout);
    assert(hcod.rank() == 4);
    assert(hcod[0].rank() == 3);
    assert(hcod[1].rank() == 1);

    hcod.downdate(1, 0);
    exitOk &= hcod.testRecomposition(&std::cout);
    assert(hcod.rank() == 3);
    assert(hcod[0].rank() == 3);
    assert(hcod[1].rank() == 0);

    hcod.update(1, ConstraintRef(0, soth::Bound::BOUND_INF));
    exitOk &= hcod.testRecomposition(&std::cout);
    assert(hcod.rank() == 4);
    assert(hcod[0].rank() == 3);
    assert(hcod[1].rank() == 1);

    exitOk &= clearIteralively(hcod);
    assert(exitOk);
  }

  if (executeAll) {
    /* Opening the first stage and next closing the last one. */
    const int NB_STAGE = 3;
    const int RANKFREE[] = {5, 5, 5};
    const int RANKLINKED[] = {0, 0, 0};
    const int NR[] = {5, 5, 5};
    const int NC = 10;

    std::vector<Eigen::MatrixXd> J(NB_STAGE);
    std::vector<soth::VectorBound> b(NB_STAGE);
    generateDeficientDataSet(J, b, NB_STAGE, RANKFREE, RANKLINKED, NR, NC);
    for (int i = 0; i < NR[0]; ++i) b[0][i] = std::make_pair(-i - 1, i + 1);

    soth::HCOD hcod(NC, NB_STAGE);
    hcod.pushBackStages(J, b);

    hcod.initialize();
    exitOk &= hcod.testRecomposition(&std::cout);

    for (int i = 0; i < NR[0]; ++i) {
      assert(hcod.rank() == 10);
      assert(hcod[0].rank() == i);
      assert(hcod[1].rank() == 5);
      assert(hcod[2].rank() == 5 - i);
      hcod.update(0, ConstraintRef(i, soth::Bound::BOUND_INF));
      exitOk &= hcod.testRecomposition(&std::cout);
      assert(exitOk);
    }
    assert(hcod.rank() == 10);
    assert(hcod[0].rank() == 5);
    assert(hcod[1].rank() == 5);
    assert(hcod[2].rank() == 0);

    exitOk &= clearIteralively(hcod);
    assert(exitOk);
  }

  /* --- DOWNDATE TESTS ----------------------------------------------------- */
  /* --- DOWNDATE TESTS ----------------------------------------------------- */
  /* --- DOWNDATE TESTS ----------------------------------------------------- */
  if (executeAll) {
    // sotDebugTrace::openFile();
    /* Removal of a full rank line, with decrease of the total rank.
     */
    const int NB_STAGE = 3;
    const int RANK[] = {3, 3, 3};
    const int NR[] = {3, 3, 3};
    const int NC = 10;

    std::vector<Eigen::MatrixXd> J(NB_STAGE);
    std::vector<soth::VectorBound> b(NB_STAGE);
    generateDataSet(J, b, NB_STAGE, RANK, NR, NC);

    soth::HCOD hcod(NC, NB_STAGE);
    hcod.pushBackStages(J, b);

    hcod.initialize();

    exitOk &= hcod.testRecomposition(&std::cout);
// if( sotDEBUGFLOW.outputbuffer.good() ) hcod.show( sotDEBUGFLOW.outputbuffer
// );
#ifndef NDEBUG
    int rank = hcod.rank();
#endif
    for (int i = 0; i < NB_STAGE; ++i) {
#ifndef NDEBUG
      const int rankStage = (int)hcod[i].rank();
#endif
      hcod.downdate(i, 1);
      exitOk &= hcod.testRecomposition(&std::cout);
      assert(hcod.rank() == --rank);
      assert(rankStage - 1 == hcod[i].rank());
    }

    exitOk &= clearIteralively(hcod);
    assert(exitOk);
  }

  if (executeAll) {
    // sotDebugTrace::openFile();
    /* Removal of a rank def line, with no decrease of neither the stage
     * nor the total rank.
     */
    const int NB_STAGE = 3;
    const int RANK[] = {2, 2, 2};
    const int NR[] = {3, 3, 3};
    const int NC = 10;

    std::vector<Eigen::MatrixXd> J(NB_STAGE);
    std::vector<soth::VectorBound> b(NB_STAGE);
    generateDataSet(J, b, NB_STAGE, RANK, NR, NC);

    soth::HCOD hcod(NC, NB_STAGE);
    hcod.pushBackStages(J, b);

    hcod.initialize();

    exitOk &= hcod.testRecomposition(&std::cout);
// if( sotDEBUGFLOW.outputbuffer.good() ) hcod.show( sotDEBUGFLOW.outputbuffer
// );
#ifndef NDEBUG
    int rank = hcod.rank();
#endif
    for (int i = 0; i < NB_STAGE; ++i) {
#ifndef NDEBUG
      const int rankStage = (int)hcod[i].rank();
#endif
      hcod.downdate(i, 1);
      exitOk &= hcod.testRecomposition(&std::cout);
      assert(hcod.rank() == rank);
      assert(rankStage == hcod[i].rank());
    }

    exitOk &= clearIteralively(hcod);
    assert(exitOk);
  }

  if (executeAll) {
    // sotDebugTrace::openFile();
    /* Insertion of a full rank line, decreasing stage rank but not the total
     * rank, on stages first, and middle (not last, this case is not possible).
     */
    const int NB_STAGE = 3;
    const int RANK[] = {3, 3, 3};
    const int NR[] = {3, 3, 3};
    const int NC = 10;

    std::vector<Eigen::MatrixXd> J(NB_STAGE);
    std::vector<soth::VectorBound> b(NB_STAGE);
    generateDataSet(J, b, NB_STAGE, RANK, NR, NC);
    {  // Link last line of 0 to first 2 lines of 1.
      const int s = 0, LAST = NR[s] - 1;
      Eigen::MatrixXd Xhi(1, NR[s + 1] - 1);
      soth::MatrixRnd::randomize(Xhi);
      J[s].row(LAST) = Xhi * J[s + 1].topRows(NR[s + 1] - 1);
    }
    {  // Link last line of 1 to 2.
      const int s = 1, LAST = NR[s] - 1;
      Eigen::MatrixXd Xhi(1, NR[s + 1]);
      soth::MatrixRnd::randomize(Xhi);
      J[s].row(LAST) = Xhi * J[s + 1];
    }

    soth::HCOD hcod(NC, NB_STAGE);
    hcod.pushBackStages(J, b);

    hcod.initialize();
    // if( sotDEBUGFLOW.outputbuffer.good() ) hcod.show(
    // sotDEBUGFLOW.outputbuffer );

    exitOk &= hcod.testRecomposition(&std::cout);
    assert(hcod.rank() == 7);
    assert(hcod[0].rank() == 3);
    assert(hcod[1].rank() == 2);
    assert(hcod[2].rank() == 2);
    assert(hcod[0].gete()(hcod[0].where(2), 0) == 3);
    assert(exitOk);

    hcod.downdate(0, (unsigned int)hcod[0].where(NR[0] - 1));
    exitOk &= hcod.testRecomposition(&std::cout);
    assert(hcod.rank() == 7);
    assert(hcod[0].rank() == 2);
    assert(hcod[1].rank() == 3);
    assert(hcod[2].rank() == 2);
    assert(exitOk);

    hcod.downdate(1, (unsigned int)hcod[1].where(NR[1] - 1));
    exitOk &= hcod.testRecomposition(&std::cout);
    assert(hcod.rank() == 7);
    assert(hcod[0].rank() == 2);
    assert(hcod[1].rank() == 2);
    assert(hcod[2].rank() == 3);
    assert(exitOk);

    exitOk &= clearIteralively(hcod);
    assert(exitOk);
  }

  // Opening at rank liberation
  if (executeAll) {
    // sotDebugTrace::openFile();
    /* Removal of a row linked to an upper 0-rank stage, causing rank opening.
     */
    const int NB_STAGE = 3;
    const int RANKFREE[] = {3, 0, 0};
    const int RANKLINKED[] = {3, 1, 3};
    const int NR[] = {3, 3, 3};
    const int NC = 10;

    std::vector<Eigen::MatrixXd> J(NB_STAGE);
    std::vector<soth::VectorBound> b(NB_STAGE);
    generateDeficientDataSet(J, b, NB_STAGE, RANKFREE, RANKLINKED, NR, NC);

    soth::HCOD hcod(NC, NB_STAGE);
    hcod.pushBackStages(J, b);

    hcod.initialize();

    exitOk &= hcod.testRecomposition(&std::cout);
    assert(hcod.rank() == 3);
    assert(hcod[0].rank() == 3);
    assert(hcod[1].rank() == 0);
    assert(hcod[2].rank() == 0);

    hcod.downdate(0, 0);
    exitOk &= hcod.testRecomposition(&std::cout);
    assert(hcod.rank() == 3);
    assert(hcod[0].rank() == 2);
    assert(hcod[1].rank() == 1);
    assert(hcod[2].rank() == 0);

    hcod.downdate(0, 0);
    exitOk &= hcod.testRecomposition(&std::cout);
    assert(hcod.rank() == 3);
    assert(hcod[0].rank() == 1);
    assert(hcod[1].rank() == 1);
    assert(hcod[2].rank() == 1);

    exitOk &= clearIteralively(hcod);
    assert(exitOk);
  }

  if (executeAll) {
    // sotDebugTrace::openFile();
    /* Removal of a full-rank row, causing rank-0 at the stage. Second, the
     * same, but causing sizeA==0 at the stage.
     */
    const int NB_STAGE = 3;
    const int RANKFREE[] = {3, 1, 0};
    const int RANKLINKED[] = {3, 1, 3};
    const int NR[] = {3, 1, 3};
    const int NC = 10;

    std::vector<Eigen::MatrixXd> J(NB_STAGE);
    std::vector<soth::VectorBound> b(NB_STAGE);
    generateDeficientDataSet(J, b, NB_STAGE, RANKFREE, RANKLINKED, NR, NC);
    {  // Stage 2 last line is free.
      MatrixXd Xhi(3, 3);
      soth::MatrixRnd::randomize(Xhi);
      J[2] = Xhi * J[0];
      MatrixXd Jl(1, NC);
      soth::MatrixRnd::randomize(Jl);
      J[2].bottomRows(1) = Jl;
    }

    soth::HCOD hcod(NC, NB_STAGE);
    hcod.pushBackStages(J, b);

    hcod.initialize();
    exitOk &= hcod.testRecomposition(&std::cout);
    assert(hcod.rank() == 5);
    assert(hcod[0].rank() == 3);
    assert(hcod[1].rank() == 1);
    assert(hcod[2].rank() == 1);

    hcod.downdate(2, (unsigned int)hcod[2].where(2));
    exitOk &= hcod.testRecomposition(&std::cout);
    assert(hcod.rank() == 4);
    assert(hcod[0].rank() == 3);
    assert(hcod[1].rank() == 1);
    assert(hcod[2].rank() == 0);
    if (sotDEBUGFLOW.outputbuffer.good()) hcod.show(sotDEBUGFLOW.outputbuffer);

    hcod.downdate(1, 0);
    exitOk &= hcod.testRecomposition(&std::cout);
    assert(hcod.rank() == 3);
    assert(hcod[0].rank() == 3);
    assert(hcod[1].rank() == 0);
    assert(hcod[2].rank() == 0);
    assert(hcod[1].sizeA() == 0);

    exitOk &= clearIteralively(hcod);
    assert(exitOk);
  }

  if (executeAll) {
    // sotDebugTrace::openFile();
    /* Removal of a full-rank row, causing rank-0 at the stage. Second, the
     * same, but causing sizeA==0 at the stage.
     */
    const int NB_STAGE = 3;
    const int RANKFREE[] = {3, 0, 3};
    const int RANKLINKED[] = {0, 4, 0};
    const int NR[] = {3, 4, 3};
    const int NC = 10;

    std::vector<Eigen::MatrixXd> J(NB_STAGE);
    std::vector<soth::VectorBound> b(NB_STAGE);
    generateDeficientDataSet(J, b, NB_STAGE, RANKFREE, RANKLINKED, NR, NC);
    {  // Stage 1 last line is free.
      MatrixXd Jl(1, NC);
      soth::MatrixRnd::randomize(Jl);
      J[1].bottomRows(1) = Jl;
    }

    soth::HCOD hcod(NC, NB_STAGE);
    hcod.pushBackStages(J, b);

    hcod.initialize();
    exitOk &= hcod.testRecomposition(&std::cout);
    assert(hcod.rank() == 7);
    assert(hcod[0].rank() == 3);
    assert(hcod[1].rank() == 1);
    assert(hcod[2].rank() == 3);

    hcod.downdate(1, (unsigned int)hcod[1].where(NR[1] - 1));
    assert(hcod.rank() == 6);
    assert(hcod[0].rank() == 3);
    assert(hcod[1].rank() == 0);
    assert(hcod[2].rank() == 3);

    while (hcod[1].sizeA() > 0) {
      hcod.downdate(1, 0);
      assert(hcod.rank() == 6);
      assert(hcod[0].rank() == 3);
      assert(hcod[1].rank() == 0);
      assert(hcod[2].rank() == 3);
    }

    exitOk &= clearIteralively(hcod);
    assert(exitOk);
  }

  if (executeAll) {
    // sotDebugTrace::openFile();
    /* Removal of a line in the deficient-overshoot last stage, causing
     * a direct overshoot removal.
     */
    const int NB_STAGE = 3;
    const int RANKFREE[] = {5, 5, 5};
    const int RANKLINKED[] = {0, 4, 0};
    const int NR[] = {5, 5, 5};
    const int NC = 13;

    std::vector<Eigen::MatrixXd> J(NB_STAGE);
    std::vector<soth::VectorBound> b(NB_STAGE);
    generateDeficientDataSet(J, b, NB_STAGE, RANKFREE, RANKLINKED, NR, NC);

    soth::HCOD hcod(NC, NB_STAGE);
    hcod.pushBackStages(J, b);

    hcod.initialize();
    exitOk &= hcod.testRecomposition(&std::cout);

    for (int i = 0; i < 3; ++i) {
      assert(hcod.rank() == 13);
      assert(hcod[0].rank() == 5);
      assert(hcod[1].rank() == 5);
      assert(hcod[2].rank() == 3);
      hcod.downdate(2, 0);
    }
    assert(hcod.rank() == 12);
    assert(hcod[0].rank() == 5);
    assert(hcod[1].rank() == 5);
    assert(hcod[2].rank() == 2);

    exitOk &= clearIteralively(hcod);
    assert(exitOk);
  }

  if (executeAll) {
    // sotDebugTrace::openFile();
    /* Removal of a line above the deficient-overshoot last stage, causing
     * an indirect overshoot removal.
     */
    const int NB_STAGE = 3;
    const int RANKFREE[] = {5, 5, 5};
    const int RANKLINKED[] = {0, 0, 0};
    const int NR[] = {5, 5, 5};
    const int NC = 10;

    std::vector<Eigen::MatrixXd> J(NB_STAGE);
    std::vector<soth::VectorBound> b(NB_STAGE);
    generateDeficientDataSet(J, b, NB_STAGE, RANKFREE, RANKLINKED, NR, NC);

    soth::HCOD hcod(NC, NB_STAGE);
    hcod.pushBackStages(J, b);

    hcod.initialize();
    exitOk &= hcod.testRecomposition(&std::cout);

    for (int i = 0; i < 3; ++i) {
      assert(hcod.rank() == 10);
      assert(hcod[0].rank() == 5);
      assert(hcod[1].rank() == 5 - i);
      assert(hcod[2].rank() == i);
      hcod.downdate(1, 0);
    }
    assert(hcod.rank() == 10);
    assert(hcod[0].rank() == 5);
    assert(hcod[1].rank() == 2);
    assert(hcod[2].rank() == 3);

    exitOk &= clearIteralively(hcod);
    assert(exitOk);
  }

  if (executeAll) {
    // sotDebugTrace::openFile();
    /* Closing the first stage. */
    const int NB_STAGE = 3;
    const int RANKFREE[] = {5, 5, 5};
    const int RANKLINKED[] = {0, 0, 0};
    const int NR[] = {5, 5, 5};
    const int NC = 10;

    std::vector<Eigen::MatrixXd> J(NB_STAGE);
    std::vector<soth::VectorBound> b(NB_STAGE);
    generateDeficientDataSet(J, b, NB_STAGE, RANKFREE, RANKLINKED, NR, NC);

    soth::HCOD hcod(NC, NB_STAGE);
    hcod.pushBackStages(J, b);

    hcod.initialize();
    exitOk &= hcod.testRecomposition(&std::cout);

    for (int i = 0; i < NR[0]; ++i) {
      assert(hcod.rank() == 10);
      assert(hcod[0].rank() == 5 - i);
      assert(hcod[1].rank() == 5);
      assert(hcod[2].rank() == i);
      hcod.downdate(0, 0);
    }
    assert(hcod.rank() == 10);
    assert(hcod[0].rank() == 0);
    assert(hcod[1].rank() == 5);
    assert(hcod[2].rank() == 5);

    exitOk &= clearIteralively(hcod);
    assert(exitOk);
  }

  /* --- RANK DEFICIENCY ---------------------------------------------------- */
  /* --- RANK DEFICIENCY ---------------------------------------------------- */
  /* --- RANK DEFICIENCY ---------------------------------------------------- */

  if (executeAll) {
    // sotDebugTrace::openFile();
    /* Iterative construction from scrach.
     */
    const int NB_STAGE = 3;
    const int RANKFREE[] = {2, 2, 3};
    const int RANKLINKED[] = {0, 2, 2};
    const int NR[] = {4, 5, 6};
    const int NC = 12;

    std::vector<Eigen::MatrixXd> J(NB_STAGE);
    std::vector<soth::VectorBound> b(NB_STAGE);
    generateDeficientDataSet(J, b, NB_STAGE, RANKFREE, RANKLINKED, NR, NC);
    for (int s = 0; s < NB_STAGE; ++s)
      for (int i = 0; i < NR[s]; ++i) b[s][i] = std::make_pair(-i - 1, i + 1);

    soth::HCOD hcod(NC, NB_STAGE);
    hcod.pushBackStages(J, b);

    hcod.initialize();

    exitOk &= hcod.testRecomposition(&std::cout);
    if (sotDEBUGFLOW.outputbuffer.good()) hcod.show(sotDEBUGFLOW.outputbuffer);

    for (int s = 0; s < NB_STAGE; ++s)
      for (int i = 0; i < NR[s]; ++i)
        hcod.update(s, ConstraintRef(i, soth::Bound::BOUND_INF));

    assert(hcod.rank());
    assert(hcod[0].rank() == 2);
    assert(hcod[1].rank() == 2);
    assert(hcod[2].rank() == 3);

    exitOk &= clearIteralively(hcod);
    assert(exitOk);
  }

  exit((exitOk ? 0 : 1));
}

/* BUG LIST
 *
 *   - Insertion of a one-line matrix in the init: crash in the decompo QR.
 */

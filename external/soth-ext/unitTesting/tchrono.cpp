/* -------------------------------------------------------------------------- *
 *
 * Chrono test of the active search. Should be properly rewritten.
 *
 * -------------------------------------------------------------------------- */

//#define SOTH_DEBUG
//#define SOTH_DEBUG_MODE 45
#include "soth/HCOD.hpp"
#include "soth/Random.hpp"
#include "soth/debug.hpp"

#ifndef WIN32
#include <sys/time.h>
#endif  // WIN32

#include <Eigen/SVD>
#include "RandomGenerator.hpp"
#include "gettimeofday.hpp"
#include "soth/DestructiveColPivQR.hpp"

using namespace soth;
using std::endl;

/* -------------------------------------------------------------------------- */
namespace soth {
struct Now {
  long int sec, usec;
  Now(void) {
    struct timeval tref;
    gettimeofday(&tref, NULL);
    sec = tref.tv_sec;
    usec = tref.tv_usec;
  }
};
double operator-(const Now& t1, const Now& t0) {
  return ((double)(t1.sec - t0.sec)) * 1000.0 +
         ((double)(t1.usec - t0.usec) + 0.0) / 1000.0;
}
std::ostream& operator<<(std::ostream& os, const Now& now) {
  return os << now.sec << "' " << now.usec;
}

// std::ostream & chronout = std::cout;

// class chrono_context_ptr_t;
// struct Chrono
// {
//   chrono_context_ptr_t context;
//   std::string definition;
//   Chrono( chrono_context_ptr_t context,std::string definition )
//     : context(context),definition(definition)
//   {}
//   ~Chrono()
//   {
//     chronout << definition << ": " << (Now()-context->t0) << std::endl;
//   }
// };
// boost::smart_ptr<Chrono> chrono_ptr_t;

// struct ChronoContext
// {
//   time_t t0;
//   std::list<chrono_ptr_t> chronoList;
// };
// std::map< std::string,ChronoContext >

}  // namespace soth

/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
int main(int argc, char** argv) {
#ifndef NDEBUG
  sotDebugTrace::openFile();
#endif

  unsigned int NB_STAGE, NC;
  std::vector<unsigned int> NR, RANKLINKED, RANKFREE;
  std::vector<Eigen::MatrixXd> J;
  std::vector<soth::VectorBound> b;

  if ((argc == 3) && std::string(argv[1]) == "-file") {
    readProblemFromFile(argv[2], J, b, NB_STAGE, NR, NC);
  } else {
    /* Initialize the seed. */
    struct timeval tv;
    gettimeofday(&tv, NULL);
    int seed = (int)(tv.tv_usec % 7919);  //= 7594;
    if (argc == 2) {
      seed = atoi(argv[1]);
    }
    std::cout << "seed = " << seed << std::endl;
    soth::Random::setSeed(seed);

    /* Decide the size of the problem. */
    generateRandomProfile(NB_STAGE, RANKFREE, RANKLINKED, NR, NC);
    /* Initialize J and b. */
    generateDeficientDataSet(J, b, NB_STAGE, RANKFREE, RANKLINKED, NR, NC);
  }

  std::cout << "NB_STAGE=" << NB_STAGE << ",  NC=" << NC << endl;
  std::cout << endl;
  for (unsigned int i = 0; i < NB_STAGE; ++i) {
    std::cout << "J" << i + 1 << " = " << (soth::MATLAB)J[i] << std::endl;
    std::cout << "e" << i + 1 << " = " << b[i] << ";" << std::endl;
  }

  // if( seed==317)  assert( std::abs(J[0](0,0)-(-1.1149))<1e-5 );

  /* SOTH structure construction. */
  soth::HCOD hcod(NC, NB_STAGE);
  for (unsigned int i = 0; i < NB_STAGE; ++i) {
    hcod.pushBackStage(J[i], b[i]);
  }
  hcod.setNameByOrder("stage_");

  /* --- TESTS -------------------------------------------------------------- */
#ifdef NDEBUG
  const unsigned int nbTest = 1000;
#else
  const unsigned int nbTest = 10;
#endif

  VectorXd solution;

  Now t0;
  for (unsigned int i = 0; i < nbTest; ++i) {
    hcod.initialize();
  }

  Now t1;
  for (unsigned int i = 0; i < nbTest; ++i) hcod.Y.computeExplicitly();

  Now t2;
  for (unsigned int i = 0; i < nbTest; ++i) hcod.computeSolution(true);

  Now t3;
  // for( unsigned int i=0;i<nbTest;++i )
  //   hcod.reset();

  Now t4;
  for (unsigned int i = 0; i < nbTest; ++i) hcod.computeStep();

  Now t5;
  for (unsigned int i = 0; i < nbTest; ++i)
    hcod.computeLagrangeMultipliers(hcod.nbStages());

  Now t6;
  for (unsigned int i = 0; i < nbTest; ++i) hcod.search(hcod.nbStages());

  Now t7;
  for (unsigned int i = 0; i < nbTest; ++i) hcod.activeSearch(solution);

  Now tf;
  std::cout << "HCOD = " << (t1 - t0) << "us" << endl;
  std::cout << "Y = " << (t2 - t1) << "us" << endl;
  std::cout << "Inv = " << (t3 - t2) << "us" << endl;
  std::cout << "reset = " << (t4 - t3) << "us" << endl;
  std::cout << "check = " << (t5 - t4) << "us" << endl;
  std::cout << "lagrange = " << (t6 - t5) << "us" << endl;
  std::cout << "l<0 = " << (t7 - t6) << "us" << endl;
  std::cout << "activesearch = " << (tf - t7) << "us" << endl;

  if (sotDEBUGFLOW.outputbuffer.good()) hcod.show(sotDEBUGFLOW.outputbuffer);

  std::cout << " --- UNITCHRONO --------------------------------- " << endl;

  {  // simple QR test
    const unsigned int Nctest = NC;
    const unsigned int Nrtest = std::min(NC, (unsigned int)21);
    MatrixXd A = MatrixXd::Random(Nrtest, Nctest);
    MatrixXd Y(Nctest, Nctest);

    Now t3;
    for (unsigned int i = 0; i < nbTest; ++i)
      Eigen::DestructiveColPivQR<MatrixXd, MatrixXd> mQR(A, Y, 1e-6);
    Now tf;
    std::cout << "QR " << Nctest << "x" << Nrtest << " = " << tf - t3 << "us"
              << endl;
  }

  {  // simple Linv test
    const int Nctest = NC;
    const int Nrtest = std::min(NC, (unsigned int)21);
    MatrixXd A = MatrixXd::Random(Nrtest, Nctest);
    VectorXd b(Nrtest);

    Now t3;
    for (unsigned int i = 0; i < nbTest; ++i)
      solveInPlaceWithLowerTriangular(A.leftCols(Nrtest), b);
    Now tf;
    std::cout << "Linv " << Nrtest << "x" << Nrtest << " = " << tf - t3 << "us"
              << endl;
  }

  {  // Double test with matrix multiplication, just to be sure.
    const int Nctest = NC;
    const int Nrtest = 21;
    MatrixXd A = MatrixXd::Random(Nrtest, Nctest);
    MatrixXd B = MatrixXd::Random(Nctest, Nctest);

    MatrixXd C(Nrtest, Nctest);
    Now t3;
    for (unsigned int i = 0; i < nbTest; ++i) C = A * B;
    Now tf;
    std::cout << Nctest << "x" << Nrtest << " = " << tf - t3 << "us" << endl;
    std::cout << C(0, 0);
  }
}

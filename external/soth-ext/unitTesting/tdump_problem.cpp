/* -------------------------------------------------------------------------- *
 *
 * Generate a .txt file with a random problem. The key of the randomizer is
 * given by command line.
 *
 * -------------------------------------------------------------------------- */
#define SOTH_DEBUG
#define SOTH_DEBUG_MODE 45
#include "RandomGenerator.hpp"
#include "soth/HCOD.hpp"
#include "soth/Random.hpp"
#include "soth/debug.hpp"

#ifndef WIN32
#include <sys/time.h>
#endif  // WIN32

#include <Eigen/SVD>
#include <fstream>
#include "gettimeofday.hpp"

using namespace soth;
using std::cerr;
using std::cout;
using std::endl;

/* Compute [m1;m2] from m1 and m2 of same col number. */
template <typename D1, typename D2>
MatrixXd stack(const MatrixBase<D1>& m1, const MatrixBase<D2>& m2) {
  assert(m1.cols() == m2.cols());
  const int m1r = m1.rows(), m2r = m2.rows(), mr = m1r + m2r, mc = m1.cols();
  MatrixXd res(mr, mc);
  for (int i = 0; i < m1r; ++i)
    for (int j = 0; j < mc; ++j) res(i, j) = m1(i, j);
  for (int i = 0; i < m2r; ++i)
    for (int j = 0; j < mc; ++j) res(m1r + i, j) = m2(i, j);
  return res;
}

/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
int main(int argc, char** argv) {
  {
    struct timeval tv;
    gettimeofday(&tv, NULL);

    int seed = (int)(tv.tv_usec % 7919);  //= 7594;
    if (argc == 2) {
      seed = atoi(argv[1]);
    }
    std::cout << "seed = " << seed << std::endl;
    soth::Random::setSeed(seed);
  }

  /* Decide the size of the problem. */
  unsigned int NB_STAGE, NC;
  std::vector<unsigned int> NR, RANKLINKED, RANKFREE;
  soth::generateRandomProfile(NB_STAGE, RANKFREE, RANKLINKED, NR, NC);

  /* Initialize J and b. */
  std::vector<Eigen::MatrixXd> J(NB_STAGE);
  std::vector<soth::VectorBound> b(NB_STAGE);
  generateDeficientDataSet(J, b, NB_STAGE, RANKFREE, RANKLINKED, NR, NC);

  std::ofstream fout("/tmp/soth.txt");
  fout << "variable size " << NC << endl << endl;

  for (unsigned int s = 0; s < NB_STAGE; ++s) {
    fout << "level" << endl << endl;

    int nbEqualities = 0;
    for (unsigned int r = 0; r < NR[s]; ++r) {
      if (b[s][r].getType() == Bound::BOUND_TWIN) nbEqualities++;
    }

    fout << "equalities " << nbEqualities << endl << endl;
    for (unsigned int r = 0; r < NR[s]; ++r) {
      if (b[s][r].getType() == Bound::BOUND_TWIN) {
        fout << J[s].row(r) << "   " << b[s][r].getBound(Bound::BOUND_TWIN)
             << endl;
      }
    }
    fout << endl << "inequalities " << NR[s] - nbEqualities << endl << endl;
    for (unsigned int r = 0; r < NR[s]; ++r) {
      switch (b[s][r].getType()) {
        case Bound::BOUND_TWIN:
          break;
        case Bound::BOUND_INF:
          fout << J[s].row(r) << "   " << b[s][r].getBound(Bound::BOUND_INF)
               << " 1e25" << endl;
          break;
        case Bound::BOUND_SUP:
          fout << J[s].row(r) << "   " << -1e25 << " "
               << b[s][r].getBound(Bound::BOUND_SUP) << endl;
          break;
        case Bound::BOUND_DOUBLE:
          fout << J[s].row(r) << "   " << b[s][r].getBound(Bound::BOUND_INF)
               << " " << b[s][r].getBound(Bound::BOUND_SUP) << endl;
          break;
        case Bound::BOUND_NONE:
          assert(false && "Never happening.");
          break;
      }
    }
    fout << endl;
  }

  fout << endl << "end" << endl << endl;
}

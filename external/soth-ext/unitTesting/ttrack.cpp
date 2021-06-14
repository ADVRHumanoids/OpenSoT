/* -------------------------------------------------------------------------- *
 *
 * Compute the optimum of a time-varying problem, to test the active-set
 * initial guess.
 *
 * -------------------------------------------------------------------------- */
#define SOTH_DEBUG
#define SOTH_DEBUG_MODE 45
#include "soth/BaseY.hpp"
#include "soth/BasicStage.hpp"
#include "soth/HCOD.hpp"
#include "soth/Random.hpp"
#include "soth/debug.hpp"

#ifndef WIN32
#include <sys/time.h>
#endif  // WIN32

#include <Eigen/SVD>
#include "RandomGenerator.hpp"
#include "gettimeofday.hpp"

using namespace soth;
using std::endl;

/* -------------------------------------------------------------------------- */
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
  return (double)(t1.sec - t0.sec) * 1000.0 +
         (double)(t1.usec - t0.usec + 0.0) / 1000.0;
}
std::ostream& operator<<(std::ostream& os, const Now& now) {
  return os << now.sec << "' " << now.usec;
}

}  // namespace soth

/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */

void increaseProblem(
    const double& DELTA, const unsigned int& NB_STAGE, const unsigned int& NC,
    const std::vector<unsigned int>& NR,
    const std::vector<unsigned int>& RANKLINKED,
    const std::vector<unsigned int>& RANKFREE,
    const std::vector<unsigned int>& NRA, std::vector<Eigen::MatrixXd>& J,
    std::vector<Eigen::MatrixXd>& Xhifree, std::vector<Eigen::MatrixXd>& Jfree,
    std::vector<Eigen::MatrixXd>& Xhilinked,
    std::vector<Eigen::MatrixXd>& Alinked, std::vector<soth::VectorBound>& b) {
  for (unsigned int s = 0; s < NB_STAGE; ++s) {
    /* NRA[s] is the size of all previous stages 0:s-1 stacked. */
    assert((RANKFREE[s] > 0) || (RANKLINKED[s] > 0));

    Eigen::MatrixXd delta_Xhifree(NR[s], RANKFREE[s]);
    Eigen::MatrixXd delta_Jfree(RANKFREE[s], NC);
    Eigen::MatrixXd delta_Xhilinked(NR[s], RANKLINKED[s]);
    Eigen::MatrixXd delta_Alinked(RANKLINKED[s], NRA[s]);

    J[s].setZero();
    if (RANKFREE[s] > 0) {
      soth::MatrixRnd::randomize(delta_Xhifree);
      soth::MatrixRnd::randomize(delta_Jfree);
      Jfree[s] += DELTA * delta_Jfree;
      Xhifree[s] += DELTA * delta_Xhifree;
      J[s] = Xhifree[s] * Jfree[s];
    }
    if (RANKLINKED[s] > 0) {
      soth::MatrixRnd::randomize(delta_Xhilinked);
      soth::MatrixRnd::randomize(delta_Alinked);
      Xhilinked[s] += DELTA * delta_Xhilinked;
      Alinked[s] += DELTA * delta_Alinked;
      for (unsigned int sb = 0; sb < s; ++sb) {
        J[s] += Xhilinked[s] *
                Alinked[s].block(0, NRA[sb], RANKLINKED[s], NR[sb]) * J[sb];
      }
    }

    for (unsigned int i = 0; i < NR[s]; ++i) {
      double x = 10 * DELTA * (Random::rand<double>() * 2 - 1);
      double y = 10 * DELTA * (Random::rand<double>() * 2 - 1);
      Bound::bound_t btype = b[s][i].getType();
      switch (btype) {
        case Bound::BOUND_INF:
        case Bound::BOUND_SUP:
        case Bound::BOUND_TWIN:
          b[s][i] = Bound(b[s][i].getBound(btype) + x, btype);
          break;
        case Bound::BOUND_DOUBLE:
          b[s][i] = Bound(b[s][i].getBound(Bound::BOUND_INF) + x,
                          b[s][i].getBound(Bound::BOUND_SUP) + y);
          break;
        case Bound::BOUND_NONE:
          assert(false && "Could never happen.");
          break;
      }
    }
  }
}
/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */

struct NotificationToCout {
  static int time;
  static bool touched;
  void operator()(std::string stage, soth::ConstraintRef cst,
                  std::string event) {
    touched = true;
    std::cout << "At " << stage << ", " << cst << ", time " << time << ": "
              << event << std::endl;
  }
  void inc() {
    if (touched) {
      touched = false;
      std::cout << "--" << std::endl;
    }
    time++;
  }
};
int NotificationToCout::time = 0;
bool NotificationToCout::touched = 0;

/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
int main(int argc, char** argv) {
#ifndef NDEBUG
  sotDebugTrace::openFile();
#endif

  unsigned int NB_STAGE, NC;
  std::vector<unsigned int> NR, RANKLINKED, RANKFREE, NRA;
  std::vector<Eigen::MatrixXd> J, Xhifree, Jfree, Xhilinked, Alinked;
  std::vector<soth::VectorBound> b;

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
  J.resize(NB_STAGE);
  Xhifree.resize(NB_STAGE);
  Jfree.resize(NB_STAGE);
  Xhilinked.resize(NB_STAGE);
  Alinked.resize(NB_STAGE);
  NRA.resize(NB_STAGE);
  b.resize(NB_STAGE);

  /* Bulid the initial values. */
  for (unsigned int s = 0; s < NB_STAGE; ++s) {
    /* NRA[s] is the size of all previous stages 0:s-1 stacked. */
    if (s == 0)
      NRA[s] = 0;
    else
      NRA[s] = NRA[s - 1] + NR[s - 1];
    b[s].resize(NR[s]);
    assert((RANKFREE[s] > 0) || (RANKLINKED[s] > 0));

    J[s].resize(NR[s], NC);
    Xhifree[s].resize(NR[s], RANKFREE[s]);
    Jfree[s].resize(RANKFREE[s], NC);
    Xhilinked[s].resize(NR[s], RANKLINKED[s]);
    Alinked[s].resize(RANKLINKED[s], NRA[s]);

    J[s].setZero();
    if (RANKFREE[s] > 0) {
      soth::MatrixRnd::randomize(Xhifree[s]);
      soth::MatrixRnd::randomize(Jfree[s]);
      J[s] += Xhifree[s] * Jfree[s];
    }
    if (RANKLINKED[s] > 0) {
      soth::MatrixRnd::randomize(Xhilinked[s]);
      soth::MatrixRnd::randomize(Alinked[s]);
      for (unsigned int sb = 0; sb < s; ++sb) {
        J[s] += Xhilinked[s] *
                Alinked[s].block(0, NRA[sb], RANKLINKED[s], NR[sb]) * J[sb];
      }
    }

    for (unsigned int i = 0; i < NR[s]; ++i) {
      double x = Random::rand<double>() * 2;   // DEBUG: should b U*2-1
      double y = Random::rand<double>() * -2;  // DEBUG
      int btype = randu(1, 4);
      switch (btype) {
        case 1:  // =
          b[s][i] = x;
          break;
        case 2:  // <
          b[s][i] = soth::Bound(y, soth::Bound::BOUND_INF);
          break;
        case 3:  // >
          b[s][i] = soth::Bound(x, soth::Bound::BOUND_SUP);
          break;
        case 4:  // < <
          b[s][i] = std::make_pair(std::min(x, y), std::max(x, y));
          break;
      }
    }
  }

  std::cout << "NB_STAGE=" << NB_STAGE << ",  NC=" << NC << endl;
  for (unsigned int i = 0; i < NB_STAGE; ++i) {
    sotDEBUG(1) << "J" << i + 1 << " = " << (soth::MATLAB)J[i] << std::endl;
    sotDEBUG(1) << "e" << i + 1 << " = " << b[i] << ";" << std::endl;
  }

  /* SOTH structure construction. */
  soth::HCOD hcod(NC, NB_STAGE);
  for (unsigned int i = 0; i < NB_STAGE; ++i) {
    hcod.pushBackStage(NR[i], J[i].data());
  }
  hcod.setNameByOrder("stage_");
  NotificationToCout coutListen;
  hcod.notifiorRegistration(coutListen);

  /* Initial solve */
  VectorXd solution;
  for (unsigned int i = 0; i < NB_STAGE; ++i) {
    hcod[i].getBoundsInternal() = b[i];
    hcod[i].setInitialActiveSet();
    sotDEBUG(1) << "J" << i + 1 << " = " << (soth::MATLAB)hcod[i].getJ()
                << std::endl;
    sotDEBUG(1) << "b" << i + 1 << " = " << hcod[i].getBounds() << std::endl;
  }
  hcod.activeSearch(solution);

  /* --- Tracking --- */
  for (unsigned int t = 0; t < 100000; t++) {
    sotDEBUG(1) << " --- TIME t=" << t << " -----------------------------"
                << std::endl;
    coutListen.inc();
    increaseProblem(1e-3, NB_STAGE, NC, NR, RANKLINKED, RANKFREE, NRA, J,
                    Xhifree, Jfree, Xhilinked, Alinked, b);
    for (unsigned int i = 0; i < NB_STAGE; ++i)
      hcod[i].getBoundsInternal() = b[i];
    hcod.activeSearch(solution);
  }
}

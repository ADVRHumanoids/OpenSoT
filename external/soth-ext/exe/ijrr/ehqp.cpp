/* -------------------------------------------------------------------------- *
 *
 * Time comparisons for IJRR 2013 (sub) paper.
 *
 * -------------------------------------------------------------------------- */
#define SOTH_DEBUG
#define SOTH_DEBUG_MODE 45
#include "COD.hpp"
#include "RandomGenerator.hpp"
#include "soth/HCOD.hpp"
#include "soth/Random.hpp"
#include "soth/debug.hpp"

#ifndef WIN32
#include <sys/time.h>
#endif  // WIN32

#include <iostream>
#include <sstream>
#include "gettimeofday.hpp"

using namespace soth;
using std::cerr;
using std::cout;
using std::endl;

/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
#include <Eigen/SVD>
using Eigen::JacobiSVD;

VectorXd svdSolve(JacobiSVD<MatrixXd> &svd, VectorXd &ref,
                  double *rankPtr = NULL, double EPS = 1e-6) {
  typedef JacobiSVD<MatrixXd> SVDType;
  const JacobiSVD<MatrixXd>::MatrixUType &U = svd.matrixU();
  const JacobiSVD<MatrixXd>::MatrixVType &V = svd.matrixV();
  const JacobiSVD<MatrixXd>::SingularValuesType &s = svd.singularValues();
  // std::cout << (MATLAB)U<< (MATLAB)s<<(MATLAB)V << std::endl;

  const int N = V.rows(), M = U.rows(), S = std::min(M, N);

  int rank = 0;
  while (rank < S && s[rank] > EPS) rank++;

  VectorXd sinv = s.head(rank).array().inverse();

  // std::cout << "U = " << (MATLAB)U.leftCols(rank) << std::endl;
  // std::cout << "V = " << (MATLAB)V.leftCols(rank) << std::endl;
  // std::cout << "S = " << (MATLAB)(MatrixXd)sinv.asDiagonal() << std::endl;

  VectorXd x;
  x = V.leftCols(rank) * sinv.asDiagonal() * U.leftCols(rank).transpose() * ref;

  if (rankPtr != NULL) *rankPtr = rank;

  return x;
}

int stageIter = 0;
int phase = 3;
int ratioSvd = 1;

void siciliano(const std::vector<Eigen::MatrixXd> &A,
               std::vector<soth::VectorXd> &b, const unsigned int NB_STAGE,
               const std::vector<unsigned int> &RANKFREE,
               const std::vector<unsigned int> & /*RANKLINKED*/,
               const std::vector<unsigned int> & /*NR*/,
               const unsigned int NC) {
  VectorXd x(NC);
  x.setZero();
  MatrixXd P(NC, NC);
  P.setIdentity();
  int rank = NC;
  COD Akcod(ComputeFullU | ComputeThinV);
  MatrixXd AkP(NC * 2, NC);

  for (unsigned int k = 0; k < NB_STAGE; ++k) {
    for (int i = 0; i < ratioSvd; ++i) Akcod.compute(A[k] * P);

    if (phase > 1) {
      VectorXd blim = b[k] - A[k] * x;
      x += Akcod.solve(blim);
    }

    if (phase > 2) {
      P -= Akcod.matrixVr() * Akcod.matrixVr().transpose();
    }

    rank = rank - RANKFREE[k];
  }
}

void hsvd(const std::vector<Eigen::MatrixXd> &A, std::vector<soth::VectorXd> &b,
          const unsigned int NB_STAGE,
          const std::vector<unsigned int> &RANKFREE,
          const std::vector<unsigned int> & /*RANKLINKED*/,
          const std::vector<unsigned int> & /*NR*/, const unsigned int NC) {
  VectorXd x(NC);
  x.setZero();
  MatrixXd Z(NC, NC);
  Z.setIdentity();
  MatrixXd Ztmp(NC, NC);
  MatrixXd P(NC, NC);
  P.setIdentity();
  int rank = NC;
  COD Akcod(ComputeFullU);
  for (unsigned int k = 0; k < NB_STAGE; ++k) {
    /*
      JacobiSVD<MatrixXd> Aksvd(A[k]*Z, ComputeThinU | ComputeThinV);
      VectorXd blim = b[k] - A[k]*x;
      x += Z*svdSolve(Aksvd,blim,&rank);
      Z = Aksvd.matrixV().rightCols(rank);
    */

    for (int i = 0; i < ratioSvd; ++i) Akcod.compute(A[k] * Z.rightCols(rank));

    if (phase > 1) {
      VectorXd blim = b[k] - A[k] * x;
      x += Z * Akcod.solve(blim);
    }

    if (phase > 2) {
      MatrixXd::ColsBlockXpr Zr = Z.rightCols(rank);
      Akcod.applyMatrixV(Zr);
    }

    rank = rank - RANKFREE[k];
  }
}

void hcod(std::vector<Eigen::MatrixXd> &A, std::vector<soth::VectorXd> &b,
          const unsigned int NB_STAGE,
          const std::vector<unsigned int> &RANKFREE,
          const std::vector<unsigned int> & /*RANKLINKED*/,
          const std::vector<unsigned int> & /*NR*/, const unsigned int NC) {
  VectorXd x(NC);
  x.setZero();
  VectorXd y(NC);
  y.setZero();
  MatrixXd Z(NC, NC);
  Z.setIdentity();
  MatrixXd Ztmp(NC, NC);
  MatrixXd P(NC, NC);
  P.setIdentity();
  double rank = NC;
  std::vector<COD> Akcod(NB_STAGE);
  std::vector<int> na(NB_STAGE + 1);
  na[0] = NC;

  for (unsigned int k = 0; k < NB_STAGE; ++k) {
    /*
      AkZ = Ak Z1 Z2 ... Zk-1
      y = L^+ b
      x = Z y = Z1 Z2 ... Zk-1 y
    */

    Akcod[k].options(ComputeFullU);

    for (unsigned int i = 0; i < k; ++i) {
      // std::cout << "AkZ " << i << "," << k<< std::endl;
      MatrixXd::ColsBlockXpr AkZ = A[k].rightCols(na[i]);
      Akcod[i].applyMatrixV(AkZ);
    }
    // std::cout << "decomp" << na[k] << std::endl;
    Akcod[k].compute(A[k].rightCols(na[k]));

    if (phase > 1) {
      VectorXd blim = b[k] - A[k] * y;
      // std::cout << "solve y" << std::endl;
      y.segment(NC - na[k], RANKFREE[k]) = Akcod[k].solve(blim, true);
    }

    na[k + 1] = na[k] - RANKFREE[k];
    rank = rank - RANKFREE[k];
  }

  if (phase > 2) {
    // std::cout << "Phase 3 " << std::endl;
    for (int i = NB_STAGE - 1; i >= 0; --i) {
      // std::cout << "Zyk " << i << std::endl;
      y.tail(na[i]).applyOnTheLeft(Akcod[i].qrv.householderQ());
    }
  }
}

/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */

#include <Eigen/SVD>
#include <boost/program_options.hpp>

int main(int argc, char **argv) {
#ifndef NDEBUG
  sotDebugTrace::openFile();
#endif

  unsigned int NB_STAGE, NC;
  std::vector<unsigned int> NR, RANKLINKED, RANKFREE;
  std::vector<Eigen::MatrixXd> J;
  std::vector<Eigen::VectorXd> e;
  std::vector<soth::VectorBound> b;

  boost::program_options::variables_map optionMap;
  {
    namespace po = boost::program_options;
    po::options_description desc("trandom options");
    desc.add_options()("help", "produce help message")(
        "seed,s", po::value<int>(), "specify the seed of the random generator")(
        "size,S", po::value<int>(), "size of the problem to try")(
        "method,m", po::value<int>(), "0 = SICILIANO COD, 1 = COD Z, 2 = HCOD")(
        "denoise,n", po::value<int>(),
        "Run n times the algo to remove the computation-time noise")(
        "ratio,r", po::value<int>(), "ratio of the SVD wrt COD")(
        "phase,p", po::value<int>(),
        "Phase of the algo 1 = Decompo, 2 = inversion, 3 = projection");

    po::positional_options_description p;
    p.add("seed", -1);
    p.add("method", 0);
    p.add("size", 20);
    p.add("denoise", 1);

    po::store(
        po::command_line_parser(argc, argv).options(desc).positional(p).run(),
        optionMap);
    po::notify(optionMap);

    if (optionMap.count("help")) {
      std::cout << desc << std::endl;
      exit(0);
    }

    if (optionMap.count("ratio")) ratioSvd = optionMap["ratio"].as<int>();
    if (optionMap.count("phase")) phase = optionMap["phase"].as<int>();
  }

  /* Initialize the seed. */
  struct timeval tv;
  gettimeofday(&tv, NULL);
  int seed = tv.tv_usec % 7919;
  if (optionMap.count("seed")) {
    seed = optionMap["seed"].as<int>();
  }
  std::cout << "seed = " << seed << std::endl;
  soth::Random::setSeed(seed);

  /* Decide the size of the problem. */
  generateFixedSizeRandomProfile(optionMap["size"].as<int>(), 1.2, 0.8, 0.5,
                                 NB_STAGE, RANKFREE, RANKLINKED, NR, NC);
  std::cout << "NB_STAGE=" << NB_STAGE << ",  NC=" << NC << endl;
  for (int k = 0; k < (int)NB_STAGE; k++) {
    sotDEBUG(20) << RANKFREE[k] << " " << RANKLINKED[k] << " " << NR[k] << endl;
  }

  /* Initialize J and b. */
  generateDeficientDataSet(J, b, NB_STAGE, RANKFREE, RANKLINKED, NR, NC);

  cout << endl;
  e.resize(NB_STAGE);
  for (unsigned int i = 0; i < NB_STAGE; ++i) {
    // std::cout << "e"<<i+1<< " = " << b[i] << ";"<<std::endl;
    e[i].resize(NR[i]);
    soth::MatrixRnd::randomize(e[i]);
#ifndef NDEBUG
    std::cout << "J" << i + 1 << " = " << (soth::MATLAB)J[i] << std::endl;
    std::cout << "e" << i + 1 << " = " << (MATLAB)e[i] << ";" << std::endl;
#endif
  }

  // std::cout << "*** Siciliano ***" << std::endl;
  // siciliano(J,e,NB_STAGE,RANKFREE,RANKLINKED,NR,NC);

  /* --- CHRONO --- */
  struct timeval t0, t1;
  double totalTime = 0;
  std::ostringstream iss;
  iss << "/tmp/ttro_" << optionMap["size"].as<int>() << "_"
      << optionMap["method"].as<int>() << "_" << optionMap["phase"].as<int>()
      << "_" << optionMap["ratio"].as<int>() << ".dat";

  std::ofstream datfile(iss.str().c_str());
  {
    std::ofstream ftrace("/tmp/sici.dat");  // reset
  }

  for (int shoot = 0; shoot < 1000; shoot++) {
    double seed = Random::rand<int>();  // % 704819;
    soth::Random::setSeed((int)seed);
    std::cout << "s" << seed << ":" << std::flush;

    generateFixedSizeRandomProfile(optionMap["size"].as<int>(), 1.2, 0.8, 0.5,
                                   NB_STAGE, RANKFREE, RANKLINKED, NR, NC);
    generateDeficientDataSet(J, b, NB_STAGE, RANKFREE, RANKLINKED, NR, NC);
    e.resize(NB_STAGE);
    for (unsigned int i = 0; i < NB_STAGE; ++i) {
      e[i].resize(NR[i]);
      soth::MatrixRnd::randomize(e[i]);
      for (unsigned int r = 0; r < NR[i]; ++r) b[i][r] = e[i][r];
    }

    HCOD hsolver(NC, NB_STAGE);
    for (unsigned int i = 0; i < NB_STAGE; ++i)
      hsolver.pushBackStage(J[i], b[i]);

    hsolver.reset();
    hsolver.setInitialActiveSet();

    gettimeofday(&t0, NULL);
    for (int noise = 0; noise < optionMap["denoise"].as<int>(); ++noise)
      switch (optionMap["method"].as<int>()) {
        case 0:
          siciliano(J, e, NB_STAGE, RANKFREE, RANKLINKED, NR, NC);
          break;
        case 1:
          hsvd(J, e, NB_STAGE, RANKFREE, RANKLINKED, NR, NC);
          break;
        case 2:
          hsolver.initialize();
          if (phase > 1) hsolver.Y.computeExplicitly();
          if (phase > 2) hsolver.computeSolution();
          break;
        case 3:
          hcod(J, e, NB_STAGE, RANKFREE, RANKLINKED, NR, NC);
          break;
      }

    gettimeofday(&t1, NULL);
    double time = (t1.tv_sec - t0.tv_sec) + (t1.tv_usec - t0.tv_usec) / 1.0e6;
    time /= optionMap["denoise"].as<int>();
    totalTime += time;

    datfile << NB_STAGE << "        \t " << time << "\t" << seed << std::endl;
  }
  std::cout << totalTime << std::endl;
}

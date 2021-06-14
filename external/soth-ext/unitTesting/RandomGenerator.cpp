/* -------------------------------------------------------------------------- *
 *
 * Generation of random problems.
 *
 * -------------------------------------------------------------------------- */

#define SOTH_DEBUG
#define SOTH_DEBUG_MODE 45
#include "RandomGenerator.hpp"
#include <fstream>
#include "../include/soth/Random.hpp"
#include "../include/soth/debug.hpp"

#ifdef WIN32
inline double round(double d) { return floor(d + 0.5); }
#endif /* WIN32 */

namespace soth {
using std::endl;

/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
void generateDeficientDataSet(std::vector<Eigen::MatrixXd> &J,
                              std::vector<soth::VectorBound> &b,
                              const unsigned int NB_STAGE,
                              const std::vector<unsigned int> &RANKFREE,
                              const std::vector<unsigned int> &RANKLINKED,
                              const std::vector<unsigned int> &NR,
                              const unsigned int NC) {
  /* Initialize J and b. */
  J.resize(NB_STAGE);
  b.resize(NB_STAGE);

  for (unsigned int s = 0; s < NB_STAGE; ++s) {
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
      for (unsigned int sb = 0; sb < s; ++sb) {
        Eigen::MatrixXd Alinked(RANKLINKED[s], NR[sb]);
        soth::MatrixRnd::randomize(Alinked);
        J[s] += Xhilinked * Alinked * J[sb];
      }
    }

    for (unsigned int i = 0; i < NR[s]; ++i) {
      double x = Random::rand<double>() * 2;   // DEBUG: should b U*2-1
      double y = Random::rand<double>() * -2;  // DEBUG
      int btype = randu(1, 4);
      // if( s==0 ) btype= randu(2,4); // DEBUG
      // if( s!=0 ) btype= 1; // DEBUG
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
}

void generateDeficientDataSet(std::vector<Eigen::MatrixXd> &J,
                              std::vector<soth::VectorBound> &b,
                              const unsigned int NB_STAGE, const int RANKFREE[],
                              const int RANKLINKED[], const int NR[],
                              const unsigned int NC) {
  std::vector<unsigned int> RF(NB_STAGE), RL(NB_STAGE), NRV(NB_STAGE);
  for (unsigned int i = 0; i < NB_STAGE; ++i) {
    RF[i] = RANKFREE[i];
    RL[i] = RANKLINKED[i];
    NRV[i] = NR[i];
  }
  generateDeficientDataSet(J, b, NB_STAGE, RF, RL, NRV, NC);
}

void generateRandomProfile(unsigned int &nbStage,
                           std::vector<unsigned int> &rankfree,
                           std::vector<unsigned int> &ranklinked,
                           std::vector<unsigned int> &nr, unsigned int &nc) {
  //    nc = Random::rand<int>() % 30 +4 ; //%  50 + 6;
  int lnc = Random::rand<int>();

  nc = lnc % 30 + 4;
  nbStage = randu(1, 1 + nc / 4);

  sotDEBUG(1) << "lnc = " << lnc << endl;
  sotDEBUG(1) << "nc = " << nc << endl;
  sotDEBUG(1) << "nbStage = " << nbStage << endl;

  const int NR = std::max(2, (int)round((0. + nc) / nbStage * .7));
  const int RANKFREE = std::max(1, (int)round(whiteNoise(NR / 2, 0.2)));
  const int RANKLINKED = (int)round(whiteNoise(NR, 1)) + 1;
  sotDEBUG(1) << "mean_NR = " << NR << "; mean_RF = " << RANKFREE
              << "; mean_RL = " << RANKLINKED << endl;

  rankfree.resize(nbStage);
  ranklinked.resize(nbStage);
  nr.resize(nbStage);
  for (unsigned int i = 0; i < nbStage; ++i) {
    if (Random::rand<double>() < 0.7) {
      sotDEBUG(1) << i << ": normal rank." << endl;
      rankfree[i] = randu(0, RANKFREE);      // whiteNoise( RANKFREE,3 );
      ranklinked[i] = randu(0, RANKLINKED);  // whiteNoise( RANKLINKED,3 );
      nr[i] = randu(1, NR);
    } else if (Random::rand<double>() < 0.05) {
      sotDEBUG(1) << i << ":  rank def." << endl;
      rankfree[i] = randu(0, RANKFREE);      // whiteNoise( RANKFREE,3 );
      ranklinked[i] = randu(0, RANKLINKED);  // whiteNoise( RANKLINKED,3 );
      nr[i] = randu(1, NR);
    } else {
      sotDEBUG(1) << i << ": full rank." << endl;
      ranklinked[i] = whiteNoise(RANKLINKED, 3);
      nr[i] = randu(1, NR);
      rankfree[i] = nr[i];
    }
    rankfree[i] = std::min(nr[i], rankfree[i]);
    ranklinked[i] = std::min(nr[i], ranklinked[i]);
    if (i == 0) {
      ranklinked[i] = 0;
      rankfree[i] = std::max(1, (int)rankfree[i]);
    } else
      ranklinked[i] = std::min(ranklinked[i], nr[i - 1]);
    if (rankfree[i] == 0) ranklinked[i] = std::max(1, (int)ranklinked[i]);
    sotDEBUG(1) << "rf" << i << " = " << rankfree[i] << ";   rl" << i << " = "
                << ranklinked[i] << ";  nr" << i << " = " << nr[i] << endl;

    // DEBUG
    if (rankfree[i] == 0) rankfree[i]++;
  }
}

void randomProblem(std::vector<Eigen::MatrixXd> &J,
                   std::vector<soth::VectorBound> &b, bool verbose) {
  unsigned int NB_STAGE, NC;
  std::vector<unsigned int> NR, RANKLINKED, RANKFREE;
  generateRandomProfile(NB_STAGE, RANKFREE, RANKLINKED, NR, NC);

  /* Initialize J and b. */
  generateDeficientDataSet(J, b, NB_STAGE, RANKFREE, RANKLINKED, NR, NC);

  if (verbose) {
    for (unsigned int i = 0; i < NB_STAGE; ++i) {
      std::cout << RANKFREE[i] << "/" << NR[i] << ",  ";
    }

    std::cout << "NB_STAGE=" << NB_STAGE << ",  NC=" << NC << endl;
    std::cout << endl;
    for (unsigned int i = 0; i < NB_STAGE; ++i) {
      std::cout << "J" << i + 1 << " = " << (soth::MATLAB)J[i] << std::endl;
      std::cout << "e" << i + 1 << " = " << b[i] << ";" << std::endl;
    }
  }
}

void readProblemFromFile(const std::string name,
                         std::vector<Eigen::MatrixXd> &J,
                         std::vector<soth::VectorBound> &b,
                         unsigned int &NB_STAGE, std::vector<unsigned int> &NR,
                         unsigned int &NC) {
  std::ifstream fin(name.c_str());
  std::string syntax1, syntax2;
  MatrixXd Ji, eiinf, eisup;

  fin >> syntax1 >> syntax2 >> NC;
  assert((syntax1 == "variable") && (syntax2 == "size"));
  NB_STAGE = 0;
  unsigned int &s = NB_STAGE;
  fin >> syntax1;
  do {
    NR.resize(s + 1);
    J.resize(s + 1);
    b.resize(s + 1);

    unsigned int nre;
    fin >> syntax2 >> nre;
    assert((syntax1 == "level") && (syntax2 == "equalities"));

    MatrixXd Je;
    VectorXd ee;
    if (nre > 0) {
      Je.resize(nre, NC);
      ee.resize(nre);
      for (unsigned int i = 0; i < nre; ++i) {
        for (unsigned int j = 0; j < NC; ++j) fin >> Je(i, j);
        fin >> ee(i);
      }
    }

    fin >> syntax1 >> NR[s];
    assert((syntax1 == "inequalities"));

    /* Copy the equalities line into the total matrix. */
    NR[s] += nre;
    assert(NR[s] > 0);
    J[s].resize(NR[s], NC);
    b[s].resize(NR[s]);
    if (nre > 0) {
      J[s].topRows(nre) = Je;
      for (unsigned int i = 0; i < nre; ++i) b[s][i] = ee(i);
    }

    /* Parse the inequalities. */
    if (NR[s] > nre) {
      double bi, bu;
      for (unsigned int i = nre; i < NR[s]; ++i) {
        for (unsigned int j = 0; j < NC; ++j) fin >> J[s](i, j);
        fin >> bi >> bu;
        if (bi < -1e10)  // bound sup only
        {
          assert(bu <= 1e10);
          b[s][i] = Bound(bu, Bound::BOUND_SUP);
        } else if (1e10 < bu)  // bound inf only
        {
          b[s][i] = Bound(bi, Bound::BOUND_INF);
        } else  // double bound
        {
          b[s][i] = std::make_pair(bi, bu);
        }
      }
    }
    sotDEBUG(5) << "J" << s << " = " << (MATLAB)J[s] << endl;
    sotDEBUG(5) << "b" << s << " = " << b[s] << endl;
    fin >> syntax1;
    s++;
  } while (syntax1 == "level");
}

void readProblemFromBinFile(const std::string name,
                            std::vector<Eigen::MatrixXd> &J,
                            std::vector<soth::VectorBound> &b,
                            unsigned int &NB_STAGE,
                            std::vector<unsigned int> &NR, unsigned int &NC) {
  std::ifstream fin((name + ".txt").c_str());
  std::ifstream fbin((name + ".dat").c_str());
  std::string syntax1, syntax2;
  MatrixXd Ji, eiinf, eisup;

  fin >> syntax1 >> syntax2 >> NC;
  assert((syntax1 == "variable") && (syntax2 == "size"));
  NB_STAGE = 0;
  unsigned int &s = NB_STAGE;
  fin >> syntax1;
  do {
    NR.resize(s + 1);
    J.resize(s + 1);
    b.resize(s + 1);

    unsigned int nre;
    fin >> syntax2 >> nre;
    assert((syntax1 == "level") && (syntax2 == "equalities"));

    MatrixXd Je;
    VectorXd ee;
    if (nre > 0) {
      Je.resize(nre, NC);
      ee.resize(nre);
      for (unsigned int i = 0; i < nre; ++i) {
        for (unsigned int j = 0; j < NC; ++j) {
          fin >> Je(i, j);
          fbin.read(reinterpret_cast<char *>(&Je(i, j)), sizeof(double));
        }
        fin >> ee(i);
        fbin.read(reinterpret_cast<char *>(&ee(i)), sizeof(double));
      }
    }

    fin >> syntax1 >> NR[s];
    assert((syntax1 == "inequalities"));

    /* Copy the equalities line into the total matrix. */
    NR[s] += nre;
    assert(NR[s] > 0);
    J[s].resize(NR[s], NC);
    b[s].resize(NR[s]);
    if (nre > 0) {
      J[s].topRows(nre) = Je;
      for (unsigned int i = 0; i < nre; ++i) b[s][i] = ee(i);
    }

    /* Parse the inequalities. */
    if (NR[s] > nre) {
      double bi, bu;
      for (unsigned int i = nre; i < NR[s]; ++i) {
        for (unsigned int j = 0; j < NC; ++j) {
          fin >> J[s](i, j);
          fbin.read(reinterpret_cast<char *>(&J[s](i, j)), sizeof(double));
        }
        fin >> bi >> bu;
        fbin.read(reinterpret_cast<char *>(&bi), sizeof(double));
        fbin.read(reinterpret_cast<char *>(&bu), sizeof(double));
        if (bi < -1e10)  // bound sup only
        {
          assert(bu <= 1e10);
          b[s][i] = Bound(bu, Bound::BOUND_SUP);
        } else if (1e10 < bu)  // bound inf only
        {
          b[s][i] = Bound(bi, Bound::BOUND_INF);
        } else  // double bound
        {
          b[s][i] = std::make_pair(bi, bu);
        }
      }
    }
    sotDEBUG(5) << "J" << s << " = " << (MATLAB)J[s] << endl;
    sotDEBUG(5) << "b" << s << " = " << b[s] << endl;
    fin >> syntax1;
    s++;
  } while (syntax1 == "level");
}

void readProblemFromFile(const std::string name,
                         std::vector<Eigen::MatrixXd> &J,
                         std::vector<soth::VectorBound> &b) {
  unsigned int NB_STAGE;
  std::vector<unsigned int> NR;
  unsigned int NC;
  readProblemFromFile(name, J, b, NB_STAGE, NR, NC);
}

void writeProblemToFile(const std::string name,
                        const std::vector<Eigen::MatrixXd> &J,
                        const std::vector<soth::VectorBound> &b,
                        const unsigned int &NB_STAGE,
                        const std::vector<unsigned int> &NR,
                        const unsigned int &NC)

{
  std::ofstream fout(name.c_str());
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
          assert(b[s][r].getType() != Bound::BOUND_NONE);
      }
    }
    fout << endl;
  }

  fout << endl << "end" << endl << endl;
}

void writeProblemToFile(const std::string name,
                        const std::vector<Eigen::MatrixXd> &J,
                        const std::vector<soth::VectorBound> &b) {
  unsigned int NB_STAGE;
  std::vector<unsigned int> NR;
  unsigned int NC;
  writeProblemToFile(name, J, b, NB_STAGE, NR, NC);
}

/* Input: size = number of cols, rowPercent = row number / col number,
 * rankPercent = rank / col number,
 * selfDeficiencyPercent = rankdef [A1 .. Ap] / sum rankdef(Ai)
 */
void generateFixedSizeRandomProfile(
    const unsigned int size, const double rowPercent, const double rankPercent,
    const double selfDeficiencyPercent, unsigned int &nbStage,
    std::vector<unsigned int> &rankfree, std::vector<unsigned int> &ranklinked,
    std::vector<unsigned int> &nr, unsigned int &nc) {
  nc = size;
  nbStage = randu(1, 1 + nc / 4);
  rankfree.resize(nbStage);
  ranklinked.resize(nbStage);
  nr.resize(nbStage);

  sotDEBUG(5) << "nc = " << nc << endl;
  sotDEBUG(5) << "nbStage = " << nbStage << endl;

  const int TOTALRANKS = int(round(size * rankPercent));
  const int TOTALROWS = int(round(size * rowPercent));
  const int TOTALSELFDEF =
      int(round((nc - TOTALRANKS) * selfDeficiencyPercent));

  VectorXd rowDistrib(nbStage);
  soth::MatrixRnd::randomize(rowDistrib, 0, 1);
  rowDistrib /= rowDistrib.sum();
  sotDEBUG(25) << rowDistrib.sum() << " " << (MATLAB)rowDistrib << std::endl;
  rowDistrib *= TOTALROWS - nbStage;
  rowDistrib.array() += 1;

  VectorXd rankDistrib(nbStage);
  soth::MatrixRnd::randomize(rankDistrib, 0, 1);
  double vrk =
      (rankDistrib.cwiseProduct((rowDistrib.array() - 1).matrix())).sum();
  sotDEBUG(20) << "vrk = " << vrk << " / " << TOTALRANKS << std::endl;
  if (vrk > TOTALRANKS - nbStage) {
    rankDistrib *= (TOTALRANKS - nbStage) / vrk;
  } else {
    VectorXd defDistrib = 1 - rankDistrib.array();
    defDistrib *= (TOTALROWS - TOTALRANKS) / (TOTALROWS - nbStage - vrk);
    rankDistrib = 1 - defDistrib.array();
  }
  sotDEBUG(15) << "percent = " << (MATLAB)rankDistrib << std::endl;
  rankDistrib = rankDistrib.cwiseProduct((rowDistrib.array() - 1).matrix());
  rankDistrib.array() += 1;
  sotDEBUG(25) << rankDistrib.sum() << " " << (MATLAB)rankDistrib << std::endl;

  sotDEBUG(15) << "Mi: " << rowDistrib.sum() << " " << (MATLAB)rowDistrib
               << std::endl;
  sotDEBUG(15) << "Ri: " << rankDistrib.sum() << " " << (MATLAB)rankDistrib
               << std::endl;
  rankDistrib = rankDistrib.unaryExpr(std::ptr_fun<double, double>(round));
  rowDistrib = rowDistrib.unaryExpr(std::ptr_fun<double, double>(round));
  VectorXd selfdefDistrib(nbStage);
  soth::MatrixRnd::randomize(selfdefDistrib, 0, 1);
  double vsd = selfdefDistrib.cwiseProduct(rowDistrib - rankDistrib).sum();
  sotDEBUG(20) << "vsd = " << vsd << " / " << TOTALSELFDEF << std::endl;
  if (vsd > TOTALSELFDEF) {
    selfdefDistrib *= TOTALSELFDEF / vsd;
  } else {
    VectorXd algdefDistrib = 1 - selfdefDistrib.array();
    algdefDistrib *= (TOTALROWS - TOTALRANKS - TOTALSELFDEF) /
                     (TOTALROWS - TOTALRANKS - vsd);
    selfdefDistrib = 1 - algdefDistrib.array();
  }
  sotDEBUG(15) << "percent = " << (MATLAB)selfdefDistrib << std::endl;
  selfdefDistrib = selfdefDistrib.cwiseProduct(rowDistrib - rankDistrib);
  sotDEBUG(25) << selfdefDistrib.sum() << " " << (MATLAB)selfdefDistrib
               << std::endl;

  sotDEBUG(15) << "Si: " << selfdefDistrib.sum() << " "
               << (MATLAB)selfdefDistrib << std::endl;
  selfdefDistrib =
      selfdefDistrib.unaryExpr(std::ptr_fun<double, double>(round));
  sotDEBUG(5) << "Mi: " << rowDistrib.sum() << " " << (MATLAB)rowDistrib
              << std::endl;
  sotDEBUG(5) << "Ri: " << rankDistrib.sum() << " " << (MATLAB)rankDistrib
              << std::endl;
  sotDEBUG(5) << "Si: " << selfdefDistrib.sum() << " " << (MATLAB)selfdefDistrib
              << std::endl;

  for (unsigned int i = 0; i < nbStage; ++i) {
    nr[i] = int(rowDistrib[i]);
    rankfree[i] = int(rankDistrib[i]);
    ranklinked[i] = int(rowDistrib[i] - rankDistrib[i] - selfdefDistrib[i]);
    sotDEBUG(10) << i << ":\t" << nr[i] << "\t" << rankfree[i] << "\t"
                 << "+" << ranklinked[i] << "=" << ranklinked[i] + rankfree[i]
                 << std::endl;
  }
}
}  // namespace soth

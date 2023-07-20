/* -------------------------------------------------------------------------- *
 *
 * Complete test of the HCOD solver: giving the command-line arguments, build a
 * problem, solve it and compare the result to a SVD resolution with an
 * exhaustive exploration of the active sets.
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

#include <iostream>
#include "gettimeofday.hpp"

using namespace soth;
using std::cerr;
using std::cout;
using std::endl;

typedef MatrixXd::Index Index;
/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */

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
/* --- SOTH SOLVER ---------------------------------------------------------- */
/* -------------------------------------------------------------------------- */

/* This piece of code tries all the possible active set, looking for the best
 * solution, and compare this solution with the current solution. The function
 * to be run is explore, with the constraints J and b as arguments, as well
 * as the optimum to be checked. */
namespace DummyActiveSet {
VectorXd SOT_Solver(std::vector<Eigen::MatrixXd> J, std::vector<VectorXd> /*e*/,
                    double /*damping*/ = 0.0) {
  const unsigned int NC = (unsigned int)J[0].cols();
  VectorXd u = VectorXd::Zero(NC);

  /*    MatrixXd Pa = MatrixXd::Identity(NC,NC);
   *    const double EPSILON = Stage::EPSILON;
   *    MatrixXd Inc = damping*MatrixXd::Identity(NC,NC);
   *
   *    for( unsigned int i=0;i<J.size();++i )
   *      {
   *	if( J[i].rows() == 0 ) continue;
   *	MatrixXd JPi = J[i]*Pa;
   *	ULV ulv; ulv.compute(JPi,EPSILON);
   *
   *	if( damping>0 )
   *	  {
   *	    MatrixXd JPidamped
   *	      = soth::StackMatrix<MatrixXd,MatrixXd>( J[i],Inc )*Pa;
   *	    sotDEBUG(5) << "JPd"<<i<<" = " << (MATLAB)JPidamped << endl;
   *	    ULV ulvd; ulvd.compute(JPidamped,EPSILON);
   *
   *	    const unsigned int nr = e[i].size();
   *	    VectorXd ed(nr+NC);
   *	    ed.head(nr) = e[i]-J[i]*u;
   *	    ed.tail(NC) = -damping*u;
   *	    sotDEBUG(5) << "et"<<i<<" = " << (MATLAB)ed << endl;
   *
   *	    u += ulvd.solve( ed );
   *	    sotDEBUG(5) << "du"<<i<<" = " << (MATLAB)ulvd.solve( ed ) << endl;
   *	  }
   *	else
   *	  {
   *	    sotDEBUG(5) << "et"<<i<<" = " << (MATLAB)(e[i]-J[i]*u) << endl;
   *	    u += ulv.solve( e[i]-J[i]*u );
   *	  }
   *
   *	ulv.decreaseProjector( Pa );
   *	sotDEBUG(5) << "P"<<i<<" = " << (MATLAB)Pa << endl;
   *	sotDEBUG(5) << "u"<<i<<" = " << (MATLAB)u << endl;
   *      }
   */
  return u;
}

std::vector<double> stageErrors(const std::vector<Eigen::MatrixXd>& Jref,
                                const std::vector<soth::VectorBound>& bref,
                                const VectorXd& usot) {
  std::vector<double> errors(Jref.size());
  for (unsigned int i = 0; i < Jref.size(); ++i) {
    const MatrixXd& J = Jref[i];
    const soth::VectorBound& b = bref[i];
    VectorXd e;
    e = J * usot;
    errors[i] = 0;
    for (unsigned int r = 0; int(r) < J.rows(); ++r) {
      double x = b[r].distance(e[r]);
      errors[i] += x * x;
    }
    sotDEBUG(5) << "err(" << i << ") = " << errors[i] << endl;
  }
  return errors;
}

/* Alphalexical strict order
 * In case of equality, return false. The test is done with EPS approximation
 * (distance of each lexem should be more than EPS).
 */
bool isLess(const std::vector<double>& m1, const std::vector<double>& m2,
            const double& EPS) {
  assert(m1.size() == m2.size());
  for (unsigned int i = 0; i < m1.size(); ++i) {
    if (std::abs(m1[i] - m2[i]) <= EPS) continue;
    if (m1[i] < m2[i])
      return true;
    else
      return false;
  }
  return false;
}

std::vector<double> operator+(const std::vector<double>& m1, const double& b) {
  std::vector<double> res;
  res.resize(m1.size());
  for (unsigned int i = 0; i < m1.size(); ++i) res[i] = m1[i] + b;
  return res;
}
std::vector<double> operator-(const std::vector<double>& m1, const double& b) {
  std::vector<double> res;
  res.resize(m1.size());
  for (unsigned int i = 0; i < m1.size(); ++i) res[i] = m1[i] - b;
  return res;
}

/* Use the old SOT solver for the given active set. The optimum
 * is then compared to the bound. If the found solution is valid, compare its
 * norm to the reference optimum. Return true is the solution found by SOT is
 * valid and better. */
bool compareSolvers(const std::vector<Eigen::MatrixXd>& Jref,
                    const std::vector<soth::VectorBound>& bref,
                    const std::vector<std::vector<Index> >& active,
                    const std::vector<std::vector<Bound::bound_t> >& bounds,
                    const double& urefnorm, const std::vector<double>& erefnorm,
                    const bool verbose = false, const double damping = 0.0) {
  /* Build the SOT problem. */
  const unsigned int NC = (unsigned int)Jref[0].cols();
  std::vector<Eigen::MatrixXd> Jsot(Jref.size());
  std::vector<VectorXd> esot(Jref.size());
  for (unsigned int i = 0; i < Jref.size(); ++i) {
    MatrixXd& J = Jsot[i];
    VectorXd& e = esot[i];
    const std::vector<Index>& Aset = active[i];
    const std::vector<Bound::bound_t>& Bset = bounds[i];
    const unsigned int NR = (unsigned int)Aset.size();

    J.resize(NR, NC);
    e.resize(NR);
    for (unsigned int r = 0; r < NR; ++r) {
      sotDEBUG(5) << "Get " << i << "," << Aset[r] << " (" << r << ")" << endl;
      J.row(r) = Jref[i].row(Aset[r]);
      e(r) = bref[i][Aset[r]].getBound(Bset[Aset[r]]);
    }

    sotDEBUG(45) << "J" << i << " = " << (MATLAB)J << endl;
    sotDEBUG(45) << "e" << i << " = " << (MATLAB)e << endl;
  }

  /* Find the optimum. */
  /* --- WARNING: the sot solver is broken and wait for a fix with the
   * --- COD class.*/
  VectorXd usot = SOT_Solver(Jsot, esot, damping);
  sotDEBUG(45) << "usot"
               << " = " << (MATLAB)usot << endl;

  /* Check the bounds. */
  std::vector<double> esotnorm = stageErrors(Jref, bref, usot);
  if (verbose || (isLess(esotnorm, erefnorm, Stage::EPSILON)))
  //	   &&( usot.norm()+Stage::EPSILON < urefnorm) ) ) //DEBUG
  {
    std::cout << endl
              << endl
              << " * * * * * * * * * * * * * " << endl
              << "usot"
              << " = " << (MATLAB)usot << endl;
    std::cout << "aset = {";
    for (unsigned int i = 0; i < Jref.size(); ++i) {
      std::cout << "\n        [ ";
      for (unsigned int r = 0; r < active[i].size(); ++r) {
        const Index ref = active[i][r];
        if (bounds[i][ref] == Bound::BOUND_INF)
          cout << "-";
        else if (bounds[i][ref] == Bound::BOUND_SUP)
          cout << "+";
        std::cout << active[i][r] << " ";
      }
      std::cout << " ]";
    }
    std::cout << "} " << endl;
    std::cout << "norm solution = " << usot.norm() << " ~?~ " << urefnorm
              << " = norm ref." << endl;
    std::cout << "e = [";
    for (unsigned int s = 0; s < esotnorm.size(); ++s) {
      std::cout << (esotnorm + Stage::EPSILON)[s] << "  (" << erefnorm[s]
                << ")     ";
    }
    std::cout << " ];" << endl;

    if (isLess(esotnorm, erefnorm, Stage::EPSILON)) {
      cout << "The solution is better." << endl;
    }
    return true;
  } else
    return false;
}

void intToVbool(const unsigned int nbConstraint, const unsigned long int ref,
                std::vector<bool>& res) {
  res.resize(nbConstraint);
  sotDEBUG(45) << "ref"
               << " = " << ref << endl;
  for (unsigned int i = 0; i < nbConstraint; ++i) {
    sotDEBUG(45) << i << ": " << (0x01 & (ref >> i)) << endl;
    res[i] = (0x01 & (ref >> i));
  }
}

void selectConstraint(const std::vector<soth::VectorBound>& bref,
                      const std::vector<bool>& activeBool,
                      std::vector<std::vector<Index> >& aset) {
  aset.resize(bref.size());
  int cst = 0;
  for (unsigned int i = 0; i < bref.size(); ++i) {
    for (int r = 0; r < bref[i].size(); ++r)
      if ((bref[i][r].getType() == Bound::BOUND_TWIN) || (activeBool[cst++]))
        aset[i].push_back(r);

    // std::cout << "a" << i << " = [ ";
    // for( unsigned int r=0;r<aset[i].size();++r )
    // 	std::cout << aset[i][r] << " ";
    // std::cout << "];" << endl;
  }
}

void selectBool(const std::vector<soth::VectorBound>& bref,
                std::vector<std::vector<Index> > aset,
                const std::vector<bool>& boundBool,
                std::vector<std::vector<Bound::bound_t> >& boundSelec) {
  int boolPrec = 0;
  boundSelec.resize(bref.size());
  for (unsigned int i = 0; i < bref.size(); ++i) {
    boundSelec[i].resize(bref[i].size(), Bound::BOUND_NONE);
    for (unsigned int r = 0; r < aset[i].size(); ++r) {
      if (bref[i][aset[i][r]].getType() == Bound::BOUND_DOUBLE) {
        sotDEBUG(5) << "Fill " << i << "," << aset[i][r] << " (" << r << ")"
                    << endl;
        boundSelec[i][aset[i][r]] =
            (boundBool[boolPrec++]) ? Bound::BOUND_INF : Bound::BOUND_SUP;
      } else {
        boundSelec[i][aset[i][r]] = bref[i][aset[i][r]].getType();
      }
      assert((boundSelec[i][aset[i][r]] != Bound::BOUND_DOUBLE) &&
             (boundSelec[i][aset[i][r]] != Bound::BOUND_NONE));
    }
  }
}

int computeNbDouble(const std::vector<soth::VectorBound>& bref,
                    std::vector<std::vector<Index> > aset) {
  int nbDoubleConstraint = 0;
  for (unsigned int i = 0; i < bref.size(); ++i) {
    for (unsigned int r = 0; r < aset[i].size(); ++r) {
      if (bref[i][aset[i][r]].getType() == Bound::BOUND_DOUBLE)
        nbDoubleConstraint++;
    }
  }
  return nbDoubleConstraint;
}

int computeNbConst(const std::vector<soth::VectorBound>& bref,
                   std::vector<int>& nr) {
  int nbConstraint = 0;
  nr.resize(bref.size());
  for (unsigned int i = 0; i < bref.size(); ++i) {
    nr[i] = (int)bref[i].size();
    for (int r = 0; r < bref[i].size(); ++r)
      if (bref[i][r].getType() != Bound::BOUND_TWIN) nbConstraint++;
  }
  return nbConstraint;
}

/* Explore all the possible active set.
 */
bool explore(const std::vector<Eigen::MatrixXd>& Jref,
             const std::vector<soth::VectorBound>& bref, const VectorXd& uref) {
  std::vector<int> nr(Jref.size());
  int nbConstraint = computeNbConst(bref, nr);
  const double urefnorm = uref.norm();
  const std::vector<double> erefnorm = stageErrors(Jref, bref, uref);

  bool res = true;
  std::cout << "Nb posibilities = " << (1 << (nbConstraint)) << endl;
  for (long int refc = 0; refc < (1 << (nbConstraint)); ++refc) {
    if (!(refc % 1000)) std::cout << refc << " ... " << endl;
    std::vector<bool> abool;
    std::vector<std::vector<Index> > aset;
    intToVbool(nbConstraint, refc, abool);
    selectConstraint(bref, abool, aset);

    int nbDoubleConstraint = computeNbDouble(bref, aset);
    sotDEBUG(15) << "Nb double = " << nbDoubleConstraint << endl;
    for (long int refb = 0; refb < (1 << (nbDoubleConstraint)); ++refb) {
      std::vector<bool> bbool;
      std::vector<std::vector<Bound::bound_t> > bset;

      intToVbool(nbDoubleConstraint, refb, bbool);
      selectBool(bref, aset, bbool, bset);

      if (compareSolvers(Jref, bref, aset, bset, urefnorm, erefnorm)) {
        std::cout << refc << "/" << refb
                  << "  ...  One more-optimal solution!! " << endl;
        res = false;
      }
    }
  }
  return res;
}

void detailActiveSet(const std::vector<Eigen::MatrixXd>& Jref,
                     const std::vector<soth::VectorBound>& bref,
                     const VectorXd& uref, unsigned long int refc,
                     unsigned long int refb) {
  std::vector<int> nr(Jref.size());
  int nbConstraint = computeNbConst(bref, nr);
  const double urefnorm = uref.norm();
  const std::vector<double> erefnorm = stageErrors(Jref, bref, uref);

  std::vector<bool> abool;
  std::vector<std::vector<Index> > aset;
  intToVbool(nbConstraint, refc, abool);
  selectConstraint(bref, abool, aset);
  int nbDoubleConstraint = computeNbDouble(bref, aset);
  std::vector<bool> bbool;
  std::vector<std::vector<Bound::bound_t> > bset;
  intToVbool(nbDoubleConstraint, refb, bbool);
  selectBool(bref, aset, bbool, bset);

  compareSolvers(Jref, bref, aset, bset, urefnorm, erefnorm, true);
}
}  // namespace DummyActiveSet

/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */

#include <boost/program_options.hpp>

int main(int argc, char** argv) {
#ifndef NDEBUG
  sotDebugTrace::openFile();
#endif

  unsigned int NB_STAGE, NC;
  std::vector<unsigned int> NR, RANKLINKED, RANKFREE;
  std::vector<Eigen::MatrixXd> J;
  std::vector<soth::VectorBound> b;

  boost::program_options::variables_map optionMap;
  {
    namespace po = boost::program_options;
    po::options_description desc("trandom options");
    desc.add_options()("help", "produce help message")(
        "seed,s", po::value<int>(), "specify the seed of the random generator")(
        "file,f", po::value<std::string>(), "ascii file")(
        "bin,b", po::value<std::string>(),
        "file root (no extention to reach both ascii file for specification "
        "and bin file for values");

    po::positional_options_description p;
    p.add("seed", -1);

    po::store(
        po::command_line_parser(argc, argv).options(desc).positional(p).run(),
        optionMap);
    po::notify(optionMap);

    if (optionMap.count("help")) {
      std::cout << desc << std::endl;
      exit(0);
    }

    int nbExclusive = 0;
    if (optionMap.count("bin")) nbExclusive++;
    if (optionMap.count("file")) nbExclusive++;
    if (optionMap.count("seed")) nbExclusive++;
    if (nbExclusive > 1) {
      std::cout << "Option bin, file and seed are exclusive." << std::endl;
      exit(-1);
    }
  }

  if (optionMap.count("file")) {
    std::cout << "Read from file ... " << std::endl;
    readProblemFromFile(optionMap["file"].as<std::string>(), J, b, NB_STAGE, NR,
                        NC);
  } else if (optionMap.count("bin")) {
    std::cout << "Read from bin file ... " << std::endl;
    readProblemFromFile(optionMap["bin"].as<std::string>(), J, b, NB_STAGE, NR,
                        NC);
  } else {
    /* Initialize the seed. */
    struct timeval tv;
    gettimeofday(&tv, NULL);
    int seed = int(tv.tv_usec % 7919);  //= 7594;
    if (optionMap.count("seed")) {
      seed = optionMap["seed"].as<int>();
    }
    std::cout << "seed = " << seed << std::endl;
    soth::Random::setSeed(seed);

    /* Decide the size of the problem. */
    generateRandomProfile(NB_STAGE, RANKFREE, RANKLINKED, NR, NC);
    /* Initialize J and b. */
    generateDeficientDataSet(J, b, NB_STAGE, RANKFREE, RANKLINKED, NR, NC);
  }

  std::cout << "NB_STAGE=" << NB_STAGE << ",  NC=" << NC << endl;
  cout << endl;
  for (unsigned int i = 0; i < NB_STAGE; ++i) {
    std::cout << "J" << i + 1 << " = " << (soth::MATLAB)J[i] << std::endl;
    std::cout << "e" << i + 1 << " = " << b[i] << ";" << std::endl;
  }
  //  assert( std::abs(J[0](0,0)-(-1.1149))<1e-5 );

  /* SOTH structure construction. */
  soth::HCOD hcod(NC, NB_STAGE);
  for (unsigned int i = 0; i < NB_STAGE; ++i) {
    hcod.pushBackStage(J[i], b[i]);
  }
  hcod.setNameByOrder("stage_");

  const double dampingFactor = 0.0;
  hcod.setDamping(dampingFactor);
  hcod.stage(0).damping(0);
  hcod.setInitialActiveSet();

  // DEBUG
  //   a{1} =  [ -5 -6 +12 +15 -16 +19 +21 +24 +25 -10 -8 +1 -20 -2 +26 -4 -18
  //   +28 -22 +27 +29 -0 +3 +7 a{1} =  [ -5 -6 +12 +15 -16 +19 +21 +24 +25 -10
  //   -8 +1 -20 -2 +26 -4 -18 +28 -22 +27 +29 -0 +3 +7

  //   cstref_vector_t Ir0;
  //   Ir0.push_back( ConstraintRef(5,Bound::BOUND_INF) );
  //   Ir0.push_back( ConstraintRef(6,Bound::BOUND_INF) );
  //   Ir0.push_back( ConstraintRef(12,Bound::BOUND_SUP) );
  //   Ir0.push_back( ConstraintRef(15,Bound::BOUND_SUP) );
  //   Ir0.push_back( ConstraintRef(16,Bound::BOUND_INF) );
  //   Ir0.push_back( ConstraintRef(19,Bound::BOUND_SUP) );
  //   Ir0.push_back( ConstraintRef(21,Bound::BOUND_SUP) );
  //   Ir0.push_back( ConstraintRef(24,Bound::BOUND_SUP) );
  //   Ir0.push_back( ConstraintRef(25,Bound::BOUND_SUP) );
  //   Ir0.push_back( ConstraintRef(10,Bound::BOUND_INF) );
  //   Ir0.push_back( ConstraintRef(8,Bound::BOUND_INF) );
  //   Ir0.push_back( ConstraintRef(1,Bound::BOUND_SUP) );
  //   Ir0.push_back( ConstraintRef(20,Bound::BOUND_INF) );
  //   Ir0.push_back( ConstraintRef(2,Bound::BOUND_INF) );
  //   Ir0.push_back( ConstraintRef(26,Bound::BOUND_SUP) );
  //   Ir0.push_back( ConstraintRef(4,Bound::BOUND_INF) );
  //   Ir0.push_back( ConstraintRef(18,Bound::BOUND_INF) );
  //   Ir0.push_back( ConstraintRef(28,Bound::BOUND_SUP) );
  //   Ir0.push_back( ConstraintRef(22,Bound::BOUND_INF) );
  //   Ir0.push_back( ConstraintRef(27,Bound::BOUND_SUP) );
  //   Ir0.push_back( ConstraintRef(29,Bound::BOUND_SUP) );
  //   Ir0.push_back( ConstraintRef(0,Bound::BOUND_INF) );
  //   Ir0.push_back( ConstraintRef(3,Bound::BOUND_SUP) );
  //   Ir0.push_back( ConstraintRef(7,Bound::BOUND_SUP) );

  // // Update stage_0, -14
  // // Update stage_0, +9
  // // Update stage_0, -11
  // // Update stage_0, +13
  // // Update stage_0, -17
  // // Update stage_0, +23

  //   b[0][14] = std::make_pair( -1e25,1e25 );
  //   b[0][9] = std::make_pair( -1e25,1e25 );
  //   b[0][11] = std::make_pair( -1e25,1e25 );
  //   b[0][13] = std::make_pair( -1e25,1e25 );
  //   b[0][17] = std::make_pair( -1e25,1e25 );
  //   b[0][23] = std::make_pair( -1e25,1e25 );

  //  hcod[0].setInitialActiveSet(Ir0,false);

  VectorXd solution;
  hcod.activeSearch(solution);
  if (sotDEBUGFLOW.outputbuffer.good()) hcod.show(sotDEBUGFLOW.outputbuffer);
  cout << "Optimal solution = " << (MATLAB)solution << endl;
  cout << "Optimal active set = ";
  hcod.showActiveSet(std::cout);

  if (0) {
    cout << " --- RE --------------------------------- " << endl;
    sotDEBUG(1) << " --- RE --------------------------------- " << endl;
    hcod.activeSearch(solution);
    cout << "Optimal solution = " << (MATLAB)solution << endl;
    cout << "Optimal active set = ";
    hcod.showActiveSet(std::cout);
  }

  cout << " --- CHECK ------------------------------ " << endl;
  sotDEBUG(1) << " --- CHECK ------------------------------ " << endl;

  std::vector<std::vector<Index> > active(NB_STAGE);
  std::vector<std::vector<Bound::bound_t> > bounds(NB_STAGE);
  for (unsigned int i = 0; i < NB_STAGE; ++i) {
    active[i].resize(hcod[i].sizeA());
    bounds[i].resize(b[i].size(), soth::Bound::BOUND_TWIN);
    for (unsigned int r = 0; r < hcod[i].sizeA(); ++r) {
      soth::ConstraintRef cst = hcod[i].which(r);
      assert(cst.row >= 0 && cst.row < b[i].size());
      assert(cst.type == soth::Bound::BOUND_TWIN ||
             cst.type == soth::Bound::BOUND_INF ||
             cst.type == soth::Bound::BOUND_SUP);
      active[i][r] = cst.row;
      bounds[i][cst.row] = cst.type;
    }
  }
  DummyActiveSet::compareSolvers(J, b, active, bounds, solution.norm(),
                                 DummyActiveSet::stageErrors(J, b, solution),
                                 true, dampingFactor);

  cout << " --- EXPLORE ---------------------------- " << endl;
  sotDEBUG(1) << " --- EXPLORE ---------------------------- " << endl;

#ifdef NDEBUG
  if (!DummyActiveSet::explore(J, b, solution)) {
    exit(-1);
  }
#else
  //  DummyActiveSet::detailActiveSet( J,b,solution,3045,3);
#endif
}

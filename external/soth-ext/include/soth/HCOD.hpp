#ifndef __SOTH_HCOD__
#define __SOTH_HCOD__

#include <boost/smart_ptr.hpp>
#include "BaseY.hpp"
#include "Stage.hpp"
#include "api.hpp"
#include "visibility.h"

namespace soth {
class SOTH_API HCOD {
 protected:
  typedef boost::shared_ptr<soth::Stage> stage_ptr_t;
  typedef std::vector<stage_ptr_t> stage_sequence_t;
  typedef stage_sequence_t::iterator stage_iter_t;
  typedef stage_sequence_t::const_iterator stage_citer_t;
  typedef stage_sequence_t::reverse_iterator stage_riter_t;
  typedef stage_sequence_t::size_type stage_sequence_size_t;
  typedef MatrixXd::Index Index;

 public:
  HCOD(Index sizeProblem, Index nbStage = 0);

  void pushBackStage(const MatrixXd& J, const VectorBound& bounds);
  void pushBackStage(const Index& nr, const double* Jdata, const Bound* bdata);
  void pushBackStage(const Index& nr, const double* Jdata);
  void pushBackStages(const std::vector<MatrixXd>& J,
                      const std::vector<VectorBound>& bounds);

  Stage& stage(Index i);
  const Stage& stage(Index i) const;
  inline Stage& operator[](Index i) { return stage(i); }
  inline const Stage& operator[](Index i) const { return stage(i); }

  void setInitialActiveSet();
  void setInitialActiveSet(const cstref_vector_t& Ir0, Index k);
  cstref_vector_t getOptimalActiveSet(Index k);
  std::vector<cstref_vector_t> getOptimalActiveSet();
  void setInitialActiveSet(const std::vector<cstref_vector_t>& Ir);

  void useDamp(bool c) { withDamp = c; }
  bool useDamp() const { return withDamp; }
  void setDamping(const double& d);
  double getMaxDamping() const;

  // sizes
  Index sizeA() const;
  int rank() const;
  Index nbStages() const { return (Index)stages.size(); }

  /* --- Decomposition --- */
 public:
  void reset(void);
  void initialize(void);
  void update(const Index& stageUp, const ConstraintRef& cst);
  void update(stage_iter_t stageIter, const ConstraintRef& cst);
  void downdate(const Index& stageDown, const Index& row);
  void downdate(stage_iter_t stageIter, const Index& row);

 protected:
  void updateY(const GivensSequence& Yup);

  /* --- Computations --- */
 public:
  void damp(void);
  void computeSolution(bool compute_u = true);
  void computeLagrangeMultipliers(const Index& stageRef);
  double computeStepAndUpdate(void);
  double computeStep(void);
  bool searchAndDowndate(const Index& stageRef);
  bool search(const Index& stageRef);

  void makeStep(double tau, bool compute_u = true);
  void makeStep(bool compute_u = true);

  /* --- The big one --- */
 public:
  // template< typename VectorGen >
  void activeSearch(VectorXd& u);

  /* --- Tests --- */
 public:
  void show(std::ostream& os, bool check = false);
  void showActiveSet(std::ostream& os) const;
  bool testRecomposition(std::ostream* os = NULL);
  bool testSolution(std::ostream* os = NULL);
  bool testLagrangeMultipliers(Index stageRef, std::ostream* os = NULL) const;
  bool testLagrangeMultipliers(Index stageRef, std::ostream& os) const {
    return testLagrangeMultipliers(stageRef, &os);
  }

  void setNameByOrder(const std::string root = "");
  void notifiorRegistration(const Stage::listener_function_t& f,
                            int stageRank = -1);

  bool isDebugOnce;
  void debugOnce(std::string filename = "", bool keepOpen = false);

 protected:
  HCOD(void) : Y(0){};

 protected:
 public:  // DEBUG
  Index sizeProblem;
  soth::BaseY Y;
  stage_sequence_t stages;
  VectorXd solution;

  VectorXd uNext, Ytu, YtuNext, rho;
  int freezedStages;
  bool isReset, isInit, isSolutionCpt, withDamp;
};

}  // namespace soth

#endif  // #ifndef __SOTH_HCOD__

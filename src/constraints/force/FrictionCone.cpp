#include <OpenSoT/constraints/force/FrictionCone.h>

namespace OpenSoT {
   namespace constraints {
       namespace force {

       FrictionCone::FrictionCone(const std::string& contact_name,
                                  const AffineHelper& wrench,
                                  XBot::ModelInterface &robot,
                                  const friction_cone& mu):
           Constraint(contact_name + "_friction_cone", wrench.getInputSize()),
           _robot(robot),
           _mu(mu),
           _Ci(5,3),
           _wrench(wrench)
       {
           _A.resize(5, wrench.getOutputSize());
           _A.setZero(_A.rows(), _A.cols());

           _b.resize(5);
           _b.setZero(_b.rows());


           _bLowerBound.resize(5);
           _bLowerBound = -1.0e20*_bLowerBound.setOnes(_bLowerBound.size());

           computeAineq();

           _friction_cone = _A * _wrench - _b;
           _Aineq = _friction_cone.getM();
           _bUpperBound = - _friction_cone.getq();

       }

       void FrictionCone::computeAineq()
       {
           _Ci.setZero(5,3);
           _A.setZero(_A.rows(), _A.cols());

           double __mu = _mu.second;
           _wRl = _mu.first;

           __mu = __mu/std::sqrt(2.);

           _Ci(0,0) = 1.;  _Ci(0,1) = 0.;  _Ci(0,2) = -__mu;
           _Ci(1,0) = -1.; _Ci(1,1) = 0.;  _Ci(1,2) = -__mu;
           _Ci(2,0) = 0.;  _Ci(2,1) = 1.;  _Ci(2,2) = -__mu;
           _Ci(3,0) = 0.;  _Ci(3,1) = -1.; _Ci(3,2) = -__mu;
           _Ci(4,0) = 0.;  _Ci(4,1) = 0.;  _Ci(4,2) = -1.;

           _Ci = _Ci*_wRl.transpose();

           _A.block<5,3>(0, 0) = _Ci;
       }



       void FrictionCone::update(const Eigen::VectorXd &x)
       {

       }

       void FrictionCone::setContactRotationMatrix(const Eigen::Matrix3d& wRl)
       {
           _mu.first = wRl;

           computeAineq();

           _friction_cone = _A * _wrench - _b;
           _Aineq = _friction_cone.getM();
           _bUpperBound = - _friction_cone.getq();

       }

       void FrictionCone::setMu(const double mu)
       {
           _mu.second = mu;

           computeAineq();

           _friction_cone = _A * _wrench - _b;
           _Aineq = _friction_cone.getM();
           _bUpperBound = - _friction_cone.getq();
       }

       void FrictionCone::setFrictionCone(const friction_cone& frc)
       {
           _mu = frc;

           computeAineq();

           _friction_cone = _A * _wrench - _b;
           _Aineq = _friction_cone.getM();
           _bUpperBound = - _friction_cone.getq();
       }

       FrictionCones::FrictionCones(const std::vector<std::string>& contact_name,
                    const std::vector<AffineHelper>& wrench,
                    XBot::ModelInterface &robot,
                    const friction_cones & mu):
           Constraint("friction_cones", wrench[0].getInputSize())
       {
           std::list<ConstraintPtr> constraint_list;
           for(unsigned int i = 0; i < contact_name.size(); ++i){
               _friction_cone_map[contact_name[i]] = boost::make_shared<FrictionCone>
                       (contact_name[i], wrench[i], robot, mu[i]);
               constraint_list.push_back(_friction_cone_map[contact_name[i]]);
           }

           _internal_constraint = boost::make_shared<OpenSoT::constraints::Aggregated>
                   (constraint_list, wrench[0].getInputSize());

           update(Eigen::VectorXd(0));
       }

       FrictionCone::Ptr FrictionCones::getFrictionCone(const std::string& contact_name)
       {
           if(_friction_cone_map.count(contact_name))
               return _friction_cone_map[contact_name];
           else
               return NULL;
       }

       void FrictionCones::update(const Eigen::VectorXd &x)
       {
           _internal_constraint->update(x);
           generateBounds();
       }

       void FrictionCones::generateBounds()
       {
           _Aineq = _internal_constraint->getAineq();
           _bUpperBound = _internal_constraint->getbUpperBound();
           _bLowerBound = _internal_constraint->getbLowerBound();
       }

       }
   }
}

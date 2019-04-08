#include <OpenSoT/constraints/force/FrictionCone.h>

namespace OpenSoT {
   namespace constraints {
       namespace force {

       FrictionCone::FrictionCone(const Eigen::VectorXd &x,
                                  XBot::ModelInterface &robot,
                                  const friction_cones& mu):
           Constraint("friction_cone", x.rows()),
           _robot(robot),
           _mu(mu),
           _Ci(5,3)
       {
           _n_of_contacts = _mu.size();



           OptvarHelper::VariableVector vars;
           for(unsigned int i = 0; i < _n_of_contacts; ++i)
            vars.emplace_back("wrench_"+i, 6);


           OptvarHelper opthelper(vars);
           _wrenches.setZero(x.size(), 0);

           for(auto& w : opthelper.getAllVariables()){
               _wrenches = _wrenches / w;
           }

           update(Eigen::VectorXd::Zero(0));

       }


       FrictionCone::FrictionCone(const std::vector<AffineHelper>& wrenches,
                                  XBot::ModelInterface &robot,
                                  const friction_cones& mu):
           Constraint("friction_cone", wrenches[0].getInputSize()),
           _robot(robot),
           _mu(mu),
           _Ci(5,3)
       {
           _n_of_contacts = _mu.size();



           _wrenches.setZero(wrenches[0].getInputSize(), 0);

           for(auto& w : wrenches){
               _wrenches = _wrenches / w;
           }

           update(Eigen::VectorXd::Zero(0));

       }

       void FrictionCone::computeAineq()
       {
           _Ci.setZero(5,3);

           _A.resize(5*_n_of_contacts, 6*_n_of_contacts);

           _A.setZero(_A.rows(), _A.cols());

           for(unsigned int i = 0; i < _n_of_contacts; ++i)
           {

                double __mu = _mu[i].second;
                _wRl = _mu[i].first;

                __mu = std::sqrt(2.*__mu)/2.;

                _Ci(0,0) = 1.; _Ci(0,1) = 0.; _Ci(0,2) = -__mu;
                _Ci(1,0) = -1.; _Ci(1,1) = 0.; _Ci(1,2) = -__mu;
                _Ci(2,0) = 0.; _Ci(2,1) = 1.; _Ci(2,2) = -__mu;
                _Ci(3,0) = 0.; _Ci(3,1) = -1.; _Ci(3,2) = -__mu;
                _Ci(4,0) = 0.; _Ci(4,1) = 0.; _Ci(4,2) = -1.;

                _Ci = _Ci*_wRl.transpose();

                _A.block<5,3>(5*i, 6*i) = _Ci;
           }

       }

       void FrictionCone::computeUpperBound()
       {
           _b.resize(5*_n_of_contacts);
           _b.setZero(_b.rows());
           _bLowerBound.resize(5*_n_of_contacts);
           _bLowerBound = -1.0e20*_bLowerBound.setOnes(_bLowerBound.size());
       }



       void FrictionCone::update(const Eigen::VectorXd &x)
       {

               _n_of_contacts = _mu.size();

               computeAineq();
               computeUpperBound();

               _friction_cone = _A * _wrenches - _b;
               _Aineq = _friction_cone.getM();
               _bUpperBound = - _friction_cone.getq();

       }

       bool FrictionCone::setContactRotationMatrix(const Eigen::Matrix3d& wRl, const unsigned int i)
       {
           if(i >= _n_of_contacts)
               return false;

           _mu[i].first = wRl;
           return true;
       }

       }
   }
}

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

           computeAineq();
           computeUpperBound();
           
           OptvarHelper::VariableVector vars;
           for(auto fc : mu){
               vars.emplace_back(fc.first + "_wrench", 6);
           }
           
           OptvarHelper opthelper(vars);
           _wrenches.setZero(x.size(), 0);
           
           for(auto& w : opthelper.getAllVariables()){
               _wrenches = _wrenches / w;
           }

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

           computeAineq();
           computeUpperBound();
           
           _wrenches.setZero(wrenches[0].getInputSize(), 0);
           
           for(auto& w : wrenches){
               _wrenches = _wrenches / w;
           }

       }

       void FrictionCone::computeAineq()
       {
           _Ci.setZero(5,3);

            _Aineq.resize(5*_n_of_contacts, 6*_n_of_contacts);

           _Aineq.setZero(_Aineq.rows(), _Aineq.cols());

           for(unsigned int i = 0; i < _n_of_contacts; ++i)
           {
               _robot.getPose(_mu[i].first, _wTl);


                double __mu = _mu[i].second;

                __mu = std::sqrt(2.*__mu)/2.;

                _Ci(0,0) = 1.; _Ci(0,1) = 0.; _Ci(0,2) = -__mu;
                _Ci(1,0) = -1.; _Ci(1,1) = 0.; _Ci(1,2) = -__mu;
                _Ci(2,0) = 0.; _Ci(2,1) = 1.; _Ci(2,2) = -__mu;
                _Ci(3,0) = 0.; _Ci(3,1) = -1.; _Ci(3,2) = -__mu;
                _Ci(4,0) = 0.; _Ci(4,1) = 0.; _Ci(4,2) = -1.;

                _Ci = _Ci*_wTl.matrix().block(0,0,3,3).transpose();

                _Aineq.block(5*i, 6*i,5,3)<<_Ci;
           }

       }

       void FrictionCone::computeUpperBound()
       {
           _bUpperBound.resize(5*_n_of_contacts);
           _bUpperBound.setZero(_bUpperBound.rows());
           _bLowerBound.resize(5*_n_of_contacts);
           _bLowerBound = -1.0e20*_bLowerBound.setOnes(_bLowerBound.size());
       }



       void FrictionCone::update(const Eigen::VectorXd &x)
       {

               _n_of_contacts = _mu.size();

               computeAineq();
               computeUpperBound();
               
               _friction_cone = _Aineq * _wrenches - _bUpperBound;
               _Aineq = _friction_cone.getM();
               _bUpperBound = - _friction_cone.getq();
               
       }

       }
   }
}

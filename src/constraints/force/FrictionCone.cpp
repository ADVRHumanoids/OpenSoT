#include <OpenSoT/constraints/force/FrictionCone.h>

namespace OpenSoT {
   namespace constraints {
       namespace force {

       FrictionCone::FrictionCone(const Eigen::VectorXd &x, iDynUtils &robot,
                                  const friction_cones& mu):
           Constraint("friction_cone", x.rows()),
           _robot(robot),
           _mu(mu),
           _Ci(5,3)
       {
           _n_of_contacts = _mu.size();

           computeAineq();
           computeUpperBound();

       }

       void FrictionCone::computeAineq()
       {
           _Ci.setZero(5,3);

           if(_Aineq.rows() != 5*_n_of_contacts)
               _Aineq.resize(5*_n_of_contacts, 6*_n_of_contacts);

           _Aineq.setZero(_Aineq.rows(), _Aineq.cols());

           for(unsigned int i = 0; i < _n_of_contacts; ++i)
           {
                FrictionCone::R w_R_n = _mu[i].first;
                double __mu = _mu[i].second;

                __mu = std::sqrt(2.*__mu)/2.;

                _Ci(0,0) = 1.; _Ci(0,1) = 0.; _Ci(0,2) = -__mu;
                _Ci(1,0) = -1.; _Ci(1,1) = 0.; _Ci(1,2) = -__mu;
                _Ci(2,0) = 0.; _Ci(2,1) = 1.; _Ci(2,2) = -__mu;
                _Ci(3,0) = 0.; _Ci(3,1) = -1.; _Ci(3,2) = -__mu;
                _Ci(4,0) = 0.; _Ci(4,1) = 0.; _Ci(4,2) = -1.;

                _Ci = _Ci*w_R_n.transpose();

                _Aineq.block(5*i, 6*i,5,3)<<_Ci;
           }

       }

       void FrictionCone::computeUpperBound()
       {
           if(_bUpperBound.rows() != 5*_n_of_contacts){
           _bUpperBound.resize(5*_n_of_contacts);
           _bUpperBound.setZero(_bUpperBound.rows());}
       }



       void FrictionCone::update(const Eigen::VectorXd &x)
       {

               _n_of_contacts = _mu.size();

               computeAineq();
               computeUpperBound();
       }

       }
   }
}

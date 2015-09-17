#include <OpenSoT/constraints/force/FrictionCone.h>
#include <OpenSoT/constraints/velocity/Dynamics.h>

using namespace yarp::math;

namespace OpenSoT {
   namespace constraints {
       namespace force {

       FrictionCone::FrictionCone(const yarp::sig::Vector &x, iDynUtils &robot,
                                  const std::map<std::string, double>& mu,
                                  const std::map<std::string, yarp::sig::Matrix>& world_R_surfaces):
           Constraint(x.size()),
           _robot(robot),
           _mu(mu),
           _world_R_surfaces(world_R_surfaces)
       {
           OpenSoT::constraints::velocity::Dynamics::crawlLinks(_robot.getForceTorqueFrameNames(),
                      std::vector<std::string>{std::begin(_robot.getLinksInContact()),
                                               std::end(_robot.getLinksInContact())},
                                               _robot,
                                               _ft_in_contact);
           assert(_world_R_surfaces.size() == _ft_in_contact.size());
           assert(_mu.size() == _ft_in_contact.size());
           for(unsigned int i = 0; i < _world_R_surfaces.size(); ++i){
               assert(_world_R_surfaces.find(_ft_in_contact[i]) != _world_R_surfaces.end());
               assert(_mu.find(_ft_in_contact[i]) != _mu.end());}

           computeAineq();
           computeUpperBound();


           update(x);


       }

       void FrictionCone::computeAineq()
       {
            _Aineq.resize(4*_ft_in_contact.size(), 3*2*_ft_in_contact.size());
            _Aineq.zero();

            yarp::sig::Matrix Ci(4,3); Ci.zero();
            Ci(0,0) = 1.0;
            Ci(1,1) = 1.0;
            Ci(2,0) = -1.0;
            Ci(3,1) = -1.0;

            for(unsigned int i = 0; i < _ft_in_contact.size(); ++i)
            {
                double mu = _mu[_ft_in_contact[i]];

                 Ci(0,2) = -mu;
                 Ci(1,2) = -mu;
                 Ci(2,2) = -mu;
                 Ci(3,2) = -mu;

                _Aineq.setSubmatrix(Ci*(_world_R_surfaces[_ft_in_contact[i]]).transposed(),i*3, i*4);
            }
       }

       void FrictionCone::computeUpperBound()
       {
           _bUpperBound.resize(4*_ft_in_contact.size(),0.0);
       }



       void FrictionCone::update(const yarp::sig::Vector &x)
       {
           //   NICE feature: this constraint is constant if the number of contact points
           //   does not change.
           std::vector<std::string> ft_in_contact;
           OpenSoT::constraints::velocity::Dynamics::crawlLinks(_robot.getForceTorqueFrameNames(),
                      std::vector<std::string>{std::begin(_robot.getLinksInContact()),
                                               std::end(_robot.getLinksInContact())},
                                               _robot,
                                               ft_in_contact);
           if(!(_ft_in_contact == ft_in_contact))
           {
               _ft_in_contact = ft_in_contact;

               assert(_world_R_surfaces.size() == _ft_in_contact.size());
               assert(_mu.size() == _ft_in_contact.size());
               for(unsigned int i = 0; i < _world_R_surfaces.size(); ++i){
                   assert(_world_R_surfaces.find(_ft_in_contact[i]) != _world_R_surfaces.end());
                   assert(_mu.find(_ft_in_contact[i]) != _mu.end());}

               computeAineq();
               computeUpperBound();
           }
       }

       }
   }
}

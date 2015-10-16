#ifndef __TASKS_VELOCITY_COP_H__
#define __TASKS_VELOCITY_COP_H__

#include <OpenSoT/Task.h>
#include <idynutils/idynutils.h>
#include <kdl/frames.hpp>
#include <yarp/sig/all.h>
#include <yarp/os/all.h>

namespace OpenSoT {
   namespace tasks {
       namespace force {
       class CoP : public Task < yarp::sig::Matrix, yarp::sig::Vector > {
       public:
           typedef boost::shared_ptr<CoP> Ptr;
       private:
           #define BASE_LINK "world"

           iDynUtils& _robot;

           yarp::sig::Vector _desiredCoP;

           void update_b();

           double _d;

           std::string _ft_in_contact;
           int _ft_index;

           yarp::sig::Vector _wrench_in_sensor_frame;

           void computeA();

           public:
           CoP(const yarp::sig::Vector& x,
               iDynUtils& robot, double ft_to_sole_height, const std::string& ft_in_contact);

           ~CoP();

           void _update(const yarp::sig::Vector& x);

           void setReference(const yarp::sig::Vector& desiredCoP);

           yarp::sig::Vector getReference() const;

           std::string getBaseLink();


       };
       }
   }
}

#endif

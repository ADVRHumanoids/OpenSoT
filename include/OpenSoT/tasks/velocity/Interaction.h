#ifndef __TASKS_VELOCITY_INTERACTION_H__
#define __TASKS_VELOCITY_INTERACTION_H__

 #include <OpenSoT/Task.h>
#include <OpenSoT/tasks/velocity/Cartesian.h>
 #include <idynutils/idynutils.h>
 #include <kdl/frames.hpp>
 #include <yarp/sig/all.h>
 #include <yarp/os/all.h>

 #define WORLD_FRAME_NAME "world"

/**
 * @example example_cartesian.cpp
 * The Cartesian class implements a task that tries to impose a pose (position and orientation)
 * of a distal link w.r.t. a base link.
 */

 namespace OpenSoT {
    namespace tasks {
        namespace velocity {
            class Interaction : protected Cartesian {
            public:
                typedef boost::shared_ptr<Interaction> Ptr;
            private:
                yarp::sig::Vector _desiredWrench;
                yarp::sig::Vector _actualWrench;
                std::string _ft_frame;
                int _ft_index;
                yarp::sig::Matrix _C;


            public:

                yarp::sig::Vector forceError;
                yarp::sig::Vector torqueError;

                /*********** TASK PARAMETERS ************/



                /****************************************/


                Interaction(std::string task_id,
                            const yarp::sig::Vector& x,
                            iDynUtils &robot,
                            std::string distal_link,
                            std::string base_link,
                            std::string ft_frame);

                ~Interaction();

                void _update(const yarp::sig::Vector& x);

                void setReferenceWrench(const yarp::sig::Vector& desiredWrench);

                const yarp::sig::Vector getReferenceWrench() const;

                const yarp::sig::Vector getActualWrench() const;

                const std::string getForceTorqueReferenceFrame() const;

                const yarp::sig::Matrix getCompliance() const;

                void setCompliance(const yarp::sig::Matrix& C);

                };
        }
    }
 }

#endif

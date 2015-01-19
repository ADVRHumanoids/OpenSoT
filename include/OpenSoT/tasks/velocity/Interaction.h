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
            class Interaction : public Cartesian {
            public:
                typedef boost::shared_ptr<Interaction> Ptr;
            private:
                /**
                 * @brief _deisredWrench desired Wrench in the base_link reference frame
                 */
                yarp::sig::Vector _desiredWrench;

                /**
                 * @brief _actualWrench measured Wrench in the base_link reference frame
                 */
                yarp::sig::Vector _actualWrench;
                std::string _ft_frame;
                int _ft_index;
                yarp::sig::Matrix _C;

                void updateActualWrench();


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

                /**
                 * @brief setReferenceWrench set desired Wrench in the specified base_link reference frame
                 * @param desiredWrench [6x1] forces and torques
                 */
                void setReferenceWrench(const yarp::sig::Vector& desiredWrench);

                /**
                 * @brief getReferenceWrench get specified reference wrench
                 * @return [6x1] forces and torques
                 */
                const yarp::sig::Vector getReferenceWrench() const;

                /**
                 * @brief getActualWrench return measured wrench in the specified base_link reference frame
                 * @return [6x1] forces and torques
                 */
                const yarp::sig::Vector getActualWrench() const;

                /**
                 * @brief getForceTorqueReferenceFrame return the reference frame of the ft sensor
                 * @return ft frame
                 */
                const std::string getForceTorqueReferenceFrame() const;

                /**
                 * @brief getCompliance a Compliance matrix
                 * @return [6x6] pd compliance Matrix
                 */
                const yarp::sig::Matrix getCompliance() const;

                /**
                 * @brief setCompliance set a pd Compliance Matrix. If the compliance matrix is not pd the old one
                 * will be used.
                 * @param C [6x6] pd compliance Matrix
                 */
                void setCompliance(const yarp::sig::Matrix& C);

                };
        }
    }
 }

#endif

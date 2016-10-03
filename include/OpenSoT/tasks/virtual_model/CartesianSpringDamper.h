/*
 * Copyright (C) 2016 Cogimon
 * Authors: Enrico Mingo Hoffman, Alessio Rocchi
 * email:  enrico.mingo@iit.it, alessio.rocchi@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU Lesser General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#ifndef __TASKS_VIRTUAL_MODEL_CARTESIAN_SPRING_DAMPER_H__
#define __TASKS_VIRTUAL_MODEL_CARTESIAN_SPRING_DAMPER_H__

 #include <OpenSoT/Task.h>
 #include <idynutils/idynutils.h>
 #include <kdl/frames.hpp>
 #include <yarp/sig/all.h>
 #include <yarp/os/all.h>

 #define WORLD_FRAME_NAME "world"


 namespace OpenSoT {
    namespace tasks {
        namespace virtual_model {
            class CartesianSpringDamper : public Task < yarp::sig::Matrix, yarp::sig::Vector > {
            public:
                typedef boost::shared_ptr<CartesianSpringDamper> Ptr;
            protected:
                iDynUtils& _robot;

                std::string _distal_link;
                std::string _base_link;

                int _distal_link_index;
                int _base_link_index;

                yarp::sig::Matrix _actualPose;
                yarp::sig::Matrix _desiredPose;
                yarp::sig::Vector _desiredTwist;

                yarp::sig::Matrix _K;
                yarp::sig::Matrix _D;

                bool _base_link_is_world;

                bool _use_inertia_matrix;

                void update_b();

                yarp::sig::Matrix _M;
                yarp::sig::Matrix _J;

            public:

                yarp::sig::Vector positionError;
                yarp::sig::Vector orientationError;
                yarp::sig::Vector linearVelocityError;
                yarp::sig::Vector orientationVelocityError;


                /*********** TASK PARAMETERS ************/



                /****************************************/


                CartesianSpringDamper(std::string task_id,
                          const yarp::sig::Vector& x,
                          iDynUtils &robot,
                          std::string distal_link,
                          std::string base_link);

                ~CartesianSpringDamper();

                void _update(const yarp::sig::Vector& x);

                void setReference(const yarp::sig::Matrix& desiredPose);

                void setReference(const yarp::sig::Matrix& desiredPose,
                                  const yarp::sig::Vector& desiredTwist);

                const yarp::sig::Matrix getReference() const;

                void getReference(yarp::sig::Matrix& desiredPose,
                                  yarp::sig::Vector& desiredTwist) const;


                const yarp::sig::Matrix getActualPose() const;

                const KDL::Frame getActualPoseKDL() const;

                const std::string getDistalLink() const;
                const std::string getBaseLink() const;
                const bool baseLinkIsWorld() const;

                void setStiffness(const yarp::sig::Matrix& Stiffness);
                void setDamping(const yarp::sig::Matrix& Damping);
                void setStiffnessDamping(const yarp::sig::Matrix& Stiffness,
                                         const yarp::sig::Matrix& Damping);

                yarp::sig::Matrix getStiffness();
                yarp::sig::Matrix getDamping();
                void getStiffnessDamping(yarp::sig::Matrix& Stiffness, yarp::sig::Matrix& Damping);



                static bool isCartesianSpringDamper(OpenSoT::Task<yarp::sig::Matrix, yarp::sig::Vector>::TaskPtr task);

                static OpenSoT::tasks::virtual_model::CartesianSpringDamper::Ptr asCartesianSpringDamper(OpenSoT::Task<yarp::sig::Matrix, yarp::sig::Vector>::TaskPtr task);

                void useInertiaMatrix(const bool use);

                yarp::sig::Vector getSpringForce();
                yarp::sig::Vector getDamperForce();

            };
        }
    }
 }

#endif

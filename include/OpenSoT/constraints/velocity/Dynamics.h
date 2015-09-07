#ifndef __BOUNDS_Dynamics_H__
#define __BOUNDS_Dynamics_H__

 #include <OpenSoT/Constraint.h>

 #include <yarp/sig/all.h>
 #include <iCub/iDynTree/DynTree.h>
 #include <idynutils/idynutils.h>

 namespace OpenSoT {
    namespace constraints {
        namespace velocity {
            /**
             * @brief The Dynamics class implements constraints on joint velocities due to
             * dynamics feasability.
             *
             * For now we do not consider external forces: so the constraint can be used just
             * on the upper body of a humanoid robot if not interacting with anything
             */
            class Dynamics: public Constraint<yarp::sig::Matrix, yarp::sig::Vector> {
            public:
                typedef boost::shared_ptr<Dynamics> Ptr;
            private:
                yarp::sig::Vector _jointTorquesMin;
                yarp::sig::Vector _jointTorquesMax;
                iDynUtils& _robot_model;
                double _dT;

                /**
                 * @brief _b is used to store the partial constraint
                 */
                yarp::sig::Vector _b;
                /**
                 * @brief _M is used to store the Inertia matrix
                 */
                yarp::sig::Matrix _M;
            public:

                Dynamics(const yarp::sig::Vector &q,
                         const yarp::sig::Vector &q_dot,
                            const yarp::sig::Vector &jointTorquesMax,
                         iDynUtils& robot_model,
                         const double dT);

                /**
                 * @brief update
                 * @param x constains the concatenation of q and q_dot!
                 */
                void update(const yarp::sig::Vector &x);
            };
        }
    }
 }

#endif

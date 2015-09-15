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
             * The constraint is written as:
             *
             * u_min <= (M/dT)dq <= u_max
             *
             * u_min = dT*tau_min - dT*b + M*q_dot - dT*Jc'Fc
             * u_max = dT*tau_max - dT*b + M*q_dot - dT*Jc'Fc
             *
             * b = C(q,q_dot)q_dot + g(q)
             * Jc = [Jc1 Jc2 ... Jcn]'
             * Fc = [fc1 fc2 ... fcn]'
             *
             * where q_dot is the velocity in the previous step, J is the Jacobian of
             * all the contacts (here we consider these Jacobians from the /base_link
             * to the ft_sensors_frames), Fc are the contact forces (at the ft_sensors_
             * frames transformed in the base_link).
             *
             */
            class Dynamics: public Constraint<yarp::sig::Matrix, yarp::sig::Vector> {
            public:
                typedef boost::shared_ptr<Dynamics> Ptr;
            private:
                yarp::sig::Vector _jointTorquesMin;
                yarp::sig::Vector _jointTorquesMax;
                iDynUtils& _robot_model;
                double _dT;
                std::string _base_link;

                /**
                 * @brief _b is used to store the partial constraint
                 */
                yarp::sig::Vector _b;
                /**
                 * @brief _M is used to store the Inertia matrix
                 */
                yarp::sig::Matrix _M;

                /**
                 * @brief _Jc is used to store Jacobians for contact points (ft_sensors)
                 */
                yarp::sig::Matrix _Jc;

                /**
                 * @brief _Fc is used to store wrenches at contact points (ft_sensors)
                 */
                yarp::sig::Vector _Fc;

                yarp::sig::Vector _tmp_wrench_in_sensor_frame;
                yarp::sig::Vector _tmp_wrench_in_base_link_frame;

                double _boundScaling;

                /**
                 * @brief updateActualWrench this method updates the actual wrench at the base_link
                 * considering all the mearusments from the ft sensors.
                 * NOTE: here we use the REACTION forces!
                 */
                void updateActualWrench();

        public:

                /**
                 * @brief crawlLinks is a method that given a list of links in contact and a list of ft_links,
                 * return the list of ft_links associated with the links in contact.
                 * We consider two possible ways to describe the ft_link in the URDF:
                 *
                 * A)               |           o = link
                 *                  o           ft = ft_link
                 *                  |
                 *                 ft
                 *                  |
                 *                o_o_o
                 *
                 * B)               |           o = link
                 *                  o           ft = ft_link
                 *                  |
                 *                  o---ft
                 *                  |
                 *                o_o_o
                 *
                 * NOTE: It works ONLY if the list of links in contact contains links that
                 * does not have any child!
                 *
                 * @param ft_links_list is the list of all the ft_links of the robot
                 * @param contact_link_list is the list of the updated links in contact
                 * @param ft_in_contact_list updated list of ft in contact
                 */
                static void crawlLinks(const std::vector<std::string>& ft_links_list,
                                const std::vector<std::string>& contact_link_list,
                                iDynUtils& robot,
                                std::vector<std::string>& ft_in_contact_list);


                /**
                 * @brief Dynamics constraint constructor
                 * @param q actual position
                 * @param q_dot previous velocity
                 * @param jointTorquesMax maximum allowed torque
                 * @param robot_model model of the robot
                 * @param dT time step
                 * @param boundScaling bound scaling between maximum/minimum allowed torques
                 * and torques computed by ID
                 */
                Dynamics(const yarp::sig::Vector &q,
                         const yarp::sig::Vector &q_dot,
                            const yarp::sig::Vector &jointTorquesMax,
                         iDynUtils& robot_model,
                         const double dT, const double boundScaling);

                /**
                 * @brief update
                 * @param x constains the concatenation of q and q_dot:
                 *  x = [q, q_dot]
                 */
                void update(const yarp::sig::Vector &x);

                /**
                 * @brief setBoundScaling sets bound scaling for the capsule constraint
                 * @param boundScaling is a number which should be lower than 1.0
                 *        (e.g. 1./2. means we are looking two steps ahead and will avoid
                 *         collision with the capsule by slowing down)
                 */
                void setBoundScaling(const double boundScaling)
                {
                    _boundScaling = boundScaling;
                }
            };
        }
    }
 }

#endif

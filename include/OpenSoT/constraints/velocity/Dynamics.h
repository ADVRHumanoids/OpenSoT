#ifndef __BOUNDS_Dynamics_H__
#define __BOUNDS_Dynamics_H__

 #include <OpenSoT/Constraint.h>
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
            class Dynamics: public Constraint<Eigen::MatrixXd, Eigen::VectorXd> {
            public:
                typedef boost::shared_ptr<Dynamics> Ptr;
            private:
                Eigen::VectorXd _jointTorquesMin;
                Eigen::VectorXd _jointTorquesMax;
                iDynUtils& _robot_model;
                double _dT;
                std::string _base_link;
                Eigen::VectorXd _tmp_x;

                inline void pile(Eigen::MatrixXd& A, const Eigen::MatrixXd& B)
                {
                    A.conservativeResize(A.rows()+B.rows(), A.cols());
                    A.block(A.rows()-B.rows(),0,B.rows(),A.cols())<<B;
                }

                inline void pile(Eigen::VectorXd &a, const Eigen::VectorXd &b)
                {
                    a.conservativeResize(a.rows()+b.rows());
                    a.segment(a.rows()-b.rows(),b.rows())<<b;
                }

                /**
                 * @brief _b is used to store the partial constraint
                 */
                Eigen::VectorXd _b;
                /**
                 * @brief _M is used to store the Inertia matrix
                 */
               Eigen::MatrixXd _M;

                /**
                 * @brief _Jc is used to store Jacobians for contact points (ft_sensors)
                 */
               Eigen::MatrixXd _Jc;

                /**
                 * @brief _Fc is used to store wrenches at contact points (ft_sensors)
                 */
                Eigen::VectorXd _Fc;

                Eigen::VectorXd _tmp_wrench_in_sensor_frame;
                Eigen::VectorXd _tmp_wrench_in_base_link_frame;

                double _boundScaling;

                /**
                 * @brief updateActualWrench this method updates the actual wrench at the base_link
                 * considering all the mearusments from the ft sensors.
                 * NOTE: here we use the REACTION forces!
                 */
                void updateActualWrench();

                /**
                 * @brief _constraint_clipper, if true the bound is clipped at 0 (it cannot change sign)
                 */
                bool _constraint_clipper;

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
                                const std::list<std::string>& contact_link_list,
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
                Dynamics(const Eigen::VectorXd &q,
                         const Eigen::VectorXd &q_dot,
                            const Eigen::VectorXd &jointTorquesMax,
                         iDynUtils& robot_model,
                         const double dT, const double boundScaling);


                /**
                 * @brief getVelocityLimits returns the current velocity limits.
                 * @return the joint torque limits. It is always a positive double [Nm]
                 */
                Eigen::VectorXd getTorqueLimits();

                /**
                 * @brief getEstimatedTorque returns an estimate of the joint torques given a certain
                 *        solution vector
                 * @param dq the commanded joint velocities
                 * @return the vector of estimated torques on the joints,
                 *         based on the model state (position and velocity)
                 */
                Eigen::VectorXd getEstimatedTorques(const Eigen::VectorXd& dq);

                /**
                 * @brief setVelocityLimits
                 * @param tauLimits the joint torque limits. It needs be a positive number [Nm]
                 */
                void setTorqueLimits(const Eigen::VectorXd tauLimits);

                /**
                 * @brief update
                 * @param x constains either the concatenation of q and q_dot:
                 *  x = [q, q_dot], or x = q
                 */
                void update(const Eigen::VectorXd &x);

                /**
                 * @brief setBoundScaling sets bound scaling for the dynamics constraint
                 * @param boundScaling is a number which should be lower than 1.0
                 *        (e.g. 1./2. means we are looking two steps ahead and will avoid
                 *         collision with the dynamic limits by slowing down)
                 */
                void setBoundScaling(const double boundScaling)
                {
                    _boundScaling = boundScaling;
                }

                /**
                 * @brief getBoundScaling returns the bound scaling of the dynamics constraint
                 * @return the bound scaling
                 */
                double getBoundScaling() const
                {
                    return _boundScaling;
                }

                bool getConstraintClipperValue() const
                {
                    return _constraint_clipper;
                }

                void setConstraintClipperValue(const bool constraint_clipper)
                {
                    _constraint_clipper = constraint_clipper;
                }
            };
        }
    }
 }

#endif

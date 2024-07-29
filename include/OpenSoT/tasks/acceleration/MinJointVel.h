#ifndef __TASKS_ACCELERATION_MINJOINTVEL_H__
#define __TASKS_ACCELERATION_MINJOINTVEL_H__

#include <OpenSoT/Task.h>
#include <OpenSoT/tasks/acceleration/Postural.h>

namespace OpenSoT { namespace tasks { namespace acceleration {

   /**
     * @brief The MinJointVel class minimize the joint velocities in a joint acceleration scheme
     */
    class MinJointVel : public OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd> {

    public:
        typedef std::shared_ptr<MinJointVel> Ptr;

        /**
         * @brief MinJointVel
         * @param robot
         * @param dT control loop in [s]
         * @param qddot
         */
        MinJointVel(const XBot::ModelInterface& robot,
                    double dT,
                    AffineHelper qddot = AffineHelper());



        /**
         * @brief setEps set regularisation weight
         * @param eps
         * @return false is negative
         */
        bool setEps(const double eps);


    private:
        double _dT;
        const XBot::ModelInterface& _robot;
        OpenSoT::tasks::acceleration::Postural::Ptr _postural;
        Eigen::MatrixXd I;

        virtual void _update();


    };

        }
    }
}

#endif

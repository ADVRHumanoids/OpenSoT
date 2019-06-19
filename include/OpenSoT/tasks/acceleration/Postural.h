/*
 * Copyright (C) 2017 IIT-ADVR
 * Authors: Arturo Laurenzi
 * email:  arturo.laurenzi@iit.it
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


#ifndef __TASKS_ACCELERATION_POSTURAL_H__
#define __TASKS_ACCELERATION_POSTURAL_H__

#include <OpenSoT/Task.h>
#include <OpenSoT/utils/Affine.h>
#include <XBotInterface/ModelInterface.h>

namespace OpenSoT { namespace tasks { namespace acceleration {
    
    class Postural : public OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd> {
      
    public:
        
        typedef boost::shared_ptr<Postural> Ptr;
        
        Postural(const XBot::ModelInterface& robot,
                 AffineHelper qddot = AffineHelper());

        Postural(const XBot::ModelInterface& robot,
                 const int x_size);
        
        virtual void _update(const Eigen::VectorXd& x);
        
        /**
         * @brief setReference sets a new reference for the postural task.
         * The task error IS NOT recomputed immediately, you need to call the _update(x) function
         * Notice how the setReference(...) needs to be called before each _update(x) of the Cartesian task,
         * since THE _update() RESETS THE FEED-FORWARD VELOCITY and ACCELERATION TERMS for safety reasons.
         * @param qref the desired position
         * @param dqref describe the desired trajectory velocity
         * @param ddqref describe the desired trajectory acceleration, and it represents
         * a feed-forward term in task computation.
         */
        void setReference(const Eigen::VectorXd& qref);
        void setReference(const Eigen::VectorXd& qref, const Eigen::VectorXd& dqref);
        void setReference(const Eigen::VectorXd& qref, const Eigen::VectorXd& dqref,
                          const Eigen::VectorXd& ddqref);

        /**
         * @brief getReference
         * @return joint position desidred
         */
        Eigen::VectorXd getReference() const;

        void getReference(Eigen::VectorXd& q_desired) const;
        void getReference(Eigen::VectorXd& q_desired,
                          Eigen::VectorXd& qdot_desired) const;
        void getReference(Eigen::VectorXd& q_desired,
                          Eigen::VectorXd& qdot_desired,
                          Eigen::VectorXd& qddot_desired) const;

        Eigen::VectorXd getActualPositions();

        /**
         * @brief getError
         * @return position error
         */
        Eigen::VectorXd getError();

        Eigen::VectorXd getVelocityError();


        void setLambda(double lambda1, double lambda2);
        virtual void setLambda(double lambda);

        /**
         * @brief getLambda return both position and velocity convergence gains
         * @param lambda position gain
         * @param lambda2 velocity gain
         */
        void getLambda(double & lambda, double & lambda2);
        using Task::getLambda;

        /**
         * @brief getLambda2 return velocity convergence gain
         * @return lambda2 gain
         */
        const double getLambda2() const;

        bool reset();
        
        virtual void _log(XBot::MatLogger::Ptr logger);

        /**
         * @brief getCachedVelocityReference can be used to get Velocity reference after update(), it will reset
         * next update()
         * @return internal velcity reference
         */
        const Eigen::VectorXd& getCachedVelocityReference() const;

        /**
         * @brief getCachedAccelerationReference can be used to get Velocity reference after update(), it will reset
         * next update()
         * @return internal acceleration reference
         */
        const Eigen::VectorXd& getCachedAccelerationReference() const;
        
        
    private:
        Eigen::VectorXd _position_error, _velocity_error;
        
        const XBot::ModelInterface& _robot;
        AffineHelper _qddot;
        AffineHelper _postural_task;
        
        int _na;
        
        Eigen::VectorXd _qddot_d, _qddot_ref, _qref, _qdot, _q, _qdot_ref, _qdot_ref_cached, _qddot_ref_cached;
        Eigen::MatrixXd _Jpostural;

        double _lambda2;
        
        
    };
    
    
} } }





#endif

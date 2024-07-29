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
#include <xbot2_interface/xbotinterface2.h>
#include <OpenSoT/tasks/acceleration/GainType.h>

namespace OpenSoT { namespace tasks { namespace acceleration {
    
    /**
     * @brief The Postural class implement a postural task on the ACTUATED joints.
     * Underactuated joints are not considered.
     * NOTICE that input vectors and matrices anyway consider the full 6+n state
     *
     */
    class Postural : public OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd> {
      
    public:
        
        typedef std::shared_ptr<Postural> Ptr;
        
        Postural(const XBot::ModelInterface& robot,
                 AffineHelper qddot, const std::string task_id = "Postural");

        Postural(const XBot::ModelInterface& robot,
                 const std::string task_id = "Postural");

        void setGainType(GainType type);
        GainType getGainType() const;
        
        /**
         * @brief setReference sets a new reference for the postural actuated part.
         * The task error IS NOT recomputed immediately, you need to call the _update(x) function
         * Notice how the setReference(...) needs to be called before each _update(x) of the Cartesian task,
         * since THE _update() RESETS THE FEED-FORWARD VELOCITY and ACCELERATION TERMS for safety reasons.
         * @param qref the desired position 6+n vector, first 6 joints are removed
         * @param dqref describe the desired trajectory velocity 6+n vector, first 6 joints are removed
         * @param ddqref describe the desired trajectory acceleration, and it represents 
         * a feed-forward term in task computation, 6+n vector, first 6 joints are removed.
         */
        void setReference(const Eigen::VectorXd& qref);
        void setReference(const Eigen::VectorXd& qref, const Eigen::VectorXd& dqref);
        void setReference(const Eigen::VectorXd& qref, const Eigen::VectorXd& dqref,
                          const Eigen::VectorXd& ddqref);

        /**
         * @brief getReference return references (6+n vectors)
         * @return joint position, velocity and acceleration desired
         */
        const Eigen::VectorXd& getReference() const;
        void getReference(Eigen::VectorXd& q_desired) const;
        void getReference(Eigen::VectorXd& q_desired,
                          Eigen::VectorXd& qdot_desired) const;
        void getReference(Eigen::VectorXd& q_desired,
                          Eigen::VectorXd& qdot_desired,
                          Eigen::VectorXd& qddot_desired) const;

        const Eigen::VectorXd& getActualPositions() const;

        /**
         * @brief getError
         * @return position error vector (6+n vector)
         */
        const Eigen::VectorXd& getError() const;

        /**
         * @brief getVelocityError
         * @return velocity error vector (6+n vector)
         */
        const Eigen::VectorXd& getVelocityError() const;

        /**
         * @brief setLambda
         * @param lambda1
         * @param lambda2
         */
        void setLambda(double lambda1, double lambda2);

        /**
         * @brief setLambda
         * @param lambda
         */
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

        /**
         * @brief reset position reference to actual, velcoity and accelration to 0
         * @return true
         */
        bool reset();
        
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
        
/**
         * @brief setKp set position gain
         * @param Kp a SPD matrix (n+6 x n+6), first 6 elements are not used
         */
        void setKp(const Eigen::MatrixXd& Kp);

        /**
         * @brief setKd set velocity gain
         * @param Kd a SPD matrix (n+6 x n+6), first 6 elements are not used
         */
        void setKd(const Eigen::MatrixXd& Kd);

        /**
         * @brief setGains set both position and velocity gains
         * @param Kp a SPD matrix (n+6 x n+6), first 6 elements are not used
         * @param Kd a SPD matrix (n+6 x n+6), first 6 elements are not used
         */
        void setGains(const Eigen::MatrixXd& Kp, const Eigen::MatrixXd& Kd);

        /**
         * @brief getKp
         * @return position gain (n+6 x n+6), first 6 elements are not used
         */
        const Eigen::MatrixXd& getKp() const;

        /**
         * @brief getKd
         * @return  velocity gain (n+6 x n+6), first 6 elements are not used
         */
        const Eigen::MatrixXd& getKd() const;

        /**
         * @brief getGains return both position and velocity gains
         * @param Kp (n+6 x n+6), first 6 elements are not used
         * @param Kd (n+6 x n+6), first 6 elements are not used
         */
        void getGains(Eigen::MatrixXd& Kp, Eigen::MatrixXd& Kd);

        
    private:
        GainType _gain_type;

        Eigen::VectorXd _position_error, _velocity_error;
        
        const XBot::ModelInterface& _robot;
        AffineHelper _qddot;
        AffineHelper _postural_task;
        
        Eigen::VectorXd _qddot_d, _qddot_ref, _qref, _qdot, _q, _qdot_ref, _qdot_ref_cached, _qddot_ref_cached;

        double _lambda2;

        Eigen::MatrixXd _Kp, _Kd;

        Eigen::MatrixXd _Mi;

        virtual void _update();
        virtual void _log(XBot::MatLogger2::Ptr logger);


        
        
    };
    
    
} } }





#endif

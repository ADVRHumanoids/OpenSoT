/*
 * Copyright (C) 2017 Walkman
 * Authors: Enrico Mingo Hoffman, Pouya Mohammadi
 * email:  enrico.mingo@iit.it, p.mohammadi@tu-bs.de
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

#ifndef __TASKS_VELOCITY_LINEAR_MOMENTUM_H__
#define __TASKS_VELOCITY_LINEAR_MOMENTUM_H__

#include <OpenSoT/Task.h>
#include <xbot2_interface/xbotinterface2.h>
#include <kdl/frames.hpp>
#include <Eigen/Dense>

namespace OpenSoT {
   namespace tasks {
       namespace velocity {

    #define BASE_LINK_COM "world"
    #define DISTAL_LINK_COM "CoM"

       /**
        * @brief The LinearMomentum class implement the tracking of a desired linear momentum:        
        *
        *               \f$ \mathbf{A} \Delta \mathbf{q} = \mathbf{h}_d \text{dT}  \f$
        *
        * where \f$ \mathbf{h}_d \f$ is the desired linear momentum at the CoM
        * @note This is basically a copy of Enrico Mingo's AngularMomentum with
        * one difference that it takes the linear part of centroidal momentum matrix.
        */
       class LinearMomentum : public Task < Eigen::MatrixXd, Eigen::VectorXd > {
       public:
           typedef std::shared_ptr<LinearMomentum> Ptr;

       private:
           XBot::ModelInterface& _robot;

           Eigen::Vector3d _desiredLinearMomentum;

           Eigen::MatrixXd _Momentum;

           void _update();

        public:
           /**
            * @brief LinearMomentum constructor
            * @param x joint states
            * @param robot reference to a model
            */
           LinearMomentum(XBot::ModelInterface& robot);
           ~LinearMomentum();

           /**
            * @brief setReference set a desired linear momentum at CoM
            * @param desiredLinearMomentum vector 3x1
            * NOTE: the input desired linear momentum has to be multiplied by \f$ \text{dT} \f$!
            */
           void setReference(const Eigen::Vector3d& desiredLinearMomentum);
           void setReference(const KDL::Vector& desiredLinearMomentum);

           /**
            * @brief getReference get the desired linear momentum at CoM
            * @param desiredLinearMomentum vector 3x1
            */
           void getReference(Eigen::Vector3d& desiredLinearMomentum) const;
           void getReference(KDL::Vector& desiredLinearMomentum) const;

           /**
            * @brief getBaseLink
            * @return "world"
            */
           std::string getBaseLink();

           /**
            * @brief getDistalLink
            * @return "CoM"
            */
           std::string getDistalLink();

           /**
            * @brief isLinearMomentum
            * @param task a pointer to a Task
            * @return true if the task is LinearMomentum
            */
           static bool isLinearMomentum(OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr task);

           /**
            * @brief asLinearMomentum
            * @param task a pointer to a Task
            * @return a pointer to an LinearMomentum Task
            */
           static OpenSoT::tasks::velocity::LinearMomentum::Ptr asLinearMomentum(OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr task);
       };
    }
   }
}

#endif

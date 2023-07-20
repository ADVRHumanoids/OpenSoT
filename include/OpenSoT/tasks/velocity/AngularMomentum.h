/*
 * Copyright (C) 2017 Walkman
 * Authors: Enrico Mingo Hoffman
 * email:  enrico.mingo@iit.it
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

#ifndef __TASKS_VELOCITY_ANGULAR_MOMENTUM_H__
#define __TASKS_VELOCITY_ANGULAR_MOMENTUM_H__

#include <OpenSoT/Task.h>
#include <XBotInterface/ModelInterface.h>
#include <kdl/frames.hpp>
#include <Eigen/Dense>

namespace OpenSoT {
   namespace tasks {
       namespace velocity {

    #define BASE_LINK_COM "world"
    #define DISTAL_LINK_COM "CoM"

       /**
        * @brief The AngularMomentum class implement the tracking of a desired angular momentum:
        *
        *               \f$ \mathbf{A} \Delta \mathbf{q} = \mathbf{h}_d \text{dT}  \f$
        *
        * where \f$ \mathbf{h}_d \f$ is the desired angular momentum at the CoM
        */
       class AngularMomentum : public Task < Eigen::MatrixXd, Eigen::VectorXd > {
       public:
           typedef std::shared_ptr<AngularMomentum> Ptr;

       private:
           XBot::ModelInterface& _robot;

           Eigen::Vector3d _desiredAngularMomentum;

           Eigen::MatrixXd _Momentum;

           void _update(const Eigen::VectorXd& x);

           std::string _base_link;
           std::string _distal_link;

        public:
           /**
            * @brief AngularMomentum constructor
            * @param x joint states
            * @param robot reference to a model
            */
           AngularMomentum(const Eigen::VectorXd& x, XBot::ModelInterface& robot);
           ~AngularMomentum();

           /**
            * @brief setReference set a desired angular momentum at CoM
            * @param desiredAngularMomentum vector 3x1
            * NOTE: the input desired angular momentum has to be multiplied by \f$ \text{dT} \f$!
            */
           void setReference(const Eigen::Vector3d& desiredAngularMomentum);
           void setReference(const KDL::Vector& desiredAngularMomentum);

           /**
            * @brief getReference get the desired angular momentum at CoM
            * @param desiredAngularMomentum vector 3x1
            */
           void getReference(Eigen::Vector3d& desiredAngularMomentum) const;
           void getReference(KDL::Vector& desiredAngularMomentum) const;

           /**
            * @brief getBaseLink
            * @return "world"
            */
           const std::string& getBaseLink() const;

           /**
            * @brief getDistalLink
            * @return "CoM"
            */
           const std::string& getDistalLink() const;

           /**
            * @brief isAngularMomentum
            * @param task a pointer to a Task
            * @return true if the task is AngularMomentum
            */
           static bool isAngularMomentum(OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr task);

           /**
            * @brief asAngularMomentum
            * @param task a pointer to a Task
            * @return a pointer to an AngularMomentum Task
            */
           static OpenSoT::tasks::velocity::AngularMomentum::Ptr asAngularMomentum(OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr task);
       };
    }
   }
}

#endif

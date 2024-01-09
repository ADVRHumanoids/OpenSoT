/*
 * Copyright (C) 2014 Walkman
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

#ifndef FRICTIONCONE_H
#define FRICTIONCONE_H


#include <OpenSoT/Constraint.h>
#include <xbot2_interface/xbotinterface2.h>
#include <kdl/frames.hpp>
#include <OpenSoT/utils/Affine.h>
#include <OpenSoT/constraints/Aggregated.h>
#include <memory>

#include <Eigen/Dense>


 namespace OpenSoT {
    namespace constraints {
        namespace force {


            class FrictionCone: public Constraint<Eigen::MatrixXd, Eigen::VectorXd> {
            public:
                typedef std::shared_ptr<FrictionCone> Ptr;

                /**
                 * @brief friction_cone is defined by a Rotation matrix (the rotation from world frame to contatc
                 * surface) and friction coefficient
                 */
                typedef std::pair<Eigen::Matrix3d, double> friction_cone;

                /**
                 * @brief _mu is a map between contacts and friction cone: The rotation is the one from
                 * world frame to contact frame
                 * NOTE that the rotation has the z-axiz parallel to the normal of the surface
                 */
                friction_cone _mu; //Friction Coefficient associated to each contact surface
                XBot::ModelInterface& _robot;

                Eigen::Matrix<double, 5, 3> _Ci;

                Eigen::MatrixXd _A;
                Eigen::VectorXd _b;

                AffineHelper _friction_cone;
                AffineHelper _wrench;

                Eigen::Matrix3d _wRl;

            public:

                FrictionCone(const std::string& contact_name,
                             const AffineHelper& wrench,
                             XBot::ModelInterface &robot,
                             const friction_cone & mu);


                void update(const Eigen::VectorXd &x);

                void setFrictionCone(const friction_cone& frc);

                void setMu(const double mu);


                /**
                 * @brief setContactRotationMatrix set the contact roation matrix for the ith-contact
                 * @param wRl
                 * @return true if everything went fine
                 */
                void setContactRotationMatrix(const Eigen::Matrix3d& wRl);

            private:
                void computeAineq();

        };

            class FrictionCones: public Constraint<Eigen::MatrixXd, Eigen::VectorXd> {
            public:
                typedef std::vector<FrictionCone::friction_cone> friction_cones;
                typedef std::shared_ptr<FrictionCones> Ptr;

                FrictionCones(const std::vector<std::string>& contact_name,
                             const std::vector<AffineHelper>& wrench,
                             XBot::ModelInterface &robot,
                             const friction_cones & mu);

                FrictionCone::Ptr getFrictionCone(const std::string& contact_name);

                void update(const Eigen::VectorXd &x);

            private:
                std::map<std::string, FrictionCone::Ptr> _friction_cone_map;
                OpenSoT::constraints::Aggregated::Ptr _internal_constraint;
                void generateBounds();
            };
    }
}
}


#endif // FRICTIONCONE_H


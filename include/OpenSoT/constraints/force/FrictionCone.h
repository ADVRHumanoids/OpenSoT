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
#include <XBotInterface/ModelInterface.h>
#include <kdl/frames.hpp>
#include <OpenSoT/utils/Affine.h>

#include <Eigen/Dense>


 namespace OpenSoT {
    namespace constraints {
        namespace force {


            class FrictionCone: public Constraint<Eigen::MatrixXd, Eigen::VectorXd> {
            public:
                typedef boost::shared_ptr<FrictionCone> Ptr;

                typedef std::pair<std::string, double> friction_cone;
                typedef std::vector<friction_cone> friction_cones;

        /**
         * @brief _mu is a map between contacts and friction cone: The rotation is the one from
         * world frame to contact frame
         * NOTE that the rotation has the z-axiz parallel to the normal of the surface
         */
        friction_cones _mu; //Friction Coefficient associated to each contact surface
        XBot::ModelInterface& _robot;

        Eigen::Matrix<double, 5, 3> _Ci;

        std::vector<Eigen::Affine3d> _wTl;

        int _n_of_contacts;

        Eigen::MatrixXd _A;
        Eigen::VectorXd _b;

        AffineHelper _friction_cone;
        AffineHelper _wrenches;

    public:

        /**
         * @brief FrictionCone
         * @param x
         * @param robot
         * @param mu is a map between links in contact and associated friction coefficient mu
         * NOTE: that all the friction cones are specified in world frame!
         */
        FrictionCone(const Eigen::VectorXd& x,
                     XBot::ModelInterface &robot,
                     const friction_cones & mu);

        FrictionCone(const std::vector<AffineHelper>& wrenches,
                     XBot::ModelInterface &robot,
                     const friction_cones & mu);


        void update(const Eigen::VectorXd &x);

        void setMu(const friction_cones& mu){ _mu = mu;}

        int getNumberOfContacts(){return _n_of_contacts;}

    private:
        void computeAineq();
        void computeUpperBound();

            };
        }
    }
 }


#endif // FRICTIONCONE_H


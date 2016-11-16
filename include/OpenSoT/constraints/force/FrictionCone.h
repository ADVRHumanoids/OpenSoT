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
 #include <idynutils/idynutils.h>
 #include <idynutils/collision_utils.h>
 #include <kdl/frames.hpp>

#include <Eigen/Dense>


 namespace OpenSoT {
    namespace constraints {
        namespace force {

            
            class FrictionCone: public Constraint<Eigen::MatrixXd, Eigen::VectorXd> {
            public:
                typedef boost::shared_ptr<FrictionCone> Ptr;
            protected:
                typedef Eigen::Matrix3d R; //Rotation
                typedef std::pair<R, double> friction_cone;
                typedef std::vector<friction_cone> friction_cones;

        /**
         * @brief _mu is a map between strings and friction cone: the rotation of the surface in world frame and the
         * friction coefficent.
         * NOTE that the rotation has the z-axiz parallel to the normal of the surface
         * NOTE2 the strings represent links in contact! They have to be ordered in the same
         * way of the optimized task!
         */
        friction_cones _mu; //Friction Coefficient associated to each contact surface
		iDynUtils& _robot;

        Eigen::MatrixXd _Ci;

        int _n_of_contacts;

            public:
                
        /**
                 * @brief FrictionCone
                 * @param x
                 * @param robot
                 * @param mu is a map between links in contact and associated friction coefficient mu
                 * NOTE: that all the friction cones are specified in world frame!
                 */
                FrictionCone(const Eigen::VectorXd& x,
                             iDynUtils &robot,
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


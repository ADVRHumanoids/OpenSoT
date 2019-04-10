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
#include <OpenSoT/constraints/Aggregated.h>
#include <boost/make_shared.hpp>

#include <Eigen/Dense>


 namespace OpenSoT {
    namespace constraints {
        namespace force {


            class FrictionCone: public Constraint<Eigen::MatrixXd, Eigen::VectorXd> {
            public:
                typedef boost::shared_ptr<FrictionCone> Ptr;

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

                void setMu(const friction_cone& mu);


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
                typedef boost::shared_ptr<FrictionCones> Ptr;

                FrictionCones(const std::vector<std::string>& contact_name,
                             const std::vector<AffineHelper>& wrench,
                             XBot::ModelInterface &robot,
                             const friction_cones & mu):
                    Constraint("friction_cones", wrench[0].getInputSize())
                {
                    std::list<ConstraintPtr> constraint_list;
                    for(unsigned int i = 0; i < contact_name.size(); ++i){
                        _friction_cone_map[contact_name[i]] = boost::make_shared<FrictionCone>
                                (contact_name[i], wrench[i], robot, mu[i]);
                        constraint_list.push_back(_friction_cone_map[contact_name[i]]);
                    }

                    _internal_constraint = boost::make_shared<OpenSoT::constraints::Aggregated>
                            (constraint_list, wrench[0].getInputSize());

                    update(Eigen::VectorXd(0));
                }

                FrictionCone::Ptr getFrictionCone(const std::string& contact_name)
                {
                    if(_friction_cone_map.count(contact_name))
                        return _friction_cone_map[contact_name];
                    else
                        return NULL;
                }

                void update(const Eigen::VectorXd &x)
                {
                    _internal_constraint->update(x);
                    generateBounds();
                }

            private:
                std::map<std::string, FrictionCone::Ptr> _friction_cone_map;
                OpenSoT::constraints::Aggregated::Ptr _internal_constraint;
                void generateBounds()
                {
                    _Aineq = _internal_constraint->getAineq();
                    _bUpperBound = _internal_constraint->getbUpperBound();
                    _bLowerBound = _internal_constraint->getbLowerBound();
                }
            };
    }
}
}


#endif // FRICTIONCONE_H


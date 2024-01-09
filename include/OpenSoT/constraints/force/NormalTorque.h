/*
 * Copyright (C) 2021 EUROBENCH
 * Authors: Enrico Mingo Hoffman, Luca Rossini
 * email:  enrico.mingo@iit.it, luca.rossini@iit.it
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

#ifndef _CONSTRAINT_NORMAL_TORQUE_LIMITS_H_
#define _CONSTRAINT_NORMAL_TORQUE_LIMITS_H_

#include <OpenSoT/Constraint.h>
#include <xbot2_interface/xbotinterface2.h>
#include <kdl/frames.hpp>
#include <OpenSoT/utils/Affine.h>
#include <OpenSoT/utils/Piler.h>
#include <OpenSoT/constraints/Aggregated.h>
#include <OpenSoT/constraints/force/FrictionCone.h>

namespace OpenSoT {
    namespace constraints {
        namespace force {

/**
 * @brief The NormalTorque class is based on the paper:
 * "Stability of Surface Contacts for Humanoid Robots: Closed-Form Formulae of the Contact Wrench Cone
for Rectangular Support Areas", by S. Caron, Q.-C. Pham and Y. Nakamura
 * https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=7139910
*/
class NormalTorque: public Constraint<Eigen::MatrixXd, Eigen::VectorXd> {
public:
    typedef std::shared_ptr<NormalTorque> Ptr;

    /**
     * @brief NormalTorque
     * @param contact_link
     * @param wrench
     * @param model
     * @param X_Lims [xl, xu] limits w.r.t. contact frame
     * @param Y_Lims [yl, yu] limits w.r.t. contact frame
     * @param mu friction coeff
     */
    NormalTorque(const std::string& contact_link,
                 const AffineHelper& wrench,
                 XBot::ModelInterface& model,
                 const Eigen::Vector2d& X_Lims,
                 const Eigen::Vector2d& Y_Lims,
                 const double& mu);

    void update(const Eigen::VectorXd &x);

    /**
     * @brief setMu update friction coefficient
     * @param mu
     */
    void setMu(const double mu);

private:

    void _updateA();

    std::string _contact_link;
    XBot::ModelInterface& _model;
    double _mu;
    double _X, _Y;

    AffineHelper _constraint;
    AffineHelper _wrench;

    /**
     * @brief _A matrix to store the constraint
     */
    Eigen::MatrixXd _A;

    Eigen::Affine3d _T;
    Eigen::Affine3d _Ti;

    /**
     * @brief _Ad rotates wrench from world to local contact_link frame
     */
    Eigen::MatrixXd _Ad;
    /**
     * @brief _Ad2 moves wrench application from contact_link frame to center of the contact
     */
    Eigen::MatrixXd _Ad2;

    Eigen::MatrixXd _AAd;

};

class NormalTorques: public Constraint<Eigen::MatrixXd, Eigen::VectorXd> {
public:
    typedef std::shared_ptr<NormalTorques> Ptr;

    NormalTorques(const std::vector<std::string>& contact_name,
                  const std::vector<AffineHelper>& wrench,
                  XBot::ModelInterface &robot,
                  const std::vector<Eigen::Vector2d> & Xs,
                  const std::vector<Eigen::Vector2d> & Ys,
                  const std::vector<double> & mu);

    NormalTorque::Ptr getNormalTorque(const std::string& contact_name);

    void update(const Eigen::VectorXd &x);

private:
    std::map<std::string, NormalTorque::Ptr> _normal_torque_map;
    OpenSoT::constraints::Aggregated::Ptr _internal_constraint;
    void generateBounds();
};

       }
   }
}

#endif

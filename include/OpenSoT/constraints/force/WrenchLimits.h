/*
 * Copyright (C) 2017 Cogimon
 * Author: Enrico Mingo Hoffman
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

#ifndef __BOUNDS_FORCE_WRENCHLIMITS_H__
#define __BOUNDS_FORCE_WRENCHLIMITS_H__

#include <OpenSoT/constraints/GenericConstraint.h>
#include <Eigen/Dense>

 namespace OpenSoT {
    namespace constraints {
        namespace force {
            class WrenchLimits: public Constraint<Eigen::MatrixXd, Eigen::VectorXd> {
            public:
                typedef boost::shared_ptr<WrenchLimits> Ptr;
            private:
                Eigen::VectorXd _lowerLims;
                Eigen::VectorXd _upperLims;

                Eigen::VectorXd _zeros;

                OpenSoT::constraints::GenericConstraint::Ptr _constr_internal;

                bool _is_released;

            public:
                WrenchLimits(const std::string& contact_name,
                             const Eigen::VectorXd& lowerLims,
                             const Eigen::VectorXd& upperLims,
                             AffineHelper wrench);

                void getWrenchLimits(Eigen::VectorXd& lowerLims, Eigen::VectorXd& upperLims);
                void setWrenchLimits(const Eigen::VectorXd& lowerLims, const Eigen::VectorXd& upperLims);

                /**
                 * @brief releaseContact of true the wrench limits are all set to 0, if false the
                 * normal limits are used.
                 * @param released true/false
                 */
                void releaseContact(bool released);

                /**
                 * @brief isReleased
                 * @return true if the wrench limts are all zeros
                 */
                bool isReleased();


            private:
                void generateBounds();
            };
        }
    }
 }

#endif

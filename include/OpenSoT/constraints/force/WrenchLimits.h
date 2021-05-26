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
#include <OpenSoT/constraints/Aggregated.h>
#include <OpenSoT/Constraint.h>
#include <Eigen/Dense>
#include <memory>


 namespace OpenSoT {
    namespace constraints {
        namespace force {
        /**
             * @brief The WrenchLimits class implements wrench limits
             */
            class WrenchLimits: public Constraint<Eigen::MatrixXd, Eigen::VectorXd> {
            public:
                typedef std::shared_ptr<WrenchLimits> Ptr;
            private:
                Eigen::VectorXd _lowerLims;
                Eigen::VectorXd _upperLims;

                Eigen::VectorXd _zeros;

                OpenSoT::constraints::GenericConstraint::Ptr _constr_internal;

                bool _is_released;

            public:
                /**
                 * @brief WrenchLimits Contructor
                 * @param contact_name name fo the contact associated to the variable
                 * @param lowerLims lower limits
                 * @param upperLims upper limits
                 * @param wrench variable
                 */
                WrenchLimits(const std::string& contact_name,
                             const Eigen::VectorXd& lowerLims,
                             const Eigen::VectorXd& upperLims,
                             AffineHelper wrench);

                /**
                 * @brief getWrenchLimits to retrieve internal limits
                 * @param lowerLims
                 * @param upperLims
                 */
                void getWrenchLimits(Eigen::VectorXd& lowerLims, Eigen::VectorXd& upperLims);

                /**
                 * @brief setWrenchLimits to set internal limits
                 * @param lowerLims
                 * @param upperLims
                 */
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

            /**
             * @brief The WrenchesLimits class extend wrench limit to a vector of wrenches
             */
            class WrenchesLimits: public Constraint<Eigen::MatrixXd, Eigen::VectorXd> {
            public:
                typedef std::shared_ptr<WrenchesLimits> Ptr;

                /**
                 * @brief WrenchesLimits constructor
                 * @param contact_name vector of names associated to a vector of wrench variables
                 * @param lowerLims lower limits
                 * @param upperLims upper limits
                 * @param wrench
                 */
                WrenchesLimits(const std::vector<std::string>& contact_name,
                               const Eigen::VectorXd& lowerLims,
                               const Eigen::VectorXd& upperLims,
                               const std::vector<AffineHelper>& wrench);

                /**
                 * @brief WrenchesLimits constructor
                 * @param contact_name vector of names associated to a vector of wrench variables
                 * @param lowerLims vector of lower limits
                 * @param upperLims vector of upper limits
                 * @param wrench
                 */
                WrenchesLimits(const std::vector<std::string>& contact_name,
                               const std::vector<Eigen::VectorXd>& lowerLims,
                               const std::vector<Eigen::VectorXd>& upperLims,
                               const std::vector<AffineHelper>& wrench);

                /**
                 * @brief getWrenchLimits to access to internal wrench limits
                 * @param contact_name of the contact to get the associated wrench limit
                 * @return a wrench limit constraint
                 */
                WrenchLimits::Ptr getWrenchLimits(const std::string& contact_name);

                void update(const Eigen::VectorXd &x);

            private:
                std::map<std::string, WrenchLimits::Ptr> wrench_lims_constraints;
                OpenSoT::constraints::Aggregated::Ptr _aggregated_constraint;
                virtual void generateBounds();

            };
        }
    }
 }

#endif

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
#include <boost/make_shared.hpp>


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

            class WrenchesLimits: public Constraint<Eigen::MatrixXd, Eigen::VectorXd> {
            public:
                typedef boost::shared_ptr<WrenchesLimits> Ptr;

                WrenchesLimits(const std::vector<std::string>& contact_name,
                               const Eigen::VectorXd& lowerLims,
                               const Eigen::VectorXd& upperLims,
                               std::vector<AffineHelper>& wrench):
                    Constraint("wrenches_limits", wrench[0].getInputSize())
                {
                    std::list<ConstraintPtr> constraint_list;
                    for(unsigned int i = 0; i < contact_name.size(); ++i){
                        wrench_lims_constraints[contact_name[i]] = boost::make_shared<WrenchLimits>
                                (contact_name[i], lowerLims, upperLims, wrench[i]);
                        constraint_list.push_back(wrench_lims_constraints[contact_name[i]]);
                    }

                    _aggregated_constraint = boost::make_shared<OpenSoT::constraints::Aggregated>
                            (constraint_list, wrench[0].getInputSize());

                    generateBounds();
                }

                WrenchesLimits(const std::vector<std::string>& contact_name,
                               const std::vector<Eigen::VectorXd>& lowerLims,
                               const std::vector<Eigen::VectorXd>& upperLims,
                               std::vector<AffineHelper>& wrench):
                    Constraint("wrenches_limits", wrench[0].getInputSize())
                {
                    std::list<ConstraintPtr> constraint_list;
                    for(unsigned int i = 0; i < contact_name.size(); ++i){
                        wrench_lims_constraints[contact_name[i]] = boost::make_shared<WrenchLimits>
                                (contact_name[i], lowerLims[i], upperLims[i], wrench[i]);
                        constraint_list.push_back(wrench_lims_constraints[contact_name[i]]);
                    }

                    _aggregated_constraint = boost::make_shared<OpenSoT::constraints::Aggregated>
                            (constraint_list, wrench[0].getInputSize());

                    generateBounds();
                }

                WrenchLimits::Ptr getWrenchLimits(const std::string& contact_name)
                {
                    if(wrench_lims_constraints.count(contact_name))
                        return wrench_lims_constraints[contact_name];
                    else
                        return NULL;
                }

                void update(const Eigen::VectorXd &x)
                {
                    _aggregated_constraint->update(x);
                    generateBounds();
                }

            private:
                std::map<std::string, WrenchLimits::Ptr> wrench_lims_constraints;
                OpenSoT::constraints::Aggregated::Ptr _aggregated_constraint;
                virtual void generateBounds()
                {
                    if(_aggregated_constraint->isInequalityConstraint())
                    {
                        _Aineq = _aggregated_constraint->getAineq();
                        _bUpperBound = _aggregated_constraint->getbUpperBound();
                        _bLowerBound = _aggregated_constraint->getbLowerBound();
                    }
                    else if(_aggregated_constraint->isBound())
                    {
                        _upperBound = _aggregated_constraint->getUpperBound();
                        _lowerBound = _aggregated_constraint->getLowerBound();
                    }
                }

            };
        }
    }
 }

#endif

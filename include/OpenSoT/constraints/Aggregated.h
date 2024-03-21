/*
 * Copyright (C) 2014 Walkman
 * Author: Alessio Rocchi
 * email:  alessio.rocchi@iit.it
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

#ifndef __BOUNDS_AGGREGATED_H__
#define __BOUNDS_AGGREGATED_H__

#include <OpenSoT/Constraint.h>
#include <Eigen/Dense>
#include <memory>
#include <OpenSoT/utils/Piler.h>
#include <list>

using namespace OpenSoT::utils;

 namespace OpenSoT {
    namespace constraints {

        /**
         * @brief The Aggregated class builds a new Constraing by piling up simpler constraints
         *        so that:
         * * For equality constraints:
         *   Aeq = [Aeq1; Aeq2], beq=[beq1;beq2]
         * * For inequality constraints:
         *   Aineq = [Aineq1; Aeq2],
         *   bLowerBound=[bLowerBound1;bLowerBound2]
         *   bUpperBound=[bUpperBound1;bUpperBound2]
         * * For bounds:
         *   lowerBound = max(bLowerBound1, bLowerBound2),
         *   upperBound = min(bUpperBound1,bUpperBound2)
         */
        class Aggregated: public Constraint<Eigen::MatrixXd, Eigen::VectorXd> {
        public:
	    typedef std::shared_ptr<Aggregated> Ptr;
        typedef MatrixPiler VectorPiler;

            enum AggregationPolicy {
                /** transform equalities Ax = b to inequalities b <= Ax <= b */
                EQUALITIES_TO_INEQUALITIES = 0x001,
                /** if enabled, unilateral bounds will be converted to unilateral:
                 *      x <= u becomes -inf <= x <= u, l <= x becomes l <= x <= inf
                 *  if not enabled, bilateral bounds will be converted to unilateral:
                 *      l <= x <= u becomes x <= u && -x <= -l
                 */
                UNILATERAL_TO_BILATERAL = 0x100
            };

        protected:
            std::vector<Eigen::VectorXd> _boundUpperBounds, _boundLowerBounds, _boundbeqs, _boundbUpperBounds, _boundbLowerBounds;
            std::vector<Eigen::MatrixXd> _boundAeqs, _boundAineqs;
            void initSupportVectorsAndMatrices(const int bound_size)
            {
                for(unsigned int i = 0; i < bound_size; ++i)
                {
                    _boundUpperBounds.push_back(Eigen::VectorXd(0));
                    _boundLowerBounds.push_back(Eigen::VectorXd(0));
                    _boundbeqs.push_back(Eigen::VectorXd(0));
                    _boundbUpperBounds.push_back(Eigen::VectorXd(0));
                    _boundbLowerBounds.push_back(Eigen::VectorXd(0));

                    _boundAeqs.push_back(Eigen::MatrixXd(0,0));
                    _boundAineqs.push_back(Eigen::MatrixXd(0,0));
                }
            }

            VectorPiler _tmpupperBound;
            VectorPiler _tmplowerBound;

            MatrixPiler _tmpAeq;
            VectorPiler _tmpbeq;

            MatrixPiler _tmpAineq;
            VectorPiler _tmpbUpperBound;
            VectorPiler _tmpbLowerBound;

            std::list< ConstraintPtr > _bounds;
            unsigned int _number_of_bounds;
            unsigned int _aggregationPolicy;

            void checkSizes();

            static const std::string concatenateConstraintsIds(const std::list<ConstraintPtr> constraints);

            static const std::string _CONSTRAINT_PLUS_;
            static const std::string _CONSTRAINT_AGGREGATED_;
            static std::string concatenatedId;


            inline void pile(Eigen::MatrixXd& A, const Eigen::MatrixXd& B)
            {
                A.conservativeResize(A.rows()+B.rows(), A.cols());
                A.block(A.rows()-B.rows(),0,B.rows(),A.cols())<<B;
            }

            inline void pile(Eigen::VectorXd&a, const Eigen::VectorXd&b)
            {
                a.conservativeResize(a.rows()+b.rows());
                a.segment(a.rows()-b.rows(),b.rows())<<b;
            }


            virtual void _log(XBot::MatLogger2::Ptr logger);

        public:

            /**
             * @brief Aggregated
             * @param bounds a std::list of Bounds
             * @param x_size the size of the x vector. Notice this constructor will NOT call
             *               update() on the base tasks
             */
            Aggregated(const std::list<ConstraintPtr> constraints,
                       const unsigned int x_size,
                       const unsigned int aggregationPolicy =
                            EQUALITIES_TO_INEQUALITIES |
                            UNILATERAL_TO_BILATERAL);

            /**
             * @brief Aggregated
             * @param bound1 pointer to the first bound
             * @param bound2 pointer to the second bound
             * @param x_size the size of the x vector. Notice this constructor will NOT call
             *               update() on the base tasks
             */
            Aggregated(ConstraintPtr bound1,
                       ConstraintPtr bound2,
                       const unsigned int &x_size,
                       const unsigned int aggregationPolicy =
                            EQUALITIES_TO_INEQUALITIES |
                            UNILATERAL_TO_BILATERAL);

            void update(const Eigen::VectorXd &x);

            void log(XBot::MatLogger2::Ptr logger) override;

            std::list< ConstraintPtr >& getConstraintsList() { return _bounds; }

            void generateAll();
        };
    }
 }

#endif

/*
 * Copyright (C) 2014 Walkman
 * Author: Alessio Rocchi, Enrico Mingo
 * email:  alessio.rocchi@iit.it, enrico.mingo@iit.it
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

#ifndef __OPENSOT_H__
#define __OPENSOT_H__

 #include "Constraint.h"
 #include "constraints/Aggregated.h"
 #include "constraints/BilateralConstraint.h"
 #include "constraints/velocity/all.h"
 #include "Solver.h"
 #include "solvers/QPOases.h"
 #include "Task.h"
 #include "tasks/Aggregated.h"
 #include "tasks/velocity/all.h"

namespace OpenSoT {

    class Factory {
    private:

        template <class Matrix_type, class Vector_type>
        class Stack {
            friend class OpenSoT::Factory;
            typename OpenSoT::Solver<Matrix_type, Vector_type>::Stack stack;
            virtual void update(const Vector_type& q);
            operator typename OpenSoT::Solver<Matrix_type, Vector_type>::Stack();
            virtual ~Stack() {}
        protected:
            Stack();
            Stack(typename OpenSoT::Solver<Matrix_type, Vector_type>::Stack stack);
        };

        Factory();

    public:
        ~Factory() {}

        class DefaultHumanoidStack : public Stack<yarp::sig::Matrix, yarp::sig::Vector>
        {
        private:
            iDynUtils model;
            iDynUtils model_com;

            DefaultHumanoidStack(std::string srdfModelFileName,
                                 const double dT);

        public:
            ~DefaultHumanoidStack() {}
            friend class OpenSoT::Factory;
            typedef boost::shared_ptr<OpenSoT::Factory::DefaultHumanoidStack> Ptr;

            // tasks
            const tasks::velocity::Cartesian::Ptr leftArm;
            const tasks::velocity::Cartesian::Ptr rightArm;
            const tasks::velocity::Cartesian::Ptr waist2LeftArm;
            const tasks::velocity::Cartesian::Ptr waist2RightArm;
            const tasks::velocity::Cartesian::Ptr leftLeg;
            const tasks::velocity::Cartesian::Ptr rightLeg;
            const tasks::velocity::Cartesian::Ptr rightLeg2LeftLeg;
            const tasks::velocity::Cartesian::Ptr leftLeg2RightLeg;
            const tasks::velocity::CoM::Ptr com;
            const tasks::velocity::MinimumEffort::Ptr minimumEffort;
            const tasks::velocity::Postural::Ptr postural;

            // constraints
            const constraints::velocity::CoMVelocity::Ptr comVelocity;
            const constraints::velocity::ConvexHull::Ptr convexHull;
            const constraints::velocity::JointLimits::Ptr jointLimits;
            const constraints::velocity::VelocityLimits::Ptr velocityLimits;

            void update(const yarp::sig::Vector& q);
        };

        DefaultHumanoidStack::Ptr getDefaultHumanoidStack(std::string srdfModelFileName,
                                                          const double dT);
    private:
        static DefaultHumanoidStack::Ptr defaultHumanoidStack;
    };
}
#endif

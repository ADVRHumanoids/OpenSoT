/*
 * Copyright (C) 2019 Cogimon/Centauro
 * Authors: Enrico Mingo Hoffman
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

#ifndef __TASKS_FORCE_H__
#define __TASKS_FORCE_H__

#include <OpenSoT/Task.h>
#include <OpenSoT/utils/Affine.h>
#include <OpenSoT/tasks/MinimizeVariable.h>
#include <OpenSoT/tasks/Aggregated.h>
#include <memory>
#include <OpenSoT/utils/AutoStack.h>

namespace OpenSoT {
   namespace tasks {
       namespace force {

       /**
         * @brief The Wrench class implements a task which minimize the error between a desired force and
         * optimized force.
         */
        class Wrench : public Task < Eigen::MatrixXd, Eigen::VectorXd > {
        public:
            typedef std::shared_ptr<Wrench> Ptr;

            /**
             * @brief Wrench
             * @param id
             * @param distal_link where the force is applied
             * @param base_link wrt the force is expressed (not used)
             * @param wrench
             *
             * NOTE: base_link is not used but is only needed to retrieve information for other computations
             * such as Inverse Dynamics (J'F)
             */
            Wrench(const std::string& id,
                   const std::string& distal_link, const std::string& base_link,
                   const AffineHelper& wrench);

            bool setReference(const Eigen::VectorXd& ref);
            void getReference(Eigen::VectorXd& ref);

            const std::string& getDistalLink() const;
            const std::string& getBaseLink() const;
        protected:
            virtual void _update(const Eigen::VectorXd& x);    
        private:
            std::string _distal_link, _base_link;
            OpenSoT::tasks::MinimizeVariable::Ptr _min_var;

            Eigen::VectorXd _tmp;

        };

        class Wrenches : public Task < Eigen::MatrixXd, Eigen::VectorXd > {
        public:
            typedef std::shared_ptr<Wrenches> Ptr;

            Wrenches(const std::string& id,
                     const std::vector<std::string>& distal_links,
                     const std::vector<std::string>& base_links,
                     const std::vector<AffineHelper>& wrenches);

            Wrench::Ptr getWrenchTask(const std::string& distal_link);

        private:
            std::map<std::string, Wrench::Ptr> wrench_tasks;
            OpenSoT::tasks::Aggregated::Ptr _aggregated_task;
            virtual void _update(const Eigen::VectorXd& x);


        };

       }
   }
}

#endif

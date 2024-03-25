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

#ifndef __SUBTASK_H__
#define __SUBTASK_H__

 #include <OpenSoT/Task.h>
 #include <OpenSoT/utils/Indices.h>
 #include <Eigen/Dense>
 #include <list>
 #include <vector>
 #include <string>
 #include <cassert>
 #include <memory>
 #include <iterator>

 namespace OpenSoT {

    /**
     * @brief SubTask represents a task which is obtained as a sub task
     * in the form \f$T(A_s,b_s)\f$,
     * where \f$A_s\f$ is a reduced task error jacobian
     * and \f$b_s\f$ its corresponding task error.
     * Updating a SubTask calls the update method for the corresponding father
     * task which it reduces, and recreates the reduced A,b and Weight matrices.
     * In the same way, the constraints of the SubTask are those of the father Task,
     * as well as the weight matrix W (the \f$W_\text{subtask}\f$ is a submatrix of \f$W\f$)
     * On the other side, the \f$\lambda\f$ for the SubTask is unique to the SubTask.
    */
    class SubTask : public Task<Eigen::MatrixXd, Eigen::VectorXd> {

    public:

        typedef std::shared_ptr<OpenSoT::SubTask> Ptr;

    protected:
        TaskPtr _taskPtr;
        Indices _subTaskMap;

        virtual void _log(XBot::MatLogger2::Ptr logger);

        void generateA();

        void generateHessianAtype();

        void generateb();

        void generateWeight();

        /** Updates the A, b, Aeq, beq, Aineq, b*Bound matrices
            @param x variable state at the current step (input) */
        virtual void _update();

        static const std::string _SUBTASK_SEPARATION_;

        Eigen::MatrixXd fullW;

    public:
        /**
         * @brief SubTask create a SubTask object by specifying the father Task through a pointer,
         * and a list of row indices. Notice the row indices start from 0 (c style)
         * @param taskPtr a pointer to the father task
         * @param rowIndices a list of indices. The index to the first row is 0.
         */
        SubTask(TaskPtr taskPtr, const std::list<unsigned int> rowIndices);

        virtual ~SubTask(){}

        /**
         * @brief setWeight sets the task weight.
         * Note the Weight needs to be positive definite.
         * If your original intent was to get a subtask
         * (i.e., reduce the number of rows of the task Jacobian),
         * please use the class SubTask
         * @param W matrix weight
         */
        virtual void setWeight(const Eigen::MatrixXd& W);

        /**
         * @brief getConstraints return a reference to the constraint list. Use the standard list methods
         * to add, remove, clear, ... the constraints list.
         * e.g.:
         *              task.getConstraints().push_back(new_constraint)
         * Notice that in subtasks, you will get the constraint list of the father Task from which the SubTask
         * is generated.
         * @return the list of constraints to which the father Task is subject.
         */
        virtual std::list< ConstraintPtr >& getConstraints();

        /** Gets the task size.
            @return the number of rows of A */
        virtual const unsigned int getTaskSize() const;


        /**
         * @brief getActiveJointsMask return a vector of length NumberOfDOFs.
         * If an element is false the corresponding column of the task jacobian is set to 0.
         * @return a vector of bool
         */
        virtual std::vector<bool> getActiveJointsMask();

        /**
         * @brief setActiveJointsMask set a mask on the jacobian
         * @param active_joints_mask
         * @return true if success
         */
        virtual bool setActiveJointsMask(const std::vector<bool>& active_joints_mask);

        /**
         * @brief isSubTask check wether a generic task is a subtask
         * @param task pointer to generic task
         * @return true if is a subtask
         */
        static bool isSubTask(OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr task){
            return (bool)std::dynamic_pointer_cast<OpenSoT::SubTask>(task);
        }

        /**
         * @brief asSubTask cast generic task to a subtask
         * @param task pointer to generic task
         * @return pointer to subtask casted from given task
         */
        static OpenSoT::SubTask::Ptr asSubTask(OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr task){
            return std::dynamic_pointer_cast<OpenSoT::SubTask>(task);
        }

        /**
         * @brief getTask return the internal pointer of the task used to create the subtask
         * @return internal pointer to task
         */
        TaskPtr getTask() {return _taskPtr;}

    };


 }

#endif

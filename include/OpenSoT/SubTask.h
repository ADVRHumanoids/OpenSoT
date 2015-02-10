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

 #include <list>
 #include <vector>
 #include <string>
 #include <cassert>
 #include <boost/shared_ptr.hpp>
 #include <yarp/sig/all.h>
 #include <yarp/math/Math.h>
 #include <yarp/math/SVD.h>
 #include <sstream>
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
    class SubTask : public Task<yarp::sig::Matrix, yarp::sig::Vector> {

    public:

        typedef boost::shared_ptr<OpenSoT::SubTask> Ptr;

        class SubTaskMap {
        public:
            /**
             * @brief RowsChunk a vector<unsigned int> of rows which are contiguous
             */
            typedef std::vector<unsigned int> RowsChunk;
            /**
             * @brief ChunkList a list<vector<int>>, a list of contiguous chunks of rows
             * e.g., {{1,2},{4},{10,11,12}}, is a list of 3 chunks of contiguous row indices,
             * where the first chunk has size 2, the second chunk has size 1, the third chunk has size 3
             */
            typedef std::list< RowsChunk > ChunkList;
        private:

            ChunkList _contiguousChunks;
            std::list<unsigned int> _rowsList;
            std::vector<unsigned int> _rowsVector;

            /**
             * @brief getNextAdjacentChunk returns an iterator pointing to the last element of a chunk
             * @param searchBegin iterator pointing to the first element where we start looking for chunks
             * @return the iterator pointing to the last element of a chunk
             */
            std::list<unsigned int>::iterator getNextAdjacentChunk(std::list<unsigned int>::iterator searchBegin);

            void generateChunks();

        public:
            /**
             * @brief SubTaskMap creates a SubTaskMap of size 1, with just one index
             * @param i
             */
            SubTaskMap(unsigned int i);

            SubTaskMap(const std::list<unsigned int>& rowsList);

            SubTaskMap(const std::vector<unsigned int> &rowsVector);

            template<class Iterator>
            SubTaskMap(Iterator it, const Iterator end) {
                while( it != end)
                {
                    _rowsList.push_back(*it);
                    ++it;
                }
            }

            SubTaskMap(const SubTaskMap& subTaskMap);

            const ChunkList &getChunks() const;

            /**
             * @brief getRowsList returns the list of all rows as a list (first row has index 0)
             * @return a list of row indices (starting from 0)
             */
            const std::list<unsigned int> &getRowsList() const;

            /**
             * @brief getRowsVector returns the list of all rows as a vector (first row has index 0)
             * @return  a vector of row indices (starting from 0)
             */
            const std::vector<unsigned int> &getRowsVector() const;

            bool isContiguous() const;

            int size() const;

            static SubTaskMap range(unsigned int from, unsigned int to);

            SubTaskMap operator+(const SubTaskMap& b) const;

            SubTaskMap operator+(const unsigned int r) const;

            bool operator==(const SubTaskMap& b) const;

            operator std::string() const;

            operator std::list<unsigned int>() const;
        };

    private:
        TaskPtr _taskPtr;
        SubTaskMap _subTaskMap;

    protected:

    public:
        /**
         * @brief SubTask create a SubTask object by specifying the father Task through a pointer,
         * and a list of row indices. Notice the row indices start from 0 (c style)
         * @param taskPtr a pointer to the father task
         * @param rowIndices a list of indices. The index to the first row is 0.
         */
        SubTask(TaskPtr taskPtr, const std::list<unsigned int> rowIndices);

        virtual ~SubTask(){}

        void generateA();

        void generateHessianAtype();

        void generateb();

        void generateWeight();

        /**
         * @brief setWeight sets the task weight.
         * Note the Weight needs to be positive definite.
         * If your original intent was to get a subtask
         * (i.e., reduce the number of rows of the task Jacobian),
         * please use the class SubTask
         * @param W matrix weight
         */
        virtual void setWeight(const yarp::sig::Matrix& W);

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

        /** Updates the A, b, Aeq, beq, Aineq, b*Bound matrices
            @param x variable state at the current step (input) */
        virtual void _update(const yarp::sig::Vector &x);

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
    };


 }

#endif

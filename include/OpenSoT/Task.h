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

#ifndef __TASK_H__
#define __TASK_H__

 #include <list>
 #include <string>
 #include <vector>
 #include <OpenSoT/Constraint.h>
 #include <assert.h>
 #include <boost/shared_ptr.hpp>
 #include <XBotInterface/Logger.hpp>

 namespace OpenSoT {

    /** Summarises all possible types of the QP's Hessian matrix. From qpOASES/Types.hpp */
    enum HessianType
    {
        HST_ZERO,                   /**< Hessian is zero matrix (i.e. LP formulation). */
        HST_IDENTITY,               /**< Hessian is identity matrix. */
        HST_POSDEF,                 /**< Hessian is (strictly) positive definite. */
        HST_POSDEF_NULLSPACE,       /**< Hessian is positive definite on null space of active bounds/constraints. */
        HST_SEMIDEF,                /**< Hessian is positive semi-definite. */
        HST_UNKNOWN                 /**< Hessian type is unknown. */
    };

    /**
     * @brief Task represents a task in the form \f$T(A,b)\f$ where \f$A\f$ is the task error jacobian and \f$b\f$ is the task error
    */
    template <class Matrix_type, class Vector_type>
    class Task {

    public:
        typedef Task< Matrix_type, Vector_type > TaskType;
        typedef boost::shared_ptr<TaskType> TaskPtr;
        typedef Constraint< Matrix_type, Vector_type > ConstraintType;
        typedef boost::shared_ptr<ConstraintType> ConstraintPtr;
    protected:

        /**
         * @brief _task_id unique name of the task
         */
        std::string _task_id;

        /**
         * @brief _x_size size of the controlled variables
         */
        unsigned int _x_size;

        /**
         * @brief _hessianType Type of Hessian associated to the Task
         */
        HessianType _hessianType;

        /**
         * @brief _A Jacobian of the Task
         */
        Matrix_type _A;

        /**
         * @brief _b error associated to the Task
         */
        Vector_type _b;

        /**
         * @brief _W Weight multiplied to the task Jacobian
         */
        Matrix_type _W;

        /**
         * @brief _lambda error scaling,
         * NOTE:
         *          _lambda >= 0.0
         */
        double _lambda;

        /**
         * @brief _bounds related to the Task
         */
        std::list< ConstraintPtr > _constraints;

        /**
         * @brief _active_joint_mask is vector of bool that represent the active joints of the task.
         * If false the corresponding column of the task jacobian is set to 0.
         */
        std::vector<bool> _active_joints_mask;

        /** Updates the A, b, Aeq, beq, Aineq, b*Bound matrices
            @param x variable state at the current step (input) */
        virtual void _update(const Vector_type &x) = 0;

        struct istrue //predicate
        {
           bool operator()(int val) const {return val == true;}
        };

        /**
         * @brief applyActiveJointsMask apply the active joint mask to the A matrix
         * @param A matrix of the Task
         */
        void applyActiveJointsMask(Matrix_type& A)
        {
            int rows = A.rows();
            for(unsigned int i = 0; i < _x_size; ++i)
            {
                if(!_active_joints_mask[i])
                    for(unsigned int j = 0; j < rows; ++j)
                        A(j,i) = 0.0;
            }
            //TODO: is necessary here to call update()?
        }

        /**
         * @brief _log can be used to log internal Task variables
         * @param logger a shared pointer to a MatLogger
         */
        virtual void _log(XBot::MatLogger::Ptr logger)
        {

        }

    private:

        /**
         * @brief _WA Jacobian of the Task times the Weight
         */
        mutable Matrix_type _WA;

        /**
         * @brief _Wb error associated to the Task times the Weight
         */
        mutable Vector_type _Wb;

    public:
        /**
         * @brief Task define a task in terms of Ax = b
         * @param task_id is a unique id
         * @param x_size is the number of variables of the task
         */
        Task(const std::string task_id,
             const unsigned int x_size) :
            _task_id(task_id), _x_size(x_size), _active_joints_mask(x_size)
        {
            _lambda = 1.0;
            _hessianType = HST_UNKNOWN;
            for(unsigned int i = 0; i < x_size; ++i)
                _active_joints_mask[i] = true;
        }

        virtual ~Task(){}

        /**
         * @brief getA
         * @return the A matrix of the task
         */
        const Matrix_type& getA() const {
            return _A;
        }

        /**
         * @brief getHessianAtype
         * @return the Hessian type
         */
        const HessianType getHessianAtype() { return _hessianType; }

        /**
         * @brief getb
         * @return the b matrix of the task
         */
        const Vector_type& getb() const { return _b; }

        /**
         * @brief getWA
         * @return the product between W and A
         */
        const Matrix_type& getWA() const {
            _WA = _W*_A;
            return _WA;
        }

        /**
         * @brief getWb
         * @return the product between W and b
         */
        const Vector_type& getWb() const {
            _Wb = _W*_b;
            return _Wb;
        }

        /**
         * @brief getWeight
         * @return the weight of the norm of the task error
         */
        const Matrix_type& getWeight() const { return _W; }

        /**
         * @brief setWeight sets the task weight.
         * Note the Weight needs to be positive definite.
         * If your original intent was to get a subtask
         * (i.e., reduce the number of rows of the task Jacobian),
         * please use the class SubTask
         * @param W matrix weight
         */
        virtual void setWeight(const Matrix_type& W) {
            assert(W.rows() == this->getTaskSize());
            assert(W.cols() == W.rows());
            _W = W;
        }

        /**
         * @brief getLambda
         * @return the lambda weight of the task
         */
        const double getLambda() const { return _lambda; }

        virtual void setLambda(double lambda)
        {
            if(lambda >= 0.0){
                _lambda = lambda;
            }
        }
        
        /**
         * @brief getConstraints return a reference to the constraint list. Use the standard list methods
         * to add, remove, clear, ... the constraints list.
         * e.g.:
         *              task.getConstraints().push_back(new_constraint)
         * @return
         */
        virtual std::list< ConstraintPtr >& getConstraints() { return _constraints; }

        /** Gets the number of variables for the task.
            @return the number of columns of A */
        const unsigned int getXSize() const { return _x_size; }

        /** Gets the task size.
            @return the number of rows of A */
        virtual const unsigned int getTaskSize() const { return _A.rows(); }

        /** Updates the A, b, Aeq, beq, Aineq, b*Bound matrices 
            @param x variable state at the current step (input) */
        void update(const Vector_type &x) {
            for(typename std::list< ConstraintPtr >::iterator i = this->getConstraints().begin();
                i != this->getConstraints().end(); ++i) (*i)->update(x);
            this->_update(x);

            typedef std::vector<bool>::const_iterator it_m;
            bool all_true = true;
            for( it_m active_joint = _active_joints_mask.begin();
                 active_joint != _active_joints_mask.end();
                 ++active_joint)
            {
                if(*active_joint == false) all_true = false;
            }

            if(!all_true) applyActiveJointsMask(_A);
        }

        /**
         * @brief getTaskID return the task id
         * @return a string with the task id
         */
        std::string getTaskID(){ return _task_id; }

        /**
         * @brief getActiveJointsMask return a vector of length NumberOfDOFs.
         * If an element is false the corresponding column of the task jacobian is set to 0.
         * @return a vector of bool
         */
        virtual std::vector<bool> getActiveJointsMask(){return _active_joints_mask;}

        /**
         * @brief setActiveJointsMask set a mask on the Jacobian. The changes take effect immediately.
         * @param active_joints_mask
         * @return true if success
         */
        virtual bool setActiveJointsMask(const std::vector<bool>& active_joints_mask)
        {
            if(active_joints_mask.size() == _active_joints_mask.size())
            {
                _active_joints_mask = active_joints_mask;

                applyActiveJointsMask(_A);

                return true;
            }
            return false;
        }

        /**
         * @brief log logs common Task internal variables
         * @param logger a shared pointer to a MathLogger
         */
        virtual void log(XBot::MatLogger::Ptr logger)
        {
            logger->add(_task_id + "_A", _A);
            logger->add(_task_id + "_b", _b);
            logger->add(_task_id + "_W", _W);
            logger->add(_task_id + "_lambda", _lambda);
            _log(logger);

            for(auto constraint : _constraints)
                constraint->log(logger);

        }
    };


 }

#endif

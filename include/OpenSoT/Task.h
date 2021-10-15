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
 #include <memory>
 #include <XBotInterface/Logger.hpp>
 #include <XBotInterface/ModelInterface.h>

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
     * @brief Task represents a task in the form \f$T(A,b,c)\f$ where \f$A\f$ is the task error jacobian, \f$b\f$ is the task error
     * and \f$c\f$ is used for LP
    */
    template <class Matrix_type, class Vector_type>
    class Task {

    public:
        typedef Task< Matrix_type, Vector_type > TaskType;
        typedef std::shared_ptr<TaskType> TaskPtr;
        typedef Constraint< Matrix_type, Vector_type > ConstraintType;
        typedef std::shared_ptr<ConstraintType> ConstraintPtr;
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
         * @brief _c vector used for LP Tasks
         */
        Vector_type _c;

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
         * @brief _weight_is_diagonal, if true the computation of W*A and W*b is optimized (default is false)
         */
        bool _weight_is_diagonal;

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
         * @brief applyActiveJointsMask apply the active joint mask to the A matrix:
         * in tasks in which b does not depend on A, this is done setting to 0 the columns
         * of A corresponding to the index set to false of the _active_joint_mask vector
         * @param A matrix of the Task
         */
        virtual void applyActiveJointsMask(Matrix_type& A)
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
        virtual void _log(XBot::MatLogger2::Ptr logger)
        {

        }

        /**
         * @brief reset permits to reset a task and all related variables. The correctness of the implementation depends to the
         * particular task. Default implementation does nothing and return false.
         * @return false
         */
        virtual bool reset()
        {
            XBot::Logger::error("reset is not implemented for task %s \n", _task_id.c_str());
            return false;
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

        /**
         * @brief _Atranspose Jacobian of the task transposed
         */
        mutable Matrix_type _Atranspose;
        
        /**
         * @brief ...
         * 
         */
        bool _is_active;
        
        /**
         * @brief ...
         * 
         */
        Matrix_type _A_last_active;

        Vector_type _error_, _tmp_, _residual_;


    public:
        /**
         * @brief Task define a task in terms of Ax = b
         * @param task_id is a unique id
         * @param x_size is the number of variables of the task
         */
        Task(const std::string task_id,
             const unsigned int x_size) :
            _task_id(task_id), _x_size(x_size), _active_joints_mask(x_size), _is_active(true), _weight_is_diagonal(false)
        {
            //Eigen:
            _A.setZero(0,x_size);
            _b.setZero(0);
            _c.setZero(x_size);
            //

            _lambda = 1.0;
            _hessianType = HST_UNKNOWN;
            for(unsigned int i = 0; i < x_size; ++i)
                _active_joints_mask[i] = true;
        }

        virtual ~Task(){}

        /**
         * @brief getWeightIsDiagonal return the flag _weight_is_diagonal
         * @return true or false
         */
        bool getWeightIsDiagonalFlag(){return _weight_is_diagonal;}

        /**
         * @brief setWeightIsDiagonalFlag set the flag _weight_is_diagonal (NOTE that no check on Weight matrix is performed, we trust you)
         * @param flag true or false
         */
        void setWeightIsDiagonalFlag(const bool flag){
            _weight_is_diagonal = flag;}

        /**
         * @brief Activated / deactivates the task by setting the A matrix to zero.
         * Important note: after activating a task, call the update() function in 
         * order to recompute a proper A matrix.
         */
        void setActive(const bool active_flag){
            
            if(!_is_active && active_flag && _A_last_active.rows() > 0){
                _A = _A_last_active;
            }
            
            _is_active = active_flag;
        }
        
        
        /**
         * @brief Returns a boolean which specifies if the task is active.
         */
        bool isActive() const {
            return _is_active;
        }
        
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
            if(_weight_is_diagonal)
                _WA.noalias() = _W.diagonal().asDiagonal()*_A;
            else
                _WA.noalias() = _W*_A;
            return _WA;
        }

        /**
         * @brief getATranspose()
         * @return A transposed
         */
        const Matrix_type& getATranspose() const {
            _Atranspose = _A.transpose(); //This brakes the use of the template!
            return _Atranspose;
        }

        /**
         * @brief getWb
         * @return the product between W and b
         */
        const Vector_type& getWb() const {
            if(_weight_is_diagonal)
                _Wb.noalias() = _W.diagonal().asDiagonal()*_b;
            else
                _Wb.noalias() = _W*_b;
            return _Wb;
        }

        /**
         * @brief getc
         * @return the _c vector of the task
         */
        const Vector_type& getc() const {
            return _c;
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
         * @brief setWeight sets the task diagonal weight.
         * Note the Weight needs to be positive definite.
         * If your original intent was to get a subtask
         * (i.e., reduce the number of rows of the task Jacobian),
         * please use the class SubTask
         * @param w scalar diagonal weight
         */
        virtual void setWeight(const double& w) {
            assert(w>=0.0);
            _W.setIdentity();
            _W = _W * w;
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
            
            if(!_is_active){
                _A_last_active = _A;
                _A.setZero(_A.rows(), _A.cols());
                return;
            }

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
         * @brief setActiveChainsMask set a mask (of true) on the Jacobian of a vector of kinematic chains
         * @param active_chain_mask vector of kinematic chains to set the active joint mask
         * @param model to retrieve the joint names from the kinematic chains
         * @return true
         */
        virtual bool setActiveChainsMask(const std::vector<std::string>& active_chain_mask, 
                                         XBot::ModelInterface::ConstPtr model)
        {
            _active_joints_mask.assign(_active_joints_mask.size(), false);
            
            for(const auto& ch : active_chain_mask){
                
                for(const auto& jid : model->chain(ch).getJointIds())
                {
                    _active_joints_mask[ model->getDofIndex(jid) ] = true;
                }
            }
            
            return true;
            
        }

        /**
         * @brief log logs common Task internal variables
         * @param logger a shared pointer to a MathLogger
         */
        virtual void log(XBot::MatLogger2::Ptr logger)
        {
            if(_A.rows() > 0)
                logger->add(_task_id + "_A", _A);
            if(_b.size() > 0)
                logger->add(_task_id + "_b", _b);
            if(_W.rows() > 0)
                logger->add(_task_id + "_W", _W);
            if(_c.size() > 0)
                logger->add(_task_id + "_c", _c);
            logger->add(_task_id + "_lambda", _lambda);
            _log(logger);

            for(auto constraint : _constraints)
                constraint->log(logger);

        }

        /**
         * @brief computeCost computes the residual of the task:
         *
         *  residual = (Ax - b)^T * W * (Ax - b)
         *
         * @param x solution
         * NOTE: the solution should be the one where the task was evaluated to compute the internal A matrix and b vector!
         * NOTE: computation can be improved as done in the solver...
         * @return the cost of the task for given solution
         */
        double computeCost(const Eigen::VectorXd& x)
        {
            _error_.noalias() = _A*x - _b;
            _tmp_.noalias() = _error_.transpose()*_W;
            _residual_.noalias() = _tmp_.transpose()*_error_;
            return _residual_[0];
        }

        /**
         * @brief checkConsistency checks if all internal matrices and vectors are correctly instantiated and the right size
         * @return true if everything is ok
         */
        bool checkConsistency()
        {
            bool a = true;
            //0) Check Weight size is not 0 if b.size > 0!
            if(_b.size() > 0)
            {
                if(_W.rows() == 0){
                    XBot::Logger::error("%s: _W.rows() == %i ! \n", _W.rows());
                    a = false;}
                if(_W.cols() == 0){
                    XBot::Logger::error("%s: _W.cols() == %i ! \n", _W.cols());
                    a = false;}
            }

            //1) Check Weight is square
            if(_W.rows() != _W.cols()){
                XBot::Logger::error("%s: _W.rows() != _W.cols() -> %i != %i! \n", _task_id.c_str(), _W.rows(), _W.cols());
                a = false;
            }

            //2) Check consistency between matrices
            if(_A.rows() != _b.size()){
                XBot::Logger::error("%s: _A.rows() != _b.size() -> %i != %i! \n", _task_id.c_str(), _A.rows(), _b.size());
                a = false;
            }
            if(_A.rows() != _W.rows()){
                XBot::Logger::error("%s: _A.rows() != _W.rows() -> %i != %i! \n", _task_id.c_str(), _A.rows(), _W.rows());
                a = false;
            }

            //3) Check task size
            if(_A.cols() != _x_size){
                XBot::Logger::error("%s: _A.cols() != _x_size -> %i != %i! \n", _task_id.c_str(), _A.cols(), _x_size);
                a = false;
            }

            //4) Check eventually c
            if(_c.size() != _x_size){
                    XBot::Logger::error("%s: _c.size() != _x_size -> %i != %i! \n", _task_id.c_str(), _c.size(), _x_size);
                    a = false;
            }

            //5) If the Hessian Type is ZERO we want to check that all the entries of _A and _b are zeros!
            if(_hessianType == HST_ZERO)
            {
                if(!_A.isZero()){
                    XBot::Logger::error("%s: Hessian is HST_ZERO but _A is not all zeros! \n", _task_id.c_str());
                    a = false;
                }

                if(!_b.isZero()){
                    XBot::Logger::error("%s: Hessian is HST_ZERO but _b is not all zeros! \n", _task_id.c_str());
                    a = false;
                }
            }
            else{
            //6) If the Hessian Type is NOT ZERO we want to check that _A and _b exists!
                if(_A.rows() == 0 || _A.cols() == 0){
                    XBot::Logger::error("%s: _A is [%i x %i]! \n", _task_id.c_str(), _A.rows(), _A.cols());
                    a = false;
                }
                if(_b.size() == 0){
                    XBot::Logger::error("%s: _b size is %i!  \n", _task_id.c_str(), _b.size());
                    a = false;
                }
            }



            if(_constraints.size() > 0)
            {

                for(auto constraint : _constraints)
                {
                    if(!(constraint->checkConsistency()))
                        a = false;
                }
            }

            if(a)
                XBot::Logger::info("%s is consistent!\n", _task_id.c_str());

            return a;

        }

    };


 }

#endif

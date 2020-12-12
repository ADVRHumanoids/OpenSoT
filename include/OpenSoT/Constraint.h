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

#ifndef __CONSTRAINT_H__
#define __CONSTRAINT_H__

#include <boost/shared_ptr.hpp>
#include <string>
#include <XBotInterface/Logger.hpp>

 namespace OpenSoT {

 /**
  * @brief The Constraint class describes all the different types of constraints:
  * 1. bounds & bilateral
  * 2. equalities
  * 3. unilateral
  */
 template <class Matrix_type, class Vector_type>
    class Constraint {
    public:
        typedef Constraint< Matrix_type, Vector_type > ConstraintType;
        typedef boost::shared_ptr<ConstraintType> ConstraintPtr;
    protected:

        /**
         * @brief _constraint_id unique name of the constraint
         */
        std::string _constraint_id;

        /**
         * @brief _x_size size of the controlled variables
         */
        unsigned int _x_size;

        /**
         * @brief _lowerBound lower bounds on controlled variables
         * e.g.:
         *              _lowerBound <= x
         */
        Vector_type _lowerBound;

        /**
         * @brief _upperBound upper bounds on controlled variables
         * e.g.:
         *              x <= _upperBound
         */
        Vector_type _upperBound;

        /**
         * @brief _Aeq Matrix for equality constraint
         * e.g.:
         *              _Aeq*x = _beq
         */
        Matrix_type _Aeq;

        /**
         * @brief _beq constraint vector for equality constraint
         * e.g.:
         *              _Aeq*x = _beq
         */
        Vector_type _beq;

        /**
         * @brief _Aineq Matrix for inequality constraint
         * e.g.:
         *              _bLowerBound <= _Aineq*x <= _bUpperBound
         */
        Matrix_type _Aineq;

        /**
         * @brief _bLowerBound lower bounds in generic inequality constraints
         * e.g.:
         *              _bLowerBound <= _Aineq*x
         */
        Vector_type _bLowerBound;

        /**
         * @brief _bUpperBound upper bounds in generic inequality constraints
         * e.g.:
         *              _Aineq*x <= _bUpperBound
         */
        Vector_type _bUpperBound;

        /**
         * @brief _log can be used to log internal Constraint variables
         * @param logger a shared pointer to a MatLogger
         */
        virtual void _log(XBot::MatLogger::Ptr logger)
        {

        }

    public:
        Constraint(const std::string constraint_id,
                   const unsigned int x_size) :
            _constraint_id(constraint_id), _x_size(x_size) {}
        virtual ~Constraint() {}

        const unsigned int getXSize() { return _x_size; }
        virtual const Vector_type& getLowerBound() { return _lowerBound; }
        virtual const Vector_type& getUpperBound() { return _upperBound; }

        virtual const Matrix_type& getAeq() { return _Aeq; }
        virtual const Vector_type& getbeq() { return _beq; }

        virtual const Matrix_type& getAineq() { return _Aineq; }
        virtual const Vector_type& getbLowerBound() { return _bLowerBound; }
        virtual const Vector_type& getbUpperBound() { return _bUpperBound; }

        /**
         * @brief isEqualityConstraint
         * @return true if Constraint enforces an equality constraint
         */
        virtual bool isEqualityConstraint() { return _Aeq.rows() > 0; }

        /**
         * @brief isEqualityConstraint
         * @return true if Constraint enforces an inequality constraint
         */
        virtual bool isInequalityConstraint() { return _Aineq.rows() > 0; }

        /**
         * @brief isUnilateralConstraint
         * @return true if the Constraint is an unilateral inequality
         */
        virtual bool isUnilateralConstraint() { return isInequalityConstraint() &&
                                                       (_bLowerBound.size() == 0 || _bUpperBound.size() == 0); }
        /**
         * @brief isBilateralConstraint
         * @return true if the Constraint is a bilateral inequality
         */
        virtual bool isBilateralConstraint() { return isInequalityConstraint() && !isUnilateralConstraint(); }

        /**
         * @brief hasBounds checks whether this Constraint contains a bound (or box constraint),
         *                  meaning the constraint matrix $A^T=\left(I \quad \tilde{A}^T\right)$
         * @return true if Constraint if this constraint contains a bound
         */
        virtual bool hasBounds() { return (_upperBound.size() > 0 || _lowerBound.size() > 0); }

        /**
         * @brief isBound checks whether this Constraint is a bound in the form lowerBound <= x <= upperBound,
         *                meaning the constraint matrix $A = I$ is a box constraint
         * @return true if Constraint is a bound
         */
        virtual bool isBound() { return this->hasBounds() &&
                                        !this->isConstraint(); }

        /**
         * @brief isConstraint checks whether this Constraint is a constraint (i.e., it is not a bound)
         * @return true if the Constraint is not a bound
         */
        virtual bool isConstraint() { return this->isEqualityConstraint() ||
                                             this->isInequalityConstraint(); }

        /**
         * @brief getTaskID return the task id
         * @return a string with the task id
         */
        std::string getConstraintID(){ return _constraint_id; }

        /** Updates the A, b, Aeq, beq, Aineq, b*Bound matrices 
            @param x variable state at the current step (input) */
        virtual void update(const Vector_type& x = Vector_type(0)) {}

        /**
         * @brief log logs common Constraint internal variables
         * @param logger a shared pointer to a MathLogger
         */
        virtual void log(XBot::MatLogger::Ptr logger)
        {
            if(_Aeq.rows() > 0 && _Aeq.cols() > 0)
                logger->add(_constraint_id + "_Aeq", _Aeq);
            if(_Aineq.rows() > 0 && _Aineq.cols() > 0)
                logger->add(_constraint_id + "_Aineq", _Aineq);
            if(_beq.size() > 0)
                logger->add(_constraint_id + "_beq", _beq);
            if(_bLowerBound.size() > 0)
                logger->add(_constraint_id + "_bLowerBound", _bLowerBound);
            if(_bUpperBound.size() > 0)
                logger->add(_constraint_id + "_bUpperBound", _bUpperBound);
            if(_upperBound.size() > 0)
                logger->add(_constraint_id + "_upperBound",  _upperBound);
            if(_lowerBound.size() > 0)
                logger->add(_constraint_id + "_lowerBound", _lowerBound);
            _log(logger);
        }

        /**
         * @brief checkConsistency checks if all internal matrices and vectors are correctly instantiated and the right size
         * @return true if everything is ok
         */
        bool checkConsistency()
        {
            bool a = true;
//          //1) If isInequalityConstraint()
            if(isInequalityConstraint())
            {
                if(_Aineq.cols() != _x_size)
                {
                    XBot::Logger::error("%s: _Aineq.cols() != _x_size -> %i != %i", _constraint_id.c_str(), _Aineq.cols(), _x_size);
                    a = false;
                }
                if(isUnilateralConstraint())
                {
                    if(_bUpperBound.size() > 0)
                    {
                        if(_Aineq.rows() != _bUpperBound.size())
                        {
                            XBot::Logger::error("%s: _Aineq.rows() != _bUpperBound.size() -> %i != %i",
                                                _constraint_id.c_str(), _Aineq.rows(), _bUpperBound.size());
                            a = false;
                        }
                    }
                    else
                    {
                        if(_Aineq.rows() != _bLowerBound.size())
                        {
                            XBot::Logger::error("%s: _Aineq.rows() != _bUpperLower.size() -> %i != %i",
                                                _constraint_id.c_str(), _Aineq.rows(), _bLowerBound.size());
                            a = false;
                        }
                    }
                }
                else
                {
                    if(_bUpperBound.size() != _bLowerBound.size()){
                        XBot::Logger::error("%s: __bUpperBound.size() != _bLowerBound.size() -> %i != %i",
                                            _constraint_id.c_str(), _bLowerBound.size(), _bUpperBound.size());
                        a = false;
                    }
                    if(_Aineq.rows() != _bLowerBound.size())
                    {
                        XBot::Logger::error("%s: _Aineq.rows() != _bUpperLower.size() -> %i != %i",
                                            _constraint_id.c_str(), _Aineq.rows(), _bLowerBound.size());
                        a = false;
                    }
                }
                if(isBound())
                {
                    XBot::Logger::error("%s isInequalityConstraint = true and isBound = true at the same time!", _constraint_id.c_str());
                    a = false;
                }
                if(isEqualityConstraint())
                {
                    XBot::Logger::error("%s isInequalityConstraint = true and isEqualityConstraint = true at the same time!", _constraint_id.c_str());
                    a = false;
                }
                if(_beq.size() != 0)
                {
                    XBot::Logger::error("%s: _beq.size() = %i, should be 0!", _constraint_id.c_str(), _beq.size());
                    a = false;
                }
            }
            //2) If isBound()
            else if(isBound())
            {
                if(_lowerBound.size() > 0 && _upperBound.size() > 0)
                {
                    if(_lowerBound.size() != _upperBound.size())
                    {
                        XBot::Logger::error("%s: _lowerBound.size() != _upperBound.size_t() -> %i != %i",
                                            _constraint_id.c_str(), _lowerBound.size(), _upperBound.size());
                        a = false;
                    }
                    if(_lowerBound.size() != _x_size)
                    {
                        XBot::Logger::error("%s: _lowerBound.size() != _x_size -> %i != %i",
                                            _constraint_id.c_str(), _lowerBound.size(), _x_size);
                        a = false;
                    }
                }
                else if(_lowerBound.size() > 0)
                {
                    if(_lowerBound.size() != _x_size)
                    {
                        XBot::Logger::error("%s: _lowerBound.size() != _x_size -> %i != %i",
                                            _constraint_id.c_str(), _lowerBound.size(), _x_size);
                        a = false;
                    }
                    XBot::Logger::warning("%s: _upperBound.size() = 0", _constraint_id.c_str());
                }
                else
                {
                    if(_upperBound.size() != _x_size)
                    {
                        XBot::Logger::error("%s: _upperBound.size() != _x_size -> %i != %i",
                                            _constraint_id.c_str(), _upperBound.size(), _x_size);
                        a = false;
                    }
                    XBot::Logger::warning("%s: _lowerBound.size() = 0", _constraint_id.c_str());
                }
                if(isEqualityConstraint())
                {
                    XBot::Logger::error("%s isBound = true and isEqualityConstraint = true at the same time!", _constraint_id.c_str());
                    a = false;
                }
                if(_beq.size() > 0)
                {
                    XBot::Logger::error("%s: _beq.size() = %i, should be 0!", _constraint_id.c_str(), _beq.size());
                    a = false;
                }
                if(_bUpperBound.size() > 0 || _bLowerBound.size() > 0)
                {
                    XBot::Logger::error("%s: _bLowerBound.size() = %i, _bLowerBound.size() = %i, both should be 0",
                                        _constraint_id.c_str(), _bLowerBound.size(), _bUpperBound.size());
                    a = false;
                }

            }
            //3) If isEqualityConstraint()
            else if(isEqualityConstraint())
            {
                if(_Aeq.rows() != _beq.size())
                {
                    XBot::Logger::error("%s: _Aeq.rows() != _beq.size() -> %i != %i", _constraint_id.c_str(), _Aeq.rows(), _beq.size());
                    a = false;
                }
                if(_Aeq.cols() != _x_size)
                {
                    XBot::Logger::error("%s: _Aeq.cols() != _x_size -> %i != %i", _constraint_id.c_str(), _Aeq.cols(), _x_size);
                    a = false;
                }
                if(_bUpperBound.size() > 0 || _bLowerBound.size() > 0)
                {
                    XBot::Logger::error("%s: _bLowerBound.size() = %i, _bLowerBound.size() = %i, both should be 0",
                                        _constraint_id.c_str(), _bLowerBound.size(), _bUpperBound.size());
                    a = false;
                }
            }
            else
            {
                XBot::Logger::error("%s: isEqualityConstraint() = false, isInequalityConstraint() = false, isBound() = false!");
                a = false;
            }

            return a;
        }
    };
 }

#endif

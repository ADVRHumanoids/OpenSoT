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

#include <OpenSoT/constraints/Aggregated.h>

#include <assert.h>
#include <limits>
#include <sstream>

using namespace OpenSoT::constraints;

const std::string Aggregated::_CONSTRAINT_PLUS_ = "+";
const std::string Aggregated::_CONSTRAINT_AGGREGATED_ = "AGGR_";
std::string Aggregated::concatenatedId = "";


Aggregated::Aggregated(const std::list<ConstraintPtr> bounds,
                       const unsigned int x_size,
                       const unsigned int aggregationPolicy) :
    Constraint(concatenateConstraintsIds(bounds), x_size),
               _bounds(bounds), _aggregationPolicy(aggregationPolicy)
{
    _number_of_bounds = _bounds.size();
    this->checkSizes();
    /* calling update to generate bounds */
    this->generateAll();
}

Aggregated::Aggregated(ConstraintPtr bound1,
                       ConstraintPtr bound2,
                       const unsigned int &x_size,
                       const unsigned int aggregationPolicy) :
    Constraint(bound1->getConstraintID() + _CONSTRAINT_PLUS_ + bound2->getConstraintID(),
               x_size), _aggregationPolicy(aggregationPolicy)
{
    _bounds.push_back(bound1);
    _bounds.push_back(bound2);

    _number_of_bounds = _bounds.size();

    this->checkSizes();
    /* calling update to generate bounds */
    this->generateAll();
}

void Aggregated::update() {
    /* iterating on all bounds.. */
    for(typename std::list< ConstraintPtr >::iterator i = _bounds.begin();
        i != _bounds.end(); i++) {

        ConstraintPtr &b = *i;
        /* update bounds */
        b->update();
    }

    this->generateAll();
}

void Aggregated::generateAll() {
    /* resetting all internal data */
    _tmpupperBound.reset(1);
    _tmplowerBound.reset(1);

    _tmpAeq.reset(_x_size);
    _tmpbeq.reset(1);

    _tmpAineq.reset(_x_size);
    _tmpbUpperBound.reset(1);
    _tmpbLowerBound.reset(1);

    if(_constraint_id.empty() || _number_of_bounds != _bounds.size()){
        _number_of_bounds = _bounds.size();
        _constraint_id = concatenateConstraintsIds(getConstraintsList());}

    if(_number_of_bounds != _boundUpperBounds.size())
        initSupportVectorsAndMatrices(_number_of_bounds);

    /* iterating on all bounds.. */
    unsigned int j = 0;
    for(typename std::list< ConstraintPtr >::iterator i = _bounds.begin(); i != _bounds.end(); i++) {

        ConstraintPtr &b = *i;

        _boundUpperBounds[j] = b->getUpperBound();
        _boundLowerBounds[j] = b->getLowerBound();

        _boundAeqs[j] = b->getAeq();
        _boundbeqs[j] = b->getbeq();

        _boundAineqs[j] = b->getAineq();
        _boundbUpperBounds[j] = b->getbUpperBound();
        _boundbLowerBounds[j] = b->getbLowerBound();

        Eigen::VectorXd& boundUpperBound = _boundUpperBounds[j];
        Eigen::VectorXd& boundLowerBound = _boundLowerBounds[j];

        Eigen::MatrixXd& boundAeq = _boundAeqs[j];
        Eigen::VectorXd& boundbeq = _boundbeqs[j];

        Eigen::MatrixXd& boundAineq = _boundAineqs[j];
        Eigen::VectorXd& boundbUpperBound = _boundbUpperBounds[j];
        Eigen::VectorXd& boundbLowerBound = _boundbLowerBounds[j];


//        Eigen::VectorXd boundUpperBound = b->getUpperBound();
//        Eigen::VectorXd boundLowerBound = b->getLowerBound();

//        Eigen::MatrixXd boundAeq = b->getAeq();
//        Eigen::VectorXd boundbeq = b->getbeq();

//        Eigen::MatrixXd boundAineq = b->getAineq();
//        Eigen::VectorXd boundbUpperBound = b->getbUpperBound();
//        Eigen::VectorXd boundbLowerBound = b->getbLowerBound();

        /* copying lowerBound, upperBound */
        if(boundUpperBound.rows() != 0 ||
           boundLowerBound.rows() != 0) {
            assert(boundUpperBound.rows() == _x_size);
            assert(boundLowerBound.rows() == _x_size);

            if(_tmpupperBound.rows() == 0 ||
               _tmplowerBound.rows() == 0) { // first valid bounds found
                assert(_tmpupperBound.rows() == _tmplowerBound.rows());
                _tmpupperBound.set(boundUpperBound);
                _tmplowerBound.set(boundLowerBound);
            } else {
                for(unsigned int i = 0; i < _x_size; ++i) {
                    // compute the minimum between current and new upper bounds
                    _tmpupperBound[i] = std::min(  _tmpupperBound[i],
                                                boundUpperBound[i]);
                    // compute the maximum between current and new lower bounds
                    _tmplowerBound[i] = std::max(  _tmplowerBound[i],
                                                boundLowerBound[i]);
                }
            }
        }

        /* copying Aeq, beq */
        if( boundAeq.rows() != 0 ||
            boundbeq.rows() != 0) {
            assert(boundAeq.rows() == boundbeq.rows());
            /* when transforming equalities to inequalities,
                Aeq*x = beq becomes
                beq <= Aeq*x <= beq */
            if(_aggregationPolicy & EQUALITIES_TO_INEQUALITIES) {
                assert(_tmpAineq.cols() == boundAeq.cols());
                _tmpAineq.pile(boundAeq);
                _tmpbUpperBound.pile(boundbeq);
                if(_aggregationPolicy & UNILATERAL_TO_BILATERAL) {
                    _tmpbLowerBound.pile(boundbeq);
                /* we want to have only unilateral constraints, so
                   beq <= Aeq*x <= beq becomes
                   -Aeq*x <= -beq && Aeq*x <= beq */
                } else {
                    assert(_tmpAineq.cols() == boundAeq.cols());
                    _tmpAineq.pile(-1.0*boundAeq);
                    _tmpbUpperBound.pile( -1.0 * boundbeq);
                }
            } else {
                assert(_tmpAeq.cols() == boundAeq.cols());
                _tmpAeq.pile(boundAeq);
                _tmpbeq.pile(boundbeq);
            }
        }

        /* copying Aineq, bUpperBound, bLowerBound*/
        if( boundAineq.rows() != 0 ||
            boundbUpperBound.rows() != 0 ||
            boundbLowerBound.rows() != 0) {

            assert(boundAineq.rows() > 0);
            assert(boundbLowerBound.rows() > 0 ||
                   boundbUpperBound.rows() > 0);
            assert(boundAineq.cols() == _x_size);

            /* if we need to transform all unilateral bounds to bilateral.. */
            if(_aggregationPolicy & UNILATERAL_TO_BILATERAL) {
                if(boundbUpperBound.rows() == 0) {
                    assert(boundAineq.rows() == boundbLowerBound.rows());
                    boundbUpperBound.resize(boundAineq.rows());
                    boundbUpperBound<<boundbUpperBound.setOnes(boundAineq.rows())*std::numeric_limits<double>::infinity();
                } else if(boundbLowerBound.rows() == 0) {
                    assert(boundAineq.rows() == boundbUpperBound.rows());
                    boundbLowerBound.resize(boundAineq.rows());
                    boundbLowerBound<<boundbLowerBound.setOnes(boundAineq.rows())*-std::numeric_limits<double>::max();
                } else {
                    assert(boundAineq.rows() == boundbLowerBound.rows());
                    assert(boundAineq.rows() == boundbUpperBound.rows());
                }
            /* if we need to transform all bilateral bounds to unilateral.. */
            } else {
                /* we need to transform l < Ax into -Ax < -l */
                if(boundbUpperBound.rows() == 0) {
                    boundAineq = -1.0 * boundAineq;
                    boundbLowerBound = -1.0 * boundbLowerBound;
                    assert(boundAineq.rows() == boundbLowerBound.rows());
                } else if(boundbLowerBound.rows() == 0) {
                    assert(boundAineq.rows() == boundbUpperBound.rows());
                } else {
                    assert(boundAineq.rows() == boundbLowerBound.rows());
                    assert(boundAineq.rows() == boundbUpperBound.rows());
                    pile(boundAineq,-1.0 * boundAineq);
                    pile(boundbUpperBound,-1.0 * boundbLowerBound);
                }
            }

            assert(_tmpAineq.cols() == boundAineq.cols());
            _tmpAineq.pile(boundAineq);
            _tmpbUpperBound.pile(boundbUpperBound);
            /*  if using UNILATERAL_TO_BILATERAL we always have lower bounds,
                otherwise, we never have them */
            if(_aggregationPolicy & UNILATERAL_TO_BILATERAL)
                _tmpbLowerBound.pile( boundbLowerBound);
        }
        j += 1;
    }

    /* checking everything went fine */
    assert(_tmplowerBound.rows() == 0 || _tmplowerBound.rows() == _x_size);
    assert(_tmpupperBound.rows() == 0 || _tmpupperBound.rows() == _x_size);

    assert(_tmpAeq.rows() == _tmpbeq.rows());
    if(_tmpAeq.rows() > 0)
        assert(_tmpAeq.cols() == _x_size);

    assert(_tmpAineq.rows() == _tmpbUpperBound.rows());
    if(!(_aggregationPolicy & UNILATERAL_TO_BILATERAL))
        assert(_tmpAineq.rows() == _tmpbLowerBound.rows());
    if(_tmpAineq.rows() > 0)
        assert(_tmpAineq.cols() == _x_size);


    _upperBound = _tmpupperBound.generate_and_get();
    _lowerBound = _tmplowerBound.generate_and_get();

    _Aeq = _tmpAeq.generate_and_get();
    _beq = _tmpbeq.generate_and_get();

    _Aineq = _tmpAineq.generate_and_get();
    _bUpperBound = _tmpbUpperBound.generate_and_get();
    _bLowerBound = _tmpbLowerBound.generate_and_get();

}

void Aggregated::checkSizes()
{
    for(std::list< ConstraintPtr >::iterator i = _bounds.begin();
        i != _bounds.end(); ++i) {
        ConstraintPtr t = *i;
        assert(this->getXSize() == t->getXSize());
    }
}

const std::string Aggregated::concatenateConstraintsIds(const std::list<ConstraintPtr> constraints) {
    concatenatedId = "";
    if(constraints.size() > 0)
    {
        concatenatedId = _CONSTRAINT_AGGREGATED_;
        int constraintSize = constraints.size();
        for(std::list<ConstraintPtr>::const_iterator i = constraints.begin(); i != constraints.end(); ++i) {
            concatenatedId += (*i)->getConstraintID();
            if(--constraintSize > 0)
                concatenatedId += _CONSTRAINT_PLUS_;
        }
    }

    return concatenatedId;
}

void Aggregated::log(XBot::MatLogger2::Ptr logger)
{
#ifdef OPENSOT_VERBOSE_MATLOG
    Constraint::log(logger);
#else
    for(auto bound : _bounds)
        bound->log(logger);
#endif
}

void Aggregated::_log(XBot::MatLogger2::Ptr logger)
{
    for(auto bound : _bounds)
        bound->log(logger);
}

#include <OpenSoT/SubConstraint.h>

using namespace OpenSoT;

const std::string SubConstraint::_SUBCONSTRAINT_SEPARATION_ = "_";

SubConstraint::SubConstraint(ConstraintPtr constrPtr, const std::list<unsigned int> rowIndices):
    Constraint(constrPtr->getConstraintID() + _SUBCONSTRAINT_SEPARATION_ + std::string(Indices(rowIndices)), constrPtr->getXSize()),
    _subConstraintMap(rowIndices),
    _constraintPtr(constrPtr)
{
    if(constrPtr->isBound()) //1. constraint ptr is a bound
    {
        this->_lowerBound.resize(rowIndices.size());
        this->_upperBound.resize(rowIndices.size());

        generateBounds();
    }
    else if(constrPtr->isInequalityConstraint()) //2. constraint ptr is inequality
    {
        this->_bLowerBound.resize(rowIndices.size());
        this->_bUpperBound.resize(rowIndices.size());
        this->_Aineq.resize(rowIndices.size(), _x_size);
    }
    else //if(constrPtr->isEqualityConstraint()) //3. is equality constraint (NOT USED)
    {
        this->_Aeq.resize(rowIndices.size(), _x_size);
        this->_beq.resize(rowIndices.size());
    }
}

void SubConstraint::update(const Eigen::VectorXd& x)
{
    _constraintPtr->update(x);
    if(_constraintPtr->isBound()) //1. constraint ptr is a bound
        generateBounds();
//    else if(constrPtr->isInequalityConstraint()) //2. constraint ptr is inequality
//    else //if(constrPtr->isEqualityConstraint()) //3. is equality constraint (NOT USED)
}

void SubConstraint::generateBounds()
{
    unsigned int chunk_size = 0;
    unsigned int j = 0;
    for(Indices::ChunkList::const_iterator i = _subConstraintMap.getChunks().begin(); i != _subConstraintMap.getChunks().end();++i)
    {
        chunk_size = i->size();

        this->_lowerBound.segment(j, chunk_size) = _constraintPtr->getLowerBound().segment(i->front(),i->back()-i->front()+1);
        this->_upperBound.segment(j, chunk_size) = _constraintPtr->getUpperBound().segment(i->front(),i->back()-i->front()+1);

        j+=chunk_size;
    }
}

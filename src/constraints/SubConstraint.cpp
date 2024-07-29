#include <OpenSoT/SubConstraint.h>

using namespace OpenSoT;

const std::string SubConstraint::_SUBCONSTRAINT_SEPARATION_ = "_";

SubConstraint::SubConstraint(ConstraintPtr constrPtr, const std::list<unsigned int> rowIndices):
    Constraint(constrPtr->getConstraintID() + _SUBCONSTRAINT_SEPARATION_ + std::string(Indices(rowIndices)), constrPtr->getXSize()),
    _subConstraintMap(rowIndices),
    _constraintPtr(constrPtr)
{
    if(constrPtr->isBound()) //1. constraint ptr is a bound, we transform it into a constraint with less rows
    {
        this->_Aineq.resize(rowIndices.size(), _x_size);
        this->_Aineq.setZero();
        unsigned int i = 0;
        for(auto id : rowIndices){
            this->_Aineq(i, id) = 1.;
            i++;
        }


        this->_bLowerBound.resize(rowIndices.size());
        this->_bUpperBound.resize(rowIndices.size());

        generateBound(this->_constraintPtr->getLowerBound(), this->_bLowerBound);
        generateBound(this->_constraintPtr->getUpperBound(), this->_bUpperBound);
    }
    else if(constrPtr->isInequalityConstraint()) //2. constraint ptr is inequality
    {
        this->_bLowerBound.resize(rowIndices.size());
        this->_bUpperBound.resize(rowIndices.size());
        this->_Aineq.resize(rowIndices.size(), _x_size);

        generateBound(this->_constraintPtr->getbLowerBound(), this->_bLowerBound);
        generateBound(this->_constraintPtr->getbUpperBound(), this->_bUpperBound);
        generateConstraint(this->_constraintPtr->getAineq(), this->_Aineq);
    }
    else //if(constrPtr->isEqualityConstraint()) //3. is equality constraint (NOT USED)
    {
        this->_Aeq.resize(rowIndices.size(), _x_size);
        this->_beq.resize(rowIndices.size());

        generateBound(this->_constraintPtr->getbeq(), this->_beq);
        generateConstraint(this->_constraintPtr->getAeq(), this->_Aeq);
    }
}

void SubConstraint::update()
{
    _constraintPtr->update();
    if(_constraintPtr->isBound()) //1. constraint ptr is a bound, we transform it into a constraint with less rows
    {
        generateBound(this->_constraintPtr->getLowerBound(), this->_bLowerBound);
        generateBound(this->_constraintPtr->getUpperBound(), this->_bUpperBound);
    }
    else if(_constraintPtr->isInequalityConstraint()) //2. constraint ptr is inequality
    {
        generateBound(this->_constraintPtr->getbLowerBound(), this->_bLowerBound);
        generateBound(this->_constraintPtr->getbUpperBound(), this->_bUpperBound);
        generateConstraint(this->_constraintPtr->getAineq(), this->_Aineq);
    }
    else //if(constrPtr->isEqualityConstraint()) //3. is equality constraint (NOT USED)
    {
        generateBound(this->_constraintPtr->getbeq(), this->_beq);
        generateConstraint(this->_constraintPtr->getAeq(), this->_Aeq);
    }
}

void SubConstraint::generateConstraint(const Eigen::MatrixXd& A, Eigen::MatrixXd& sub_A)
{
    unsigned int chunk_size = 0;
    unsigned int j = 0;
    for(Indices::ChunkList::const_iterator i = _subConstraintMap.getChunks().begin(); i != _subConstraintMap.getChunks().end(); ++i)
    {
        chunk_size = i->size();
        sub_A.block(j,0,chunk_size,_x_size) = A.block(i->front(),0,i->back()-i->front()+1, _x_size);
        j+=chunk_size;
    }
}

void SubConstraint::generateBound(const Eigen::VectorXd& bound, Eigen::VectorXd& sub_bound)
{
    unsigned int chunk_size = 0;
    unsigned int j = 0;
    for(Indices::ChunkList::const_iterator i = _subConstraintMap.getChunks().begin(); i != _subConstraintMap.getChunks().end();++i)
    {
        chunk_size = i->size();

        sub_bound.segment(j, chunk_size) = bound.segment(i->front(),i->back()-i->front()+1);

        j+=chunk_size;
    }
}

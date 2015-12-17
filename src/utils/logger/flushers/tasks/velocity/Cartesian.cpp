#include <OpenSoT/utils/logger/flushers/tasks/velocity/Cartesian.h>

using namespace OpenSoT::flushers::tasks::velocity;

Cartesian::Cartesian(OpenSoT::tasks::velocity::Cartesian::Ptr cartesian, const iDynUtils &model):
    TaskFlusher(cartesian)
{

    std::list<std::string> description;

    description.push_back("error/position/x");
    description.push_back("error/position/y");
    description.push_back("error/position/z");

    description.push_back("error/orientation/x");
    description.push_back("error/orientation/y");
    description.push_back("error/orientation/z");

    this->setDescription(description);

    _cartesian = boost::dynamic_pointer_cast<OpenSoT::tasks::velocity::Cartesian>(_task);
}

std::string Cartesian::toString() const
{
    std::stringstream ss;

    for(unsigned int i = 0; i < _cartesian->positionError.size(); ++i)
        ss << ", " << _cartesian->positionError[i];
    for(unsigned int i = 0; i < _cartesian->orientationError.size(); ++i)
        ss << ", " << _cartesian->orientationError[i];

    return ss.str();
}

OpenSoT::Indices Cartesian::getIndices(int label) const
{
    std::list<unsigned int> emptyList;
    OpenSoT::Indices indices(emptyList);

    if(label & POSITION_ERROR)
        indices = indices + OpenSoT::Indices::range(0,_cartesian->positionError.size()-1);
    if(label & ORIENTATION_ERROR)
        indices = indices + OpenSoT::Indices::range(_cartesian->positionError.size(),
                                                    2*_cartesian->orientationError.size()-1);

    if(indices.size() == 0)
        /// @TODO throw assertion
        ;

    return indices;
}

int Cartesian::getSize() const
{
    // position error, orientation error
    return _cartesian->positionError.size() + _cartesian->orientationError.size();
}

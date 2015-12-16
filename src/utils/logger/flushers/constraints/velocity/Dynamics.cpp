#include <OpenSoT/utils/logger/flushers/constraints/velocity/Dynamics.h>

using namespace OpenSoT::flushers::constraints::velocity;

std::string Dynamics::toString()
{
    std::stringstream ss;
    OpenSoT::constraints::velocity::Dynamics::Ptr _dynamics =
            boost::dynamic_pointer_cast<OpenSoT::constraints::velocity::Dynamics>(_constraint);
    yarp::sig::Vector tau_estimated = _dynamics->getEstimatedTorques(_q_dot);
    for(unsigned int i = 0; i < _constraint->getXSize(); ++i)
        ss << ", " << _dynamics->getTorqueLimits()[i];
    for(unsigned int i = 0; i < _constraint->getXSize(); ++i)
        ss << ", " << tau_estimated[i];
    for(unsigned int i = 0; i < _constraint->getXSize(); ++i)
        ss << ", " << _dynamics->getbUpperBound()[i];
    for(unsigned int i = 0; i < _constraint->getXSize(); ++i)
        ss << ", " << _dynamics->getbLowerBound()[i];
    ss << ", " << _dynamics->getBoundScaling();
    return ss.str();
}

OpenSoT::Indices Dynamics::getIndices(int label) const
{
    switch(label){
    case TORQUE_LIMITS:
        return 0;
    case ESTIMATED_TORQUE:
        return _constraint->getXSize();
    case BUPPERBOUND:
        return _constraint->getXSize()*2;
    case BLOWERBOUND:
        return _constraint->getXSize()*3;
    case SIGMA:
        return _constraint->getXSize()*4;
    default:
        return -1;  /* @TODO should throw assert here */
    }
}

int Dynamics::getSize()
{
    // torque limits, estimated torque, sigma, bUpperBound, bLowerBound
    return _constraint->getXSize()*4 + 1;
}

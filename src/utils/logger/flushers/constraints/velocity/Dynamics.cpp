#include <OpenSoT/utils/logger/flushers/constraints/velocity/Dynamics.h>

using namespace OpenSoT::flushers::constraints::velocity;

Dynamics::Dynamics(OpenSoT::constraints::velocity::Dynamics::Ptr dynamics,
                   const iDynUtils &model)
    : ConstraintFlusher(dynamics)
{
    const std::vector<std::string> jointNames = model.getJointNames();
    std::vector<std::string> description;

    for(unsigned int i = 0; i < jointNames.size(); ++i)
        description.push_back(jointNames[i] + " tau_{lim}");
    for(unsigned int i = 0; i < jointNames.size(); ++i)
        description.push_back(jointNames[i] + " tau_{est}");
    for(unsigned int i = 0; i < jointNames.size(); ++i)
        description.push_back(jointNames[i] + " bUpperBound");
    for(unsigned int i = 0; i < jointNames.size(); ++i)
        description.push_back(jointNames[i] + " bLowerBound");
    description.push_back("bound scaling");

    this->setDescription(description);
}

std::string Dynamics::toString() const
{
    std::stringstream ss;
    OpenSoT::constraints::velocity::Dynamics::Ptr _dynamics =
            boost::dynamic_pointer_cast<OpenSoT::constraints::velocity::Dynamics>(_constraint);
    Eigen::VectorXd tau_estimated = _dynamics->getEstimatedTorques(_q_dot);
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
    std::list<unsigned int> emptyList;
    OpenSoT::Indices indices(emptyList);
    if(label & TORQUE_LIMITS)
        indices = indices + OpenSoT::Indices::range(0,_constraint->getXSize()-1);
    if(label & ESTIMATED_TORQUE)
        indices = indices + OpenSoT::Indices::range(_constraint->getXSize(),
                                                    2*_constraint->getXSize()-1);
    if(label & BUPPERBOUND)
        indices = indices + OpenSoT::Indices::range(2*_constraint->getXSize(),
                                                    3*_constraint->getXSize()-1);
    if(label & BLOWERBOUND)
        indices = indices + OpenSoT::Indices::range(3*_constraint->getXSize(),
                                                    4*_constraint->getXSize()-1);
    if(label & SIGMA)
        indices = indices + 4*_constraint->getXSize();

    if(indices.size() == 0)
        /// @TODO throw assertion
        ;

    return indices;
}

int Dynamics::getSize() const
{
    // torque limits, estimated torque, sigma, bUpperBound, bLowerBound
    return _constraint->getXSize()*4 + 1;
}

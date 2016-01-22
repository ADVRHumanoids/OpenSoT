#include <OpenSoT/utils/logger/flushers/RobotFlusher.h>

using namespace OpenSoT::flushers;

RobotFlusher::RobotFlusher(RobotUtils& robot):
    _robot(robot),
    _nDoFs(robot.getNumberOfJoints())
{
    std::vector<std::string> description;
    std::vector<std::string> jointNames = robot.getJointNames();
    for(unsigned int i = 0; i < _nDoFs; ++i)
        description.push_back(jointNames[i]+ " position");
    for(unsigned int i = 0; i < _nDoFs; ++i)
        description.push_back(jointNames[i]+ " velocity");
    for(unsigned int i = 0; i < _nDoFs; ++i)
        description.push_back(jointNames[i]+ " torque");
    this->setDescription(description);
}

std::string RobotFlusher::toString() const
{
    std::stringstream ss;
    yarp::sig::Vector q = _robot.sensePosition();
    yarp::sig::Vector dq = _robot.senseVelocity();
    yarp::sig::Vector tau = _robot.senseTorque();
    for(unsigned int i = 0; i < _nDoFs; ++i)
        ss << ", " << q[i];
    for(unsigned int i = 0; i < _nDoFs; ++i)
        ss << ", " << dq[i];
    for(unsigned int i = 0; i < _nDoFs; ++i)
        ss << ", " << tau[i];

    return ss.str();
}

OpenSoT::Indices RobotFlusher::getIndices(int label) const
{
    std::list<unsigned int> emptyList;
    OpenSoT::Indices indices(emptyList);

    if(label & Q)
        indices = indices + OpenSoT::Indices::range(0,_nDoFs-1);
    if(label & DQ)
        indices = indices + OpenSoT::Indices::range(_nDoFs,
                                                    2*_nDoFs-1);
    if(label & TAU)
        indices = indices + OpenSoT::Indices::range(2*_nDoFs,
                                                    3*_nDoFs-1);

    if(indices.size() == 0)
        /// @TODO throw assertion
        ;

    return indices;
}

int RobotFlusher::getSize() const
{
    // q, dq, tau
    return 3*_nDoFs;
}

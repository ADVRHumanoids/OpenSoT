#include <OpenSoT/utils/logger/L.h>
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

void OpenSoT::flushers::RobotFlusher::defaultPlot(OpenSoT::L &l, OpenSoT::Indices &i)
{
    l.plotter->figure(10.24,7.68,"estimated torques for torso vs real torques");

    OpenSoT::plotters::Plottable qPlottable =
        l.getFlusher(_robot)->i(Q);
    if(i.size() > 0)
        qPlottable.second.filter(i);
    l.plotter->subplot(2,2,1);
    l.plotter->plot_t(qPlottable);
    l.plotter->title("Robot Configuration");
    l.plotter->autoLegend(qPlottable);
    l.plotter->xlabel("t [s]");
    l.plotter->ylabel("q [rad]");


    OpenSoT::plotters::Plottable dqPlottable =
        l.getFlusher(_robot)->i(DQ);
    if(i.size() > 0)
        dqPlottable.second.filter(i);
    l.plotter->subplot(2,2,2);
    l.plotter->plot_t(dqPlottable);
    l.plotter->title("Robot Velocities");
    l.plotter->autoLegend(dqPlottable);
    l.plotter->xlabel("t [s]");
    l.plotter->ylabel("\dq [rad/s]");


    OpenSoT::plotters::Plottable tauPlottable =
        l.getFlusher(_robot)->i(TAU);
    if(i.size() > 0)
        tauPlottable.second.filter(i);
    l.plotter->subplot(2,2,3);
    l.plotter->plot_t(tauPlottable);
    l.plotter->title("Robot Torques");
    l.plotter->autoLegend(tauPlottable);
    l.plotter->xlabel("t [s]");
    l.plotter->ylabel("tau [Nm]");

    l.plotter->tight_layout();
    l.plotter->savefig();
    l.plotter->show();
}

void OpenSoT::flushers::RobotFlusher::defaultPlot(OpenSoT::L &l)
{
    std::list<unsigned int> empty_list;
    OpenSoT::Indices i(empty_list);
    this->defaultPlot(l, i);
}

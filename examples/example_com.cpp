#include <drc_shared/comanutils.h>
#include "OpenSoT/tasks/velocity/CoM.h"
#include "OpenSoT/solvers/QPOases.h"

using namespace OpenSoT;
namespace vTasks = OpenSoT::tasks::velocity;

void main(int argc, char* argv[]) {
    yarp::os::Network::init();

    ComanUtils robot;
    yarp::sig::Vector q = robot.sensePosition();
    yarp::sig::Vector dq(q.size(),0.0);

    vTasks::CoM::Ptr com(new vTasks::CoM(q));

    solver::QPOases_sot::Stack stack;
    stack.push_bask(com);
    solvers::QPOases_sot solver(com);

    double t_now = yarp::os::Time::now();
    while(yarp::os::Time::now() - t_now < 10.0) {
        q = robot.sensePosition();
        com->update(q);
        solver.solve(dq);
        q += dq;
        yarp::os::Time::delay(0.01);
    }
    return;
}

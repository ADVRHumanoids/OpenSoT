#include <drc_shared/comanutils.h>
#include "OpenSoT/tasks/velocity/CoM.h"
#include "OpenSoT/solvers/QPOases.h"
#include <yarp/math/Math.h>

using namespace yarp::math;
using namespace OpenSoT;
namespace vTasks = OpenSoT::tasks::velocity;

int main(int argc, char* argv[]) {
    yarp::os::Network::init();

    ComanUtils robot("example_com");
    yarp::sig::Vector q = robot.sensePosition();
    yarp::sig::Vector dq(q.size(),0.0);

    vTasks::CoM::Ptr com(new vTasks::CoM(q));

    solvers::QPOases_sot::Stack stack;
    stack.push_back(com);
    solvers::QPOases_sot solver(stack);

    yarp::sig::Vector comInitialP = com->getActualPosition();

    double t_start = yarp::os::Time::now();
    double t = t_start;
    while(t - t_start < 10.0) {
        double delta = cos(0.1*t);

        yarp::sig::Vector comReference = comInitialP;
        comReference += yarp::sig::Vector(3,delta);

        com->setReference(comReference);
        com->update(q);
        solver.solve(dq);

        q += dq;

        robot.move(q);
        yarp::os::Time::delay(0.01);
        t = yarp::os::Time::now();
    }
    return 1;
}

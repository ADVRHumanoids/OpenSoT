#include <idynutils/comanutils.h>
#include "OpenSoT/tasks/velocity/MinimumEffort.h"
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

    vTasks::MinimumEffort::Ptr minimumEffort(new vTasks::MinimumEffort(q, robot.idynutils));

    solvers::QPOases_sot::Stack stack;
    stack.push_back(minimumEffort);
    solvers::QPOases_sot solver(stack);

    robot.setPositionDirectMode();
    double t_start = yarp::os::Time::now();
    double t = t_start;
    while(t - t_start < 10.0) {

        minimumEffort->update(q);
        solver.solve(dq);

        q += dq;

        robot.move(q);
        yarp::os::Time::delay(0.01);
        t = yarp::os::Time::now();
    }
    return 1;
}

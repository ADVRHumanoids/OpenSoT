#include <idynutils/comanutils.h>
#include "OpenSoT/tasks/velocity/Postural.h"
#include "OpenSoT/solvers/QPOases.h"
#include <yarp/math/Math.h>
#include <algorithm>

using namespace yarp::math;
using namespace OpenSoT;
namespace vTasks = OpenSoT::tasks::velocity;

int main(int argc, char* argv[]) {
    yarp::os::Network::init();

    ComanUtils robot("example_com");

    yarp::sig::Vector q = robot.sensePosition();
    yarp::sig::Vector dq(q.size(),0.0);

    vTasks::Postural::Ptr postural(new vTasks::Postural(
                                       cartesian_utils::toEigen(q)));

    solvers::QPOases_sot::Stack stack;
    stack.push_back(postural);
    solvers::QPOases_sot solver(stack);

    // assuming all torso joints are consecutive..
    int firstTorsoId = *std::min_element(robot.idynutils.torso.joint_numbers.begin(),
                                         robot.idynutils.torso.joint_numbers.end());
    int torsoDoFs = robot.idynutils.torso.joint_numbers.size();
    yarp::sig::Vector initialTorsoPosture = q.subVector(firstTorsoId,firstTorsoId+torsoDoFs-1);
    yarp::sig::Vector initialPosture = q;

    robot.setPositionDirectMode();
    double t_start = yarp::os::Time::now();
    double t = t_start;
    Eigen::VectorXd _dq(dq.size()); _dq.setZero(dq.size());
    while(t - t_start < 10.0) {
        double delta = .3*cos(0.1*t);

        yarp::sig::Vector posturalReference = initialPosture;
        yarp::sig::Vector torsoPosturalReference = initialTorsoPosture + yarp::sig::Vector(3,delta);
        posturalReference.setSubvector(firstTorsoId, torsoPosturalReference);

        postural->setReference(cartesian_utils::toEigen(posturalReference));
        postural->update(cartesian_utils::toEigen(q));
        solver.solve(_dq);

        dq = cartesian_utils::fromEigentoYarp(_dq);
        q += dq;

        robot.move(q);
        yarp::os::Time::delay(0.01);
        t = yarp::os::Time::now();
    }
    return 1;
}

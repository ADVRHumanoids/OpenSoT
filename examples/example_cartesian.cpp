#include <drc_shared/comanutils.h>
#include "OpenSoT/tasks/velocity/Cartesian.h"
#include "OpenSoT/solvers/QPOases.h"
#include <yarp/math/Math.h>
#define  m  1

using namespace yarp::math;
using namespace OpenSoT;
namespace vTasks = OpenSoT::tasks::velocity;

int main(int argc, char* argv[]) {
    yarp::os::Network::init();

    ComanUtils robot("example_com");
    yarp::sig::Vector q = robot.sensePosition();
    yarp::sig::Vector dq(q.size(),0.0);

    vTasks::Cartesian::Ptr cartesian(new vTasks::Cartesian("cartesian::l_wrist::world", q,
                                                           robot.idynutils,
                                                           robot.idynutils.left_arm.end_effector_name,
                                                           "world"));

    solvers::QPOases_sot::Stack stack;
    stack.push_back(cartesian);
    solvers::QPOases_sot solver(stack);

    yarp::sig::Matrix initialPose = cartesian->getActualPose();
    yarp::sig::Vector initialPosition = initialPose.getCol(3).subVector(0,2);

    // setting a lower lambda for convergence
    cartesian->setLambda(0.1);

    // setting a lower gain for orientation error: we care about position
    cartesian->setOrientationErrorGain(.1);

    robot.setPositionDirectMode();
    double t_start = yarp::os::Time::now();
    double t = t_start;
    while(t - t_start < 10.0) {
        // delta goes from -.025m to +.025m
        double delta = .025*m*cos(0.1*t);

        // changing position reference in x,y,z directions
        yarp::sig::Matrix poseReference = initialPose;
        yarp::sig::Vector positionReference = initialPosition + yarp::sig::Vector(3,delta);
        poseReference.setSubcol(positionReference,0,3);

        cartesian->setReference(poseReference);
        cartesian->update(q);
        solver.solve(dq);

        q += dq;

        robot.move(q);
        yarp::os::Time::delay(0.01);
        t = yarp::os::Time::now();
    }
    return 1;
}

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>
#include <idynutils/idynutils.h>
#include <idynutils/RobotUtils.h>
#include <OpenSoT/interfaces/yarp/tasks/YCartesian.h>
#include <OpenSoT/interfaces/yarp/tasks/YCoM.h>
#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/utils/DefaultHumanoidStack.h>
#include <OpenSoT/utils/VelocityAllocation.h>

#define MODULE_NAME "example_python_simple_stabilizer"
#define dT          10e-3

int main(int argc, char **argv) {
    yarp::os::Network::init();

    RobotUtils robot( MODULE_NAME, "coman",
                     std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.urdf",
                     std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.srdf");
    yarp::os::Time::delay(1.0);
    yarp::sig::Vector q = robot.sensePosition();
    robot.idynutils.updateiDyn3Model(q,true);
    robot.idynutils.setFloatingBaseLink(robot.idynutils.left_leg.end_effector_name);
    OpenSoT::DefaultHumanoidStack DHS(robot.idynutils, dT, q);

    double K = 0.01;
    DHS.velocityLimits->setVelocityLimits(0.3);
    DHS.waist->setLambda(K);
    DHS.waist->setOrientationErrorGain(0.1);
    DHS.com->setLambda(K);
    DHS.leftLeg->setLambda(K);
    DHS.leftLeg->setOrientationErrorGain(0.1);
    DHS.rightLeg->setLambda(K);
    DHS.rightLeg->setOrientationErrorGain(0.1);

    //TODO:CHECK THIS STACK SINCE IT IS WRONG!
    OpenSoT::AutoStack::Ptr autoStack =
        ( DHS.com + DHS.waist) /
        ( DHS.rightLeg + DHS.leftLeg) /
        ( DHS.postural);
    autoStack << DHS.jointLimits << DHS.velocityLimits;

    OpenSoT::solvers::QPOases_sot solver(autoStack->getStack(),
                                         autoStack->getBounds(), 2e10);

    robot.setPositionDirectMode();

    yarp::sig::Vector dq(q.size(), 0.0);
    double tic, toc;
    while(true) {

        tic = yarp::os::Time::now();

        KDL::Rotation orientation;
        KDL::Vector linearAcc;
        KDL::Vector angularVel;

        robot.getIMU()->sense(orientation, linearAcc, angularVel);
        yarp::sig::Matrix imu_orientation;
        cartesian_utils::fromKDLRotationToYARPMatrix(orientation, imu_orientation);

        robot.idynutils.setIMUOrientation(imu_orientation, robot.getIMU()->getReferenceFrame());
        robot.idynutils.updateiDyn3Model(q, true);

        autoStack->update(q);
        if(solver.solve(dq))
            q+=dq;
        else
            std::cout << "Error computing solve()" << std::endl;
        robot.move(q);

        toc = yarp::os::Time::now();

        yarp::os::Time::delay(dT-(toc-tic));
    }

}

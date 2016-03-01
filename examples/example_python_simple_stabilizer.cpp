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

    robot.setPositionDirectMode();
    yarp::sig::Vector q = robot.sensePosition();
    robot.idynutils.switchAnchorAndFloatingBase(robot.idynutils.left_leg.end_effector_name);
    robot.idynutils.updateiDyn3Model(q, true);

    KDL::Rotation orientation;
    KDL::Vector linearAcc;
    KDL::Vector angularVel;

    robot.getIMU()->sense(orientation, linearAcc, angularVel);
    yarp::sig::Matrix imu_orientation;
    cartesian_utils::fromKDLRotationToYARPMatrix(orientation, imu_orientation);
    robot.idynutils.setIMUOrientation(imu_orientation, robot.getIMU()->getReferenceFrame());
    robot.idynutils.updateiDyn3Model(q, true);


    OpenSoT::DefaultHumanoidStack DHS(robot.idynutils, dT, q);

    double K = 1.0;
    DHS.velocityLimits->setVelocityLimits(0.3);
    DHS.waist->setLambda(K);
    DHS.waist->setOrientationErrorGain(0.1);
    DHS.com->setLambda(K);
    DHS.right2LeftLeg->setLambda(K);
    DHS.right2LeftLeg->setOrientationErrorGain(0.1);
    DHS.postural->setLambda(K);
    DHS.postural->setReference(q);

    //TODO:CHECK THIS STACK SINCE IT IS WRONG!
    OpenSoT::AutoStack::Ptr autoStack =
        ( DHS.waist) /
        ( DHS.right2LeftLeg) /
        ( DHS.com) /
        ( DHS.postural);
    autoStack << DHS.jointLimits << DHS.velocityLimits;
    autoStack->update(q);



    OpenSoT::solvers::QPOases_sot solver(autoStack->getStack(),
                                         autoStack->getBounds(), 2e10);

    yarp::sig::Vector dq(q.size(), 0.0);
    double tic, toc;
    while(true) {

        tic = yarp::os::Time::now();

        KDL::Rotation orientation_new;
        robot.getIMU()->sense(orientation_new, linearAcc, angularVel);
        double x,y,z,w,x_new,y_new,z_new,w_new;
        orientation.GetQuaternion(x,y,z,w);
        orientation_new.GetQuaternion(x_new,y_new,z_new,w_new);

        x += (x_new-x)*0.5;
        y += (y_new-y)*0.5;
        z += (z_new-z)*0.5;
        w += (w_new-w)*0.5;

        orientation.Quaternion(x,y,z,w);
        yarp::sig::Matrix imu_orientation;
        cartesian_utils::fromKDLRotationToYARPMatrix(orientation, imu_orientation);

        robot.idynutils.setIMUOrientation(imu_orientation, robot.getIMU()->getReferenceFrame());
        robot.idynutils.updateiDyn3Model(q, true);

//        KDL::Frame anchor_T_world; std::string anchor;
//        robot.idynutils.getWorldPose(anchor_T_world, anchor);
//        std::cout<<"Anchor is in "<<anchor<<std::endl;
//        std::cout<<"anchor_T_world:"<<std::endl; cartesian_utils::printKDLFrame(anchor_T_world);
//        std::cout<<std::endl;

        autoStack->update(q);

        std::cout<<"CoM Error: ["<<DHS.com->getError().toString()<<""<<std::endl;
        std::cout<<"right2LeftLeg Error: ["<<DHS.right2LeftLeg->getError().toString()<<""<<std::endl;
        std::cout<<"waist Error: ["<<DHS.waist->getError().toString()<<""<<std::endl;
        std::cout<<std::endl;

        if(solver.solve(dq))
            q+=dq;
        else
            std::cout << "Error computing solve()" << std::endl;
        robot.move(q);

        toc = yarp::os::Time::now();

        if((toc-tic) < dT)
            yarp::os::Time::delay(dT-(toc-tic));
    }

}

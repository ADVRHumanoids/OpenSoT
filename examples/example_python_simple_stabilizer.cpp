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
#include <qpOASES/Options.hpp>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <geometry_msgs/TransformStamped.h>

#define MODULE_NAME "example_python_simple_stabilizer"
#define dT          10e-3
#define PUBLISH_WORLD false

void publish_world(ros::Publisher& world_pulisher, const KDL::Frame& anchor_T_world, const std::string& anchor)
{
    geometry_msgs::TransformStamped T;

    T.header.frame_id = anchor;
    T.child_frame_id = "world";

    T.transform.translation.x = anchor_T_world.p.x();
    T.transform.translation.y = anchor_T_world.p.y();
    T.transform.translation.z = anchor_T_world.p.z();

    double x,y,z,w;
    anchor_T_world.M.GetQuaternion(x,y,z,w);
    T.transform.rotation.x = x;
    T.transform.rotation.y = y;
    T.transform.rotation.z = z;
    T.transform.rotation.w = w;

    T.header.stamp = ros::Time::now();

    world_pulisher.publish(T);
}

int main(int argc, char **argv) {
    yarp::os::Network::init();

#if PUBLISH_WORLD
    ros::init(argc, argv, MODULE_NAME);
    ros::NodeHandle n;
    ros::Publisher world_pub = n.advertise<geometry_msgs::TransformStamped>("anchor_to_world_pose", 1000);
#endif
    RobotUtils robot( MODULE_NAME, "coman",
                     std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.urdf",
                     std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.srdf");
    yarp::os::Time::delay(3.0);

    robot.setPositionDirectMode();
    yarp::os::Time::delay(3.0);
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

    double K = 0.1;
    DHS.velocityLimits->setVelocityLimits(0.3);
    DHS.waist->setLambda(K);
    DHS.waist->setOrientationErrorGain(0.1);
    DHS.com->setLambda(K);
    DHS.right2LeftLeg->setLambda(K);
    DHS.right2LeftLeg->setOrientationErrorGain(0.1);
    DHS.postural->setLambda(K);

    //TODO:CHECK THIS STACK SINCE IT IS WRONG!
    OpenSoT::AutoStack::Ptr autoStack =
        ( DHS.right2LeftLeg) /
        ( DHS.waist) /
        ( DHS.com) /
        ( DHS.postural);
    autoStack << DHS.jointLimits << DHS.velocityLimits;
    autoStack->update(q);

    std::cout<<"right2LeftLeg Initial Error:"<<std::endl;
    std::cout<<"["<<DHS.right2LeftLeg->getError().toString()<<"]"<<std::endl;
    std::cout<<"waist Initial Error:"<<std::endl;
    std::cout<<"["<<DHS.waist->getError().toString()<<"]"<<std::endl;
    std::cout<<"CoM Initial Error:"<<std::endl;
    std::cout<<"["<<DHS.com->getError().toString()<<"]"<<std::endl;
    getchar();




    qpOASES::Options opt;
    OpenSoT::solvers::QPOases_sot solver(autoStack->getStack(),
                                         autoStack->getBounds(), 2e12);
    for(unsigned int i = 0; i < solver.getNumberOfTasks(); ++i)
    {
        if(solver.getOptions(i, opt)){
            opt.terminationTolerance = 1e-3;
            if(!(solver.setOptions(i, opt))){
                std::cout<<"Error setting task "<<i<<" options"<<std::endl;
                return 0;
            }
        }
        else{
            std::cout<<"Error reading task "<<i<<" options"<<std::endl;
            return 0;}
    }

    yarp::sig::Vector dq(q.size(), 0.0);
    double tic, toc;
    while(true) {

        tic = yarp::os::Time::now();

        //q = robot.sensePosition();
        KDL::Rotation orientation_m;
        robot.getIMU()->sense(orientation_m, linearAcc, angularVel);

        double r,p,y,rr,pp,yy;
        double gain = 0.1;
        orientation.GetRPY(r,p,y);
        orientation_m.GetRPY(rr,pp,yy);
        r += (rr-r)*gain;
        p += (pp-p)*gain;
        y += (yy-y)*gain;
        orientation = orientation.RPY(r,p,y);

        cartesian_utils::fromKDLRotationToYARPMatrix(orientation, imu_orientation);


        robot.idynutils.setIMUOrientation(imu_orientation, robot.getIMU()->getReferenceFrame());
        robot.idynutils.updateiDyn3Model(q, true);
#if PUBLISH_WORLD
        publish_world(world_pub, robot.idynutils.getAnchor_T_World(), robot.idynutils.getAnchor());
#endif

        autoStack->update(q);

//        if(getchar()){
//            std::cout<<"right2LeftLeg Error: ["<<DHS.right2LeftLeg->getError().toString()<<""<<std::endl;
//            std::cout<<"waist Error: ["<<DHS.waist->getError().toString()<<""<<std::endl;
//            std::cout<<"CoM Error: ["<<DHS.com->getError().toString()<<""<<std::endl;
//            KDL::Frame anchor_T_world; std::string anchor; robot.idynutils.getWorldPose(anchor_T_world, anchor);
//            std::cout<<"anchor_T_world:"<<std::endl; cartesian_utils::printKDLFrame(anchor_T_world);
//            std::cout<<std::endl;}

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

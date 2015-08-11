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

#define MODULE_NAME "example_python"
#define dT          25e-3

typedef boost::accumulators::accumulator_set<double,
                                            boost::accumulators::stats<boost::accumulators::tag::rolling_mean>
                                            > Accumulator;
int main(int argc, char **argv) {
    yarp::os::Network::init();
    Accumulator time_accumulator(boost::accumulators::tag::rolling_mean::window_size = 1000);
    RobotUtils robot( MODULE_NAME, "bigman",
                     std::string(OPENSOT_TESTS_ROBOTS_DIR)+"bigman/bigman.urdf",
                     std::string(OPENSOT_TESTS_ROBOTS_DIR)+"bigman/bigman.srdf");
    yarp::os::Time::delay(1.0);
    yarp::sig::Vector q = robot.sensePosition();
    robot.idynutils.updateiDyn3Model(q,true);
    robot.idynutils.setFloatingBaseLink(robot.idynutils.left_leg.end_effector_name);
    OpenSoT::DefaultHumanoidStack DHS(robot.idynutils, dT, q);


    /*                            */
    /*       CREATING STACK       */
    /*                            */

    // defining a stack composed of size two,
    // where the task of first priority is an aggregated of leftLeg and rightLeg,
    // task of priority two is leftArm and rightArm,
    // and the stack is subject to bounds jointLimits and velocityLimits
    OpenSoT::AutoStack::Ptr autoStack = 
        ( DHS.rightLeg << DHS.convexHull) /
        ( (DHS.leftArm + DHS.waist_Orientation + DHS.com) << DHS.selfCollisionAvoidance << DHS.convexHull) /
        ( (DHS.postural)  << DHS.selfCollisionAvoidance << DHS.convexHull);
        /*( DHS.rightLeg ) /
        ( (DHS.waist_Position_Z + DHS.com_XY) ) /
        ( (DHS.leftArm ) ) /
        ( (DHS.postural) );*/
    autoStack << DHS.jointLimits; // << DHS.velocityLimits; commented since we are using VelocityALlocation


    /*                            */
    /*      CONFIGURING STACK     */
    /*                            */

    DHS.com->setLambda(0.0);        // com is a minimum velocity task
    DHS.convexHull->setSafetyMargin(0.05); // 5cm bound
    DHS.rightLeg->setLambda(0.6);   DHS.rightLeg->setOrientationErrorGain(1.0);
    DHS.leftLeg->setLambda(0.6);    DHS.leftLeg->setOrientationErrorGain(1.0);
    DHS.rightArm->setLambda(0.1);   DHS.rightArm->setOrientationErrorGain(0.3);
    DHS.leftArm->setLambda(0.1);    DHS.leftArm->setOrientationErrorGain(0.3);
    DHS.jointLimits->setBoundScaling(0.3);
    DHS.selfCollisionAvoidance->setBoundScaling(0.3);
    DHS.velocityLimits->setVelocityLimits(0.3);

    yarp::sig::Matrix pW = DHS.postural->getWeight();
    for(unsigned int i_t = 0; i_t < 3; ++i_t)
        pW(robot.idynutils.torso.joint_numbers[i_t],
            robot.idynutils.torso.joint_numbers[i_t]) *= 1e3;
    for(unsigned int i_t = 0; i_t < 6; ++i_t)
    {
        double amt = 7.5e1;
        if(i_t == 3 || i_t == 4)
            amt = 3;
        pW(robot.idynutils.left_leg.joint_numbers[i_t],
           robot.idynutils.left_leg.joint_numbers[i_t]) *= amt;
        pW(robot.idynutils.right_leg.joint_numbers[i_t],
           robot.idynutils.right_leg.joint_numbers[i_t]) *= amt;
    }
    DHS.postural->setWeight(pW);

    pW = DHS.waist_Orientation->getWeight();
    pW(1, 1) *= 1e-2;
    DHS.waist_Orientation->setWeight(pW);

    std::list<std::pair<std::string,std::string> > whiteList;
    // lower body - arms collision whitelist for WalkMan (for upper-body manipulation tasks - i.e. not crouching)
    whiteList.push_back(std::pair<std::string,std::string>("LLowLeg","LSoftHandLink"));
    whiteList.push_back(std::pair<std::string,std::string>("LHipMot","LSoftHandLink"));
    whiteList.push_back(std::pair<std::string,std::string>("RLowLeg","RSoftHandLink"));
    whiteList.push_back(std::pair<std::string,std::string>("RHipMot","RSoftHandLink"));

    // torso - arms collision whitelist for WalkMan
    whiteList.push_back(std::pair<std::string,std::string>("DWS","LSoftHandLink"));
    whiteList.push_back(std::pair<std::string,std::string>("DWS","LWrMot2"));
    whiteList.push_back(std::pair<std::string,std::string>("DWS","RSoftHandLink"));
    whiteList.push_back(std::pair<std::string,std::string>("DWS","RWrMot2"));
    whiteList.push_back(std::pair<std::string,std::string>("TorsoProtections","LElb"));
    whiteList.push_back(std::pair<std::string,std::string>("TorsoProtections","LSoftHandLink"));
    whiteList.push_back(std::pair<std::string,std::string>("TorsoProtections","RElb"));
    whiteList.push_back(std::pair<std::string,std::string>("TorsoProtections","RSoftHandLink"));
    whiteList.push_back(std::pair<std::string,std::string>("Waist","LSoftHandLink"));
    whiteList.push_back(std::pair<std::string,std::string>("Waist","LWrMot2"));
    whiteList.push_back(std::pair<std::string,std::string>("Waist","RSoftHandLink"));
    whiteList.push_back(std::pair<std::string,std::string>("Waist","RWrMot2"));

    // arm - am collision whitelist for WalkMan
    whiteList.push_back(std::pair<std::string,std::string>("LShr","RShr"));
    whiteList.push_back(std::pair<std::string,std::string>("LShr","RSoftHandLink"));
    whiteList.push_back(std::pair<std::string,std::string>("LShr","RWrMot2"));
    whiteList.push_back(std::pair<std::string,std::string>("LSoftHandLink","RShr"));
    whiteList.push_back(std::pair<std::string,std::string>("LSoftHandLink","RSoftHandLink"));
    whiteList.push_back(std::pair<std::string,std::string>("LSoftHandLink","RWrMot2"));
    whiteList.push_back(std::pair<std::string,std::string>("LWrMot2","RShr"));
    whiteList.push_back(std::pair<std::string,std::string>("LWrMot2","RSoftHandLink"));
    whiteList.push_back(std::pair<std::string,std::string>("LWrMot2","RWrMot2"));

    DHS.selfCollisionAvoidance->setCollisionWhiteList(whiteList);

    OpenSoT::VelocityAllocation(autoStack,
                                dT,
                                0.3,
                                0.3);

    // setting higher velocity limit to last stack --
    // TODO next feature of VelocityAllocation is a last_stack_speed ;)
    typedef std::list<OpenSoT::Constraint<yarp::sig::Matrix,yarp::sig::Vector>::ConstraintPtr>::iterator it_constraint;
    OpenSoT::Task<yarp::sig::Matrix, yarp::sig::Vector>::TaskPtr lastTask = autoStack->getStack()[2];
    for(it_constraint i_c = lastTask->getConstraints().begin() ;
        i_c != lastTask->getConstraints().end() ; ++i_c) {
        if( boost::dynamic_pointer_cast<
                OpenSoT::constraints::velocity::VelocityLimits>(
                    *i_c))
            boost::dynamic_pointer_cast<
                            OpenSoT::constraints::velocity::VelocityLimits>(
                                *i_c)->setVelocityLimits(.6);
    }
/*
    OpenSoT::Task<yarp::sig::Matrix, yarp::sig::Vector>::TaskPtr beforeLastTask = autoStack->getStack()[1];
        for(it_constraint i_c = beforeLastTask->getConstraints().begin() ;
            i_c != beforeLastTask->getConstraints().end() ; ++i_c) {
            if( boost::dynamic_pointer_cast<
                    OpenSoT::constraints::velocity::VelocityLimits>(
                        *i_c))
                boost::dynamic_pointer_cast<
                                OpenSoT::constraints::velocity::VelocityLimits>(
                                    *i_c)->setVelocityLimits(.7);
    }*/


    /*                            */
    /*  CREATING TASK INTERFACES  */
    /*                            */


    OpenSoT::interfaces::yarp::tasks::YCartesian leftArm(robot.idynutils.getRobotName(),
                                                         MODULE_NAME, DHS.leftArm);


    OpenSoT::interfaces::yarp::tasks::YCartesian waist(robot.idynutils.getRobotName(),
                                                         MODULE_NAME, DHS.waist);
    /*
    OpenSoT::interfaces::yarp::tasks::YCartesian leftLeg(robot.idynutils.getRobotName(),
                                                         MODULE_NAME, DHS.leftLeg);
    OpenSoT::interfaces::yarp::tasks::YCartesian rightLeg(robot.idynutils.getRobotName(),
                                                         MODULE_NAME, DHS.rightLeg);
    */

    OpenSoT::interfaces::yarp::tasks::YCoM com(robot.idynutils.getRobotName(),
                                               MODULE_NAME, DHS.com);

    OpenSoT::solvers::QPOases_sot solver(autoStack->getStack(),
                                         autoStack->getBounds(), 1e10);

    robot.setPositionDirectMode();
    yarp::sig::Vector dq;
    double tic, toc;
    int print_mean = 0;
    while(true) {
        tic = yarp::os::Time::now();
        robot.idynutils.updateiDyn3Model(q, true);

        yarp::sig::Matrix M(6+robot.idynutils.iDyn3_model.getNrOfDOFs(), 6+robot.idynutils.iDyn3_model.getNrOfDOFs());
        robot.idynutils.iDyn3_model.getFloatingBaseMassMatrix(M);
        M.removeCols(0,6); M.removeRows(0,6);
        DHS.postural->setWeight(M);

        autoStack->update(q);
        if(solver.solve(dq))
            q+=dq;
        else
            std::cout << "Error computing solve()" << std::endl;
        robot.move(q);
        toc = yarp::os::Time::now();
        time_accumulator(toc-tic);

        // print mean every 5s
        if(((++print_mean)%int(5.0/dT))==0) {
            print_mean = 0;
            std::cout << "dt = "
                      << boost::accumulators::extract::rolling_mean(time_accumulator) << std::endl;

            std::cout << "l_wrist reference:" << DHS.leftArm->getReference().toString() << std::endl;
            std::cout << "r_wrist reference:" << DHS.rightArm->getReference().toString() << std::endl;
            std::cout << "waist reference:" << DHS.waist->getReference().toString() << std::endl;
            std::cout << "Active Capsules Pairs: " << DHS.selfCollisionAvoidance->getbUpperBound().size() << std::endl;

        }
        yarp::os::Time::delay(dT-(toc-tic));
    }
}

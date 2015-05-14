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
    Accumulator time_accumulator(boost::accumulators::tag::rolling_mean::window_size = 1000);
    RobotUtils robot( MODULE_NAME, "bigman",
                     std::string(OPENSOT_TESTS_ROBOTS_DIR)+"bigman/bigman.urdf",
                     std::string(OPENSOT_TESTS_ROBOTS_DIR)+"bigman/bigman.srdf");
    yarp::os::Time::delay(1.0);
    yarp::sig::Vector q = robot.sensePosition();
    OpenSoT::DefaultHumanoidStack DHS(robot.idynutils, dT, q);

    // defining a stack composed of size two,
    // where the task of first priority is an aggregated of leftLeg and rightLeg,
    // task of priority two is leftArm and rightArm,
    // and the stack is subject to bounds jointLimits and velocityLimits
    OpenSoT::AutoStack::Ptr autoStack = 
        ( (DHS.rightLeg + DHS.leftLeg) << DHS.selfCollisionAvoidance ) /
        ( (DHS.leftArm + DHS.rightArm) << DHS.selfCollisionAvoidance ) /
        (DHS.com << DHS.comVelocity << DHS.selfCollisionAvoidance ) /
        DHS.minimumVelocity << DHS.selfCollisionAvoidance;
    autoStack << DHS.jointLimits << DHS.velocityLimits;

    DHS.rightLeg->setLambda(0.6);   DHS.rightLeg->setOrientationErrorGain(1.0);
    DHS.leftLeg->setLambda(0.6);    DHS.leftLeg->setOrientationErrorGain(1.0);
    DHS.rightArm->setLambda(0.6);   DHS.rightArm->setOrientationErrorGain(0.1);
    DHS.leftArm->setLambda(0.6);    DHS.leftArm->setOrientationErrorGain(0.1);
    DHS.comVelocity->setVelocityLimits(yarp::sig::Vector(0.1,3));
    DHS.velocityLimits->setVelocityLimits(0.9);

    OpenSoT::VelocityAllocation(autoStack,
                                3e-3,
                                0.7,
                                1.0);

    // setting higher velocity limit to last stack --
    // TODO next feature of VelocityAllocation is a last_stack_speed ;)
    typedef std::list<OpenSoT::Constraint<yarp::sig::Matrix,yarp::sig::Vector>::ConstraintPtr>::iterator it_constraint;
    OpenSoT::Task<yarp::sig::Matrix, yarp::sig::Vector>::TaskPtr lastTask = autoStack->getStack().back();
    for(it_constraint i_c = lastTask->getConstraints().begin() ;
        i_c != lastTask->getConstraints().end() ; ++i_c) {
        if( boost::dynamic_pointer_cast<
                OpenSoT::constraints::velocity::VelocityLimits>(
                    *i_c))
            boost::dynamic_pointer_cast<
                            OpenSoT::constraints::velocity::VelocityLimits>(
                                *i_c)->setVelocityLimits(2.0);
    }

    OpenSoT::interfaces::yarp::tasks::YCartesian leftArm(robot.idynutils.getRobotName(),
                                                         MODULE_NAME, DHS.leftArm);
    OpenSoT::interfaces::yarp::tasks::YCartesian rightArm(robot.idynutils.getRobotName(),
                                                         MODULE_NAME, DHS.rightArm);
    OpenSoT::interfaces::yarp::tasks::YCartesian leftLeg(robot.idynutils.getRobotName(),
                                                         MODULE_NAME, DHS.leftLeg);
    OpenSoT::interfaces::yarp::tasks::YCartesian rightLeg(robot.idynutils.getRobotName(),
                                                         MODULE_NAME, DHS.rightLeg);

    OpenSoT::interfaces::yarp::tasks::YCoM com(robot.idynutils.getRobotName(),
                                               MODULE_NAME, DHS.com);

    OpenSoT::solvers::QPOases_sot solver(autoStack->getStack(),
                                         autoStack->getBounds());

    yarp::sig::Vector dq;
    double tic, toc;
    int print_mean = 0;
    while(true) {
        tic = yarp::os::Time::now();
        robot.idynutils.updateiDyn3Model(q, true);
        autoStack->update(q);
        if(solver.solve(dq))
            q+=dq;
        toc = yarp::os::Time::now();
        time_accumulator(toc-tic);

        // print mean every 5s
        if(((++print_mean)%int(5.0/dT))==0) {
            print_mean = 0;
            std::cout << "dt = "
                      << boost::accumulators::extract::rolling_mean(time_accumulator) << std::endl;
        }
        yarp::os::Time::delay(dT-(toc-tic));
    }
}

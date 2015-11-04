#include <idynutils/idynutils.h>
#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/utils/DefaultHumanoidStack.h>
#include <OpenSoT/utils/VelocityAllocation.h>

int main(int argc, char **argv) {

    iDynUtils _robot("coman",
                     std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.urdf",
                     std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.srdf");
    OpenSoT::DefaultHumanoidStack DHS(_robot, 3e-3, _robot.zeros);

    // defining a stack composed of size two,
    // where the task of first priority is an aggregated of leftArm and rightArm,
    // rightArm contains a convexHull constraint;
    // the task at the second priority level is an aggregated of rightLeg and leftLeg,
    // and the stack is subject to bounds jointLimits and velocityLimits
    OpenSoT::AutoStack::Ptr autoStack = 
        (DHS.leftArm + DHS.rightArm)
        / (DHS.rightLeg + DHS.leftLeg)
        / (DHS.com << DHS.comVelocity)
        / DHS.postural;
    autoStack << DHS.jointLimits;

    OpenSoT::VelocityAllocation(autoStack,
                                3e-3,
                                0.1,
                                0.3);
    unsigned int i = 0;
    typedef std::vector< OpenSoT::Task<yarp::sig::Matrix, yarp::sig::Vector>::TaskPtr >::iterator it_t;
    for(it_t task = autoStack->getStack().begin();
        task != autoStack->getStack().end();
        ++task)
    {
        std::cout << "Task "<< i << " has velocity bounds" <<
            boost::dynamic_pointer_cast<
                OpenSoT::constraints::velocity::VelocityLimits>(
                    (*task)->getConstraints().front()
                         )->getVelocityLimits()
                  << std::endl;
        ++i;
    }

}

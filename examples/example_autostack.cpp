#include <idynutils/idynutils.h>
#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/utils/DefaultHumanoidStack.h>

int main(int argc, char **argv) {

    iDynUtils _robot("coman",
                     std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.urdf",
                     std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.srdf");
    yarp::sig::Vector q = _robot.iDyn3_model.getAng();

    OpenSoT::DefaultHumanoidStack DHS(_robot, 3e-3, _robot.zeros);

    // defining a stack composed of size two,
    // where the task of first priority is an aggregated of leftArm and rightArm,
    // rightArm contains a convexHull constraint;
    // the task at the second priority level is an aggregated of rightLeg and leftLeg,
    // and the stack is subject to bounds jointLimits and velocityLimits
    OpenSoT::AutoStack::Ptr autoStack = 
        ((DHS.leftArm + (DHS.rightArm << DHS.convexHull))
        / (DHS.rightLeg + DHS.leftLeg)) << DHS.jointLimits << DHS.velocityLimits;

    q += .01*q;
    _robot.updateiDyn3Model(q,true);

    // automatically update all tasks and constraints in the autostack
    autoStack->update(q);
}

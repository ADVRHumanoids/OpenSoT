#include <iCub/iDynTree/yarp_kdl.h>
#include <idynutils/idynutils.h>
#include <kdl/frames.hpp>
#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/utils/DefaultHumanoidStack.h>
#include <OpenSoT/utils/Previewer.h>
#include <OpenSoT/utils/PreviewerUtils.h>

/**
 * @brief The MyTrajGen class dummy trajectory generator:
 *        it creates a constant trajectory that lasts one second.
 *        The goal pose is with p=initialPose, R=I
 */
class MyTrajGen
{
    KDL::Frame f;
public:
    typedef boost::shared_ptr<MyTrajGen> Ptr;

    MyTrajGen(yarp::sig::Matrix yf) {
        bool res = YarptoKDL(yf, f);
        assert( res && "Error converting yarp::Matrix to kdl::Frame"); }
    KDL::Frame Pos(double time) { return f; }
    KDL::Twist Vel(double time) { return KDL::Twist(); }
    double Duration() { return 1.0; }
};

typedef OpenSoT::Previewer<MyTrajGen> Previewer;

int main(int argc, char **argv) {

    const double dT = 3e-3;
    iDynUtils _robot("coman",
                     std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.urdf",
                     std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.srdf");
    yarp::sig::Vector q = _robot.iDyn3_model.getAng();
    _robot.updateiDyn3Model(q,true);

    OpenSoT::DefaultHumanoidStack DHS(_robot, dT, q);

    // defining a stack composed of size two,
    // where the task of first priority is an aggregated of leftArm and rightArm,
    // rightArm contains a convexHull constraint;
    // the task at the second priority level is an aggregated of rightLeg and leftLeg,
    // and the stack is subject to bounds jointLimits and velocityLimits
    OpenSoT::AutoStack::Ptr autoStack = 
        ((DHS.leftArm + (DHS.rightArm << DHS.convexHull))
        / (DHS.rightLeg + DHS.leftLeg)) << DHS.jointLimits << DHS.velocityLimits;

    MyTrajGen::Ptr trajLeftArm(new MyTrajGen(DHS.leftArm->getActualPose()));
    MyTrajGen::Ptr trajRightArm(new MyTrajGen(DHS.rightArm->getActualPose()));

    Previewer::TrajectoryBindings bindings;
    bindings.push_back(Previewer::TrajBinding(trajLeftArm, DHS.leftArm));
    bindings.push_back(Previewer::TrajBinding(trajRightArm, DHS.rightArm));

    Previewer::Ptr previewer(new Previewer(dT, _robot, autoStack, bindings));

    Previewer::Results results;
    if(!previewer->check(1.0,3,&results))
    {
        std::cout << "Trajectory unfeasible!" << std::endl;

        std::cout << "Logged " << results.failures.size() << " failures:" << std::endl;
        for(unsigned int i = 0; i < results.failures.size(); ++i)
        {
            std::cout << "@t:" << results.failures[i].t
                      << " - " << Previewer::Results::reasonToString(results.failures[i].reason)
                      << std::endl;
        }
    }
    else
    {
        std::cout << "Logged " << results.trajectory.size() << " trajectory nodes:" << std::endl;
        for(unsigned int i = 0; i < results.trajectory.size(); ++i)
        {
            std::cout << "@t:" << results.trajectory[i].t
                      << " - " << results.trajectory[i].q.toString()
                      << std::endl;
        }
    }
    OpenSoT::PreviewerUtils::plotPreviewerErrors(results);
    OpenSoT::PreviewerUtils::plotPreviewerTrajectory(results);
}

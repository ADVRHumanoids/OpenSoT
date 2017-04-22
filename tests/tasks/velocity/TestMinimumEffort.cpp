#include <gtest/gtest.h>
#include <OpenSoT/tasks/velocity/MinimumEffort.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>
#include <ModelInterfaceIDYNUTILS/ModelInterfaceIDYNUTILS.h>
#include <boost/make_shared.hpp>
#include <advr_humanoids_common_utils/conversion_utils_YARP.h>

using namespace yarp::math;

namespace {

class testMinimumEffortTask: public ::testing::Test
{
public:
    typedef idynutils2 iDynUtils;
    static void null_deleter(iDynUtils *) {}
protected:
    iDynUtils _robot;
    XBot::ModelInterfaceIDYNUTILS::Ptr _model_ptr;
    std::string _path_to_cfg;
    int nJ;

    testMinimumEffortTask() : _robot("coman",
                                     std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.urdf",
                                     std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.srdf")
    {
        nJ = _robot.iDynTree_model.getNrOfDOFs();
        std::string robotology_root = std::getenv("ROBOTOLOGY_ROOT");
        std::string relative_path = "/external/OpenSoT/tests/configs/coman/configs/config_coman.yaml";

        _path_to_cfg = robotology_root + relative_path;

        _model_ptr = std::dynamic_pointer_cast<XBot::ModelInterfaceIDYNUTILS>
                (XBot::ModelInterface::getModel(_path_to_cfg));
        _model_ptr->loadModel(boost::shared_ptr<iDynUtils>(&_robot, &null_deleter));

        if(_model_ptr)
            std::cout<<"pointer address: "<<_model_ptr.get()<<std::endl;
        else
            std::cout<<"pointer is NULL "<<_model_ptr.get()<<std::endl;
    }

    virtual ~testMinimumEffortTask() {

    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }

};

TEST_F(testMinimumEffortTask, testMinimumEffortTask_)
{
    // setting initial position with bent legs
    yarp::sig::Vector q_whole(nJ, 1E-2);
    q_whole[_robot.iDynTree_model.getDOFIndex("RHipSag")] = -25.0*M_PI/180.0;
    q_whole[_robot.iDynTree_model.getDOFIndex("RKneeSag")] = 50.0*M_PI/180.0;
    q_whole[_robot.iDynTree_model.getDOFIndex("RAnkSag")] = -25.0*M_PI/180.0;
    q_whole[_robot.iDynTree_model.getDOFIndex("LHipSag")] = -25.0*M_PI/180.0;
    q_whole[_robot.iDynTree_model.getDOFIndex("LKneeSag")] = 50.0*M_PI/180.0;
    q_whole[_robot.iDynTree_model.getDOFIndex("LAnkSag")] = -25.0*M_PI/180.0;


    _robot.updateiDynTreeModel(conversion_utils_YARP::toEigen(q_whole));

    OpenSoT::tasks::velocity::MinimumEffort minimumEffort(
                conversion_utils_YARP::toEigen(q_whole), *(_model_ptr.get()));

    EXPECT_EQ(minimumEffort.getA().rows(), nJ);
    EXPECT_EQ(minimumEffort.getb().size(), nJ);

    EXPECT_TRUE(minimumEffort.getWeight().rows() == nJ);
    EXPECT_TRUE(minimumEffort.getWeight().cols() == nJ);

    EXPECT_TRUE(minimumEffort.getConstraints().size() == 0);

    double K = 0.1;//0.8;
    minimumEffort.setLambda(K);
    EXPECT_DOUBLE_EQ(minimumEffort.getLambda(), K);
    _robot.updateiDynTreeModel(conversion_utils_YARP::toEigen(q_whole), true);
    double initial_effort = yarp::math::dot(_robot.iDynTree_model.getTorques(),
                                    conversion_utils_YARP::toYARP(minimumEffort.getWeight())*_robot.iDynTree_model.getTorques());
    std::cout<<"Initial Effort: "<<initial_effort<<std::endl;
    double initial_effort2 = minimumEffort.computeEffort();
    std::cout<<"Initial Effort2: "<<initial_effort2<<std::endl;
    for(unsigned int i = 0; i < 25; ++i)
    {
        minimumEffort.update(conversion_utils_YARP::toEigen(q_whole));
        double old_effort = minimumEffort.computeEffort();

        q_whole += pinv(conversion_utils_YARP::toYARP(minimumEffort.getA()),1E-6)
                *minimumEffort.getLambda()*conversion_utils_YARP::toYARP(minimumEffort.getb());

        minimumEffort.update(conversion_utils_YARP::toEigen(q_whole));
        EXPECT_LE(minimumEffort.computeEffort(), old_effort);
        std::cout << "Effort at step" << i << ": " << minimumEffort.computeEffort() << std::endl;

    }
    _robot.updateiDynTreeModel(conversion_utils_YARP::toEigen(q_whole), true);
    double final_effort = yarp::math::dot(_robot.iDynTree_model.getTorques(),
                                    conversion_utils_YARP::toYARP(minimumEffort.getWeight())*_robot.iDynTree_model.getTorques());
    EXPECT_LT(final_effort, initial_effort);
}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

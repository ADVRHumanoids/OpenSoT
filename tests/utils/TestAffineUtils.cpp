#include <OpenSoT/utils/Affine.h>
#include <OpenSoT/utils/AffineUtils.h>
#include <gtest/gtest.h>
#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <boost/make_shared.hpp>

std::string robotology_root = std::getenv("ROBOTOLOGY_ROOT");
std::string relative_path = "/external/OpenSoT/tests/configs/coman/configs/config_coman_floating_base.yaml";
std::string path_to_cfg = robotology_root + relative_path;

namespace{

class testAffineUtils: public ::testing::Test
{
protected:

    testAffineUtils()
    {
        model_ptr = XBot::ModelInterface::getModel(path_to_cfg);
        q.setZero(model_ptr->getJointNum());

        q[model_ptr->getDofIndex("RHipSag")] = -25.0*M_PI/180.0;
        q[model_ptr->getDofIndex("RKneeSag")] = 50.0*M_PI/180.0;
        q[model_ptr->getDofIndex("RAnkSag")] = -25.0*M_PI/180.0;
        q[model_ptr->getDofIndex("LHipSag")] = -25.0*M_PI/180.0;
        q[model_ptr->getDofIndex("LKneeSag")] = 50.0*M_PI/180.0;
        q[model_ptr->getDofIndex("LAnkSag")] = -25.0*M_PI/180.0;
        q[model_ptr->getDofIndex("LShSag")] =  20.0*M_PI/180.0;
        q[model_ptr->getDofIndex("LShLat")] = 10.0*M_PI/180.0;
        q[model_ptr->getDofIndex("LElbj")] = -80.0*M_PI/180.0;

        model_ptr->setJointPosition(q);
        model_ptr->update();
    }

    virtual ~testAffineUtils() {

    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }

    XBot::ModelInterface::Ptr model_ptr;
    Eigen::VectorXd q;

};

TEST_F(testAffineUtils, testCartesianTaskToAffine)
{
    //1. Creates a Cartesian velocity Task
    OpenSoT::tasks::velocity::Cartesian::Ptr task =
            boost::make_shared<OpenSoT::tasks::velocity::Cartesian>(
                "LSOLE", this->q, *(this->model_ptr), "l_sole", "world");

    //2. Creates variables
    std::vector<std::pair<std::string, int>> name_size_pairs;

    name_size_pairs.emplace_back("dq", this->q.size());
    int slack_size = 6;
    name_size_pairs.emplace_back("slack", slack_size);
    OpenSoT::OptvarHelper opt_helper(name_size_pairs);

    //3. Creates Affine Task
    OpenSoT::AffineUtils::AffineTask::Ptr affine_task =
            OpenSoT::AffineUtils::AffineTask::toAffine(task, opt_helper.getVariable("dq"));

    EXPECT_EQ(affine_task->getA().rows(), task->getA().rows());
    EXPECT_EQ(affine_task->getA().cols(), task->getA().cols() + slack_size);
    EXPECT_EQ(affine_task->getb(), task->getb());
    EXPECT_EQ(affine_task->getWeight(), task->getWeight());
    EXPECT_EQ(affine_task->getA().block(0,0,task->getA().rows(),task->getA().cols()), task->getA());

    std::cout<<"affine_task->getb(): "<<affine_task->getb().transpose()<<std::endl;
    std::cout<<"task->getb(): "<<task->getb().transpose()<<std::endl;

    //4. Update q and model
    this->q[model_ptr->getDofIndex("RHipSag")] = -15.0*M_PI/180.0;
    this->q[model_ptr->getDofIndex("LHipSag")] = -15.0*M_PI/180.0;
    this->model_ptr->setJointPosition(this->q);
    this->model_ptr->update();

    //5. Update task reference
    Eigen::Affine3d T = Eigen::Affine3d::Identity();
    task->setReference(T);

    affine_task->update(Eigen::VectorXd(1));

    Eigen::Affine3d T_ref; task->getReference(T_ref);
    EXPECT_EQ(T.matrix(), T_ref.matrix());
    EXPECT_EQ(affine_task->getA().rows(), task->getA().rows());
    EXPECT_EQ(affine_task->getA().cols(), task->getA().cols() + slack_size);
    EXPECT_EQ(affine_task->getb(), task->getb());
    EXPECT_EQ(affine_task->getWeight(), task->getWeight());
    EXPECT_EQ(affine_task->getA().block(0,0,task->getA().rows(),task->getA().cols()), task->getA());
    std::cout<<"affine_task->getb(): "<<affine_task->getb().transpose()<<std::endl;
    std::cout<<"task->getb(): "<<task->getb().transpose()<<std::endl;

}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


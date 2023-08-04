#include <OpenSoT/utils/Affine.h>
#include <OpenSoT/utils/AffineUtils.h>
#include <gtest/gtest.h>
#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <OpenSoT/constraints/velocity/VelocityLimits.h>
#include <OpenSoT/constraints/velocity/CartesianVelocity.h>
#include <memory>

std::string relative_path = OPENSOT_TEST_PATH "configs/coman/configs/config_coman_floating_base.yaml";
std::string path_to_cfg = relative_path;

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

TEST_F(testAffineUtils, testConstraintsToAffine)
{
    //1. Creates a CoM Limits constraint
    Eigen::Vector3d com_vel_lims;
    com_vel_lims<<0.1, 0.1, 0.1;
    OpenSoT::constraints::velocity::CartesianVelocity::Ptr constraint =
            std::make_shared<OpenSoT::constraints::velocity::CartesianVelocity>(
                com_vel_lims, 0.01, std::make_shared<OpenSoT::tasks::velocity::CoM>(this->q, *this->model_ptr.get()));

    //2. Creates variables
    std::vector<std::pair<std::string, int>> name_size_pairs;

    name_size_pairs.emplace_back("dq", this->q.size());
    int slack_size = 6;
    name_size_pairs.emplace_back("slack", slack_size);
    OpenSoT::OptvarHelper opt_helper(name_size_pairs);

    //3. Creates Affine Constraint
    OpenSoT::AffineUtils::AffineConstraint::Ptr affine_constraint =
            OpenSoT::AffineUtils::AffineConstraint::toAffine(constraint, opt_helper.getVariable("dq"));

    EXPECT_EQ(affine_constraint->getAineq().rows(), constraint->getAineq().rows());
    EXPECT_EQ(affine_constraint->getAineq().cols(), constraint->getAineq().cols() + slack_size);
    EXPECT_EQ(affine_constraint->getbLowerBound(), constraint->getbLowerBound());
    EXPECT_EQ(affine_constraint->getbUpperBound(), constraint->getbUpperBound());
    EXPECT_EQ(affine_constraint->getAineq().block(0,0,constraint->getAineq().rows(),constraint->getAineq().cols()),
              constraint->getAineq());

    //4. Update q and model
    this->q[model_ptr->getDofIndex("RHipSag")] = -15.0*M_PI/180.0;
    this->q[model_ptr->getDofIndex("LHipSag")] = -15.0*M_PI/180.0;
    this->model_ptr->setJointPosition(this->q);
    this->model_ptr->update();

    affine_constraint->update(this->q);
    EXPECT_EQ(affine_constraint->getAineq().rows(), constraint->getAineq().rows());
    EXPECT_EQ(affine_constraint->getAineq().cols(), constraint->getAineq().cols() + slack_size);
    EXPECT_EQ(affine_constraint->getbLowerBound(), constraint->getbLowerBound());
    EXPECT_EQ(affine_constraint->getbUpperBound(), constraint->getbUpperBound());
    EXPECT_EQ(affine_constraint->getAineq().block(0,0,constraint->getAineq().rows(),constraint->getAineq().cols()),
              constraint->getAineq());

}

TEST_F(testAffineUtils, testBoundsToAffine)
{
    //1. Creates a Velocity Limits constraint
    OpenSoT::constraints::velocity::VelocityLimits::Ptr bound =
            std::make_shared<OpenSoT::constraints::velocity::VelocityLimits>(
                M_PI, 0.01, this->model_ptr->getJointNum());

    //2. Creates variables
    std::vector<std::pair<std::string, int>> name_size_pairs;

    name_size_pairs.emplace_back("dq", this->q.size());
    int slack_size = 6;
    name_size_pairs.emplace_back("slack", slack_size);
    OpenSoT::OptvarHelper opt_helper(name_size_pairs);

    //3. Creates Affine Constraint
    OpenSoT::AffineUtils::AffineConstraint::Ptr affine_bound =
            OpenSoT::AffineUtils::AffineConstraint::toAffine(bound, opt_helper.getVariable("dq"));

    EXPECT_EQ(affine_bound->getAineq().rows(), bound->getLowerBound().rows());
    EXPECT_EQ(affine_bound->getAineq().cols(), bound->getLowerBound().rows() + slack_size);
    EXPECT_EQ(affine_bound->getbLowerBound(), bound->getLowerBound());
    EXPECT_EQ(affine_bound->getbUpperBound(), bound->getUpperBound());

    //4. Update bound
    bound->setVelocityLimits(M_PI/4.);
    bound->update(this->q);
    affine_bound->update(this->q);

    EXPECT_EQ(affine_bound->getAineq().rows(), bound->getLowerBound().rows());
    EXPECT_EQ(affine_bound->getAineq().cols(), bound->getLowerBound().rows() + slack_size);
    EXPECT_EQ(affine_bound->getbLowerBound(), bound->getLowerBound());
    EXPECT_EQ(affine_bound->getbUpperBound(), bound->getUpperBound());

    std::cout<<"affine_bound->getbLowerBound(): "<<affine_bound->getbLowerBound().transpose()<<std::endl;
    std::cout<<"bound->getLowerBound(): "<<bound->getLowerBound().transpose()<<std::endl;
}

TEST_F(testAffineUtils, testCartesianTaskToAffine)
{
    //1. Creates a Cartesian velocity Task
    OpenSoT::tasks::velocity::Cartesian::Ptr task =
            std::make_shared<OpenSoT::tasks::velocity::Cartesian>(
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


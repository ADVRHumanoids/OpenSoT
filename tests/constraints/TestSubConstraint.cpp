#include <gtest/gtest.h>
#include <OpenSoT/SubConstraint.h>
#include <OpenSoT/constraints/velocity/JointLimits.h>
#include <XBotInterface/ModelInterface.h>
#include <OpenSoT/utils/AutoStack.h>

std::string relative_path = OPENSOT_TEST_PATH "configs/coman/configs/config_coman_RBDL.yaml";
std::string _path_to_cfg = relative_path;

using namespace OpenSoT::constraints;

namespace{
class TestSubConstraint: public ::testing::Test
{
protected:

    OpenSoT::constraints::velocity::JointLimits::Ptr _joint_limits;

    TestSubConstraint()

    {
        _model_ptr = XBot::ModelInterface::getModel(_path_to_cfg);

        if(_model_ptr)
            std::cout<<"pointer address: "<<_model_ptr.get()<<std::endl;
        else
            std::cout<<"pointer is NULL "<<_model_ptr.get()<<std::endl;

        q.resize(_model_ptr->getJointNum());
        q.setZero();
        Eigen::VectorXd qmin(_model_ptr->getJointNum()), qmax(_model_ptr->getJointNum());
        _model_ptr->getJointLimits(qmin, qmax);
        _joint_limits = std::make_shared<velocity::JointLimits>(q, qmax, qmin);

    }

    virtual ~TestSubConstraint() {
    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }

    XBot::ModelInterface::Ptr _model_ptr;
    Eigen::VectorXd q;

};

TEST_F(TestSubConstraint, testSubBounds)
{
    std::list<unsigned int> indices;
    for(unsigned int i = 6; i < this->_model_ptr->getJointNum(); ++i)
        indices.push_back(i);

    OpenSoT::SubConstraint::Ptr sub_postural = std::make_shared<OpenSoT::SubConstraint>(this->_joint_limits, indices);

    sub_postural->update(this->q);

    std::cout<<"lower bound[:]: "<<this->_joint_limits->getLowerBound().transpose()<<std::endl;
    std::cout<<"lower bound[6:end]: "<<sub_postural->getLowerBound().transpose()<<std::endl;

    std::cout<<"upper bound[:]: "<<this->_joint_limits->getUpperBound().transpose()<<std::endl;
    std::cout<<"upper bound[6:end]: "<<sub_postural->getUpperBound().transpose()<<std::endl;

    EXPECT_EQ(sub_postural->getLowerBound().size(), indices.size());
    EXPECT_EQ(sub_postural->getUpperBound().size(), indices.size());
    EXPECT_EQ(sub_postural->getLowerBound(), this->_joint_limits->getLowerBound().segment(6, indices.size()));
    EXPECT_EQ(sub_postural->getUpperBound(), this->_joint_limits->getUpperBound().segment(6, indices.size()));

    this->q.setRandom();

    sub_postural->update(this->q);
    std::cout<<"update q..."<<std::endl;

    std::cout<<"lower bound[:]: "<<this->_joint_limits->getLowerBound().transpose()<<std::endl;
    std::cout<<"lower bound[6:end]: "<<sub_postural->getLowerBound().transpose()<<std::endl;

    std::cout<<"upper bound[:]: "<<this->_joint_limits->getUpperBound().transpose()<<std::endl;
    std::cout<<"upper bound[6:end]: "<<sub_postural->getUpperBound().transpose()<<std::endl;

    EXPECT_EQ(sub_postural->getLowerBound(), this->_joint_limits->getLowerBound().segment(6, indices.size()));
    EXPECT_EQ(sub_postural->getUpperBound(), this->_joint_limits->getUpperBound().segment(6, indices.size()));

}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


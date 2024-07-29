#include <gtest/gtest.h>
#include <OpenSoT/SubConstraint.h>
#include <OpenSoT/constraints/velocity/JointLimits.h>
#include <OpenSoT/constraints/velocity/CartesianVelocity.h>
#include <xbot2_interface/xbotinterface2.h>
#include <OpenSoT/utils/AutoStack.h>
#include "../common.h"

#define toRad(X) (X * M_PI/180.0)

using namespace OpenSoT::constraints;

namespace{
class TestSubConstraint: public TestBase
{
protected:

    OpenSoT::constraints::velocity::JointLimits::Ptr _joint_limits;
    OpenSoT::constraints::velocity::CartesianVelocity::Ptr _vcom_constraint;

    TestSubConstraint() : TestBase("coman_floating_base")

    {
        q = _model_ptr->getNeutralQ();
        q[_model_ptr->getQIndex("LHipSag")] = toRad(-23.5);
        q[_model_ptr->getQIndex("LHipLat")] = toRad(2.0);
        q[_model_ptr->getQIndex("LHipYaw")] = toRad(-4.0);
        q[_model_ptr->getQIndex("LKneeSag")] = toRad(50.1);
        q[_model_ptr->getQIndex("LAnkLat")] = toRad(-2.0);
        q[_model_ptr->getQIndex("LAnkSag")] = toRad(-26.6);

        q[_model_ptr->getQIndex("RHipSag")] = toRad(-23.5);
        q[_model_ptr->getQIndex("RHipLat")] = toRad(-2.0);
        q[_model_ptr->getQIndex("RHipYaw")] = toRad(0.0);
        q[_model_ptr->getQIndex("RKneeSag")] = toRad(50.1);
        q[_model_ptr->getQIndex("RAnkLat")] = toRad(2.0);
        q[_model_ptr->getQIndex("RAnkSag")] = toRad(-26.6);


        Eigen::VectorXd qmin, qmax;
        _model_ptr->getJointLimits(qmin, qmax);
        _joint_limits = std::make_shared<velocity::JointLimits>(*_model_ptr, qmax, qmin);

        Eigen::Vector3d vcom_max;
        vcom_max<<10., 20., 30.;
        _vcom_constraint = std::make_shared<velocity::CartesianVelocity>(vcom_max, 0.01,
                                                                         std::make_shared<OpenSoT::tasks::velocity::CoM>(*_model_ptr));

    }

    virtual ~TestSubConstraint() {
    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }

    Eigen::VectorXd q;

};

TEST_F(TestSubConstraint, testSubBounds)
{
    std::list<unsigned int> indices;
    for(unsigned int i = 6; i < this->_model_ptr->getNv(); ++i)
        indices.push_back(i);

    OpenSoT::SubConstraint::Ptr sub_postural = this->_joint_limits%indices;

    sub_postural->update();

    std::cout<<"lower bound[:]: "<<this->_joint_limits->getLowerBound().transpose()<<std::endl;
    std::cout<<"lower bound[6:end]: "<<sub_postural->getbLowerBound().transpose()<<std::endl;

    std::cout<<"upper bound[:]: "<<this->_joint_limits->getUpperBound().transpose()<<std::endl;
    std::cout<<"upper bound[6:end]: "<<sub_postural->getbUpperBound().transpose()<<std::endl;

    std::cout<<"Aineq: \n"<<sub_postural->getAineq()<<std::endl;

    EXPECT_EQ(sub_postural->getbLowerBound().size(), indices.size());
    EXPECT_EQ(sub_postural->getbUpperBound().size(), indices.size());
    EXPECT_EQ(sub_postural->getbLowerBound(), this->_joint_limits->getLowerBound().segment(6, indices.size()));
    EXPECT_EQ(sub_postural->getbUpperBound(), this->_joint_limits->getUpperBound().segment(6, indices.size()));
    EXPECT_EQ(sub_postural->getAineq().rows(), indices.size());
    EXPECT_EQ(sub_postural->getAineq().cols(), this->_joint_limits->getLowerBound().size());
    EXPECT_EQ(sub_postural->getAineq().rightCols(indices.size()), Eigen::MatrixXd(indices.size(), indices.size()).setIdentity());

    this->q = _model_ptr->generateRandomQ();

    sub_postural->update();
    std::cout<<"lower bound[:]: "<<this->_joint_limits->getLowerBound().transpose()<<std::endl;
    std::cout<<"lower bound[6:end]: "<<sub_postural->getbLowerBound().transpose()<<std::endl;

    std::cout<<"upper bound[:]: "<<this->_joint_limits->getUpperBound().transpose()<<std::endl;
    std::cout<<"upper bound[6:end]: "<<sub_postural->getbUpperBound().transpose()<<std::endl;

    std::cout<<"Aineq: \n"<<sub_postural->getAineq()<<std::endl;

    EXPECT_EQ(sub_postural->getbLowerBound().size(), indices.size());
    EXPECT_EQ(sub_postural->getbUpperBound().size(), indices.size());
    EXPECT_EQ(sub_postural->getbLowerBound(), this->_joint_limits->getLowerBound().segment(6, indices.size()));
    EXPECT_EQ(sub_postural->getbUpperBound(), this->_joint_limits->getUpperBound().segment(6, indices.size()));
    EXPECT_EQ(sub_postural->getAineq().rows(), indices.size());
    EXPECT_EQ(sub_postural->getAineq().cols(), this->_joint_limits->getLowerBound().size());
    EXPECT_EQ(sub_postural->getAineq().rightCols(indices.size()), Eigen::MatrixXd(indices.size(), indices.size()).setIdentity());
}

TEST_F(TestSubConstraint, testSubInequalityConstraint)
{
    std::list<unsigned int> indices = {0,1};

    OpenSoT::SubConstraint::Ptr sub_vcom = this->_vcom_constraint%indices;

    sub_vcom->update();

    std::cout<<"lower bound[:]: "<<this->_vcom_constraint->getbLowerBound().transpose()<<std::endl;
    std::cout<<"lower bound[0:1]: "<<sub_vcom->getbLowerBound().transpose()<<std::endl;

    std::cout<<"upper bound[:]: "<<this->_vcom_constraint->getbUpperBound().transpose()<<std::endl;
    std::cout<<"upper bound[0:1]: "<<sub_vcom->getbUpperBound().transpose()<<std::endl;

    std::cout<<"Aineq[:,:]: \n"<<this->_vcom_constraint->getAineq()<<std::endl;
    std::cout<<"Aineq[0:1,:]: \n"<<sub_vcom->getAineq()<<std::endl;

    EXPECT_EQ(sub_vcom->getbLowerBound().size(), indices.size());
    EXPECT_EQ(sub_vcom->getbUpperBound().size(), indices.size());
    EXPECT_EQ(sub_vcom->getAineq().rows(), indices.size());
    EXPECT_EQ(sub_vcom->getAineq().cols(), this->_model_ptr->getNv());

    EXPECT_EQ(sub_vcom->getbLowerBound(), this->_vcom_constraint->getbLowerBound().segment(0, indices.size()));
    EXPECT_EQ(sub_vcom->getbUpperBound(), this->_vcom_constraint->getbUpperBound().segment(0, indices.size()));
    EXPECT_EQ(sub_vcom->getAineq(), this->_vcom_constraint->getAineq().block(0,0,2,2));

    this->q = _model_ptr->generateRandomQ();

    sub_vcom->update();
    std::cout<<"update q..."<<std::endl;

    std::cout<<"lower bound[:]: "<<this->_vcom_constraint->getbLowerBound().transpose()<<std::endl;
    std::cout<<"lower bound[0:1]: "<<sub_vcom->getbLowerBound().transpose()<<std::endl;

    std::cout<<"upper bound[:]: "<<this->_vcom_constraint->getbUpperBound().transpose()<<std::endl;
    std::cout<<"upper bound[0:1]: "<<sub_vcom->getbUpperBound().transpose()<<std::endl;

    std::cout<<"Aineq[:,:]: \n"<<this->_vcom_constraint->getAineq()<<std::endl;
    std::cout<<"Aineq[0:1,:]: \n"<<sub_vcom->getAineq()<<std::endl;

    EXPECT_EQ(sub_vcom->getbLowerBound().size(), indices.size());
    EXPECT_EQ(sub_vcom->getbUpperBound().size(), indices.size());
    EXPECT_EQ(sub_vcom->getAineq().rows(), indices.size());
    EXPECT_EQ(sub_vcom->getAineq().cols(), this->_model_ptr->getNv());

    EXPECT_EQ(sub_vcom->getbLowerBound(), this->_vcom_constraint->getbLowerBound().segment(0, indices.size()));
    EXPECT_EQ(sub_vcom->getbUpperBound(), this->_vcom_constraint->getbUpperBound().segment(0, indices.size()));
    EXPECT_EQ(sub_vcom->getAineq(), this->_vcom_constraint->getAineq().block(0,0,2,2));

}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


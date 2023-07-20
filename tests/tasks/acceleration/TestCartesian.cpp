#include <OpenSoT/tasks/acceleration/Cartesian.h>
#include <OpenSoT/tasks/acceleration/Postural.h>
#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/solvers/iHQP.h>
#include <gtest/gtest.h>
#include <XBotInterface/ModelInterface.h>
#include <OpenSoT/solvers/eHQP.h>

std::string relative_path = OPENSOT_TEST_PATH "configs/coman/configs/config_coman_floating_base.yaml";
std::string _path_to_cfg = relative_path;

namespace{

class testCartesianTask: public ::testing::Test
{
protected:

    testCartesianTask()
    {
        _model = XBot::ModelInterface::getModel(_path_to_cfg);
        _q.setZero(_model->getJointNum());
        setGoodInitialPosition();
        _dq.setZero(_model->getJointNum());

        _model->setJointPosition(_q);
        _model->setJointVelocity(_dq);
        _model->update();

        KDL::Frame l_sole_T_Waist;
        _model->getPose("Waist", "l_sole", l_sole_T_Waist);

        l_sole_T_Waist.p.x(0.0);
        l_sole_T_Waist.p.y(0.0);

        this->setWorld(l_sole_T_Waist, _q);


        l_arm.reset(
            new OpenSoT::tasks::acceleration::Cartesian("l_arm",_q, *_model, "LSoftHand", "world"));
        l_arm->getReference(_l_arm_ref);
        std::cout<<"_l_arm_initial: "<<_l_arm_ref.matrix()<<std::endl;
        r_arm.reset(
            new OpenSoT::tasks::acceleration::Cartesian("r_arm",_q, *_model, "RSoftHand", "world"));
        r_arm->getReference(_r_arm_ref);
        std::cout<<"_r_arm_initial: "<<_r_arm_ref.matrix()<<std::endl;
        OpenSoT::tasks::acceleration::Postural::Ptr postural(new
            OpenSoT::tasks::acceleration::Postural(*_model,_q.size()));

        autostack = (l_arm + r_arm)/(postural);
        autostack->update(_q);

        eHQP.reset(new OpenSoT::solvers::eHQP(autostack->getStack()));

    }

    virtual ~testCartesianTask() {

    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }

    void setWorld(const KDL::Frame& l_sole_T_Waist, Eigen::VectorXd& q)
    {
        _model->setFloatingBasePose(l_sole_T_Waist);
        _model->update();
        _model->getJointPosition(q);
    }

    void setGoodInitialPosition() {
        _q[_model->getDofIndex("RHipSag")] = -25.0*M_PI/180.0;
        _q[_model->getDofIndex("RKneeSag")] = 50.0*M_PI/180.0;
        _q[_model->getDofIndex("RAnkSag")] = -25.0*M_PI/180.0;

        _q[_model->getDofIndex("LHipSag")] = -25.0*M_PI/180.0;
        _q[_model->getDofIndex("LKneeSag")] = 50.0*M_PI/180.0;
        _q[_model->getDofIndex("LAnkSag")] = -25.0*M_PI/180.0;

        _q[_model->getDofIndex("LShSag")] =  20.0*M_PI/180.0;
        _q[_model->getDofIndex("LShLat")] = 20.0*M_PI/180.0;
        _q[_model->getDofIndex("LShYaw")] = -15.0*M_PI/180.0;
        _q[_model->getDofIndex("LElbj")] = -80.0*M_PI/180.0;

        _q[_model->getDofIndex("RShSag")] =  20.0*M_PI/180.0;
        _q[_model->getDofIndex("RShLat")] = -20.0*M_PI/180.0;
        _q[_model->getDofIndex("RShYaw")] = 15.0*M_PI/180.0;
        _q[_model->getDofIndex("RElbj")] = -80.0*M_PI/180.0;

    }

    Eigen::VectorXd _q, _dq;
    XBot::ModelInterface::Ptr _model;
    Eigen::Affine3d _l_arm_ref, _r_arm_ref;
    OpenSoT::tasks::acceleration::Cartesian::Ptr l_arm, r_arm;
    OpenSoT::AutoStack::Ptr autostack;
    OpenSoT::solvers::eHQP::Ptr eHQP;
};

TEST_F(testCartesianTask, testCartesianTask_)
{
    _r_arm_ref.matrix()(1,3) += 0.1;
    std::cout<<"_r_arm_ref: "<<_r_arm_ref.matrix()<<std::endl;
    this->r_arm->setReference(_r_arm_ref);

    double dt = 0.001;
    Eigen::VectorXd ddq(_q.size()); ddq.setZero(ddq.size());
    for(unsigned int i = 0; i < 1000; ++i)
    {
        _model->setJointPosition(_q);
        _model->setJointVelocity(_dq);
        _model->update();

        autostack->update(_q);

        if(!(eHQP->solve(ddq)))
            std::cout<<"CAN NOT SOLVE"<<std::endl;

        _dq += ddq*dt;
        _q += _dq*dt + 0.5*ddq*dt*dt;
    }

    Eigen::Affine3d r_actual, l_actual;
    this->r_arm->getActualPose(r_actual);
    this->l_arm->getActualPose(l_actual);
    std::cout<<"r_actual: "<<r_actual.matrix()<<std::endl;
    std::cout<<"l_actual: "<<l_actual.matrix()<<std::endl;

    for(unsigned int i = 0; i < 4; ++i)
    {
        for(unsigned int j = 0; j < 4; ++j)
            EXPECT_LE(r_actual.matrix()(i,j) - _r_arm_ref.matrix()(i,j), 1e-4);
    }

    for(unsigned int i = 0; i < 4; ++i)
    {
        for(unsigned int j = 0; j < 4; ++j)
            EXPECT_LE(l_actual.matrix()(i,j) - _l_arm_ref.matrix()(i,j), 1e-4);
    }
}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


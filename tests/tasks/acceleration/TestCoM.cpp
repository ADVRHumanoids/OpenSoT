#include <OpenSoT/tasks/acceleration/Cartesian.h>
#include <OpenSoT/tasks/acceleration/Postural.h>
#include <OpenSoT/tasks/acceleration/CoM.h>
#include <OpenSoT/tasks/acceleration/CoM.h>
#include <OpenSoT/constraints/GenericConstraint.h>
#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/solvers/iHQP.h>
#include <gtest/gtest.h>
#include <XBotInterface/ModelInterface.h>
#include <OpenSoT/solvers/eHQP.h>
#include <OpenSoT/constraints/velocity/VelocityLimits.h>

std::string robotology_root = std::getenv("ROBOTOLOGY_ROOT");
std::string relative_path = "/external/OpenSoT/tests/configs/coman/configs/config_coman_floating_base.yaml";
std::string _path_to_cfg = robotology_root + relative_path;

namespace{

class testCoMTask: public ::testing::Test
{
protected:

    testCoMTask()
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


        com.reset(
            new OpenSoT::tasks::acceleration::CoM(*_model, _q));
        com->getReference(_com_ref);
        std::cout<<"_com_initial: \n"<<_com_ref.matrix()<<std::endl;
        l_sole.reset(
            new OpenSoT::tasks::acceleration::Cartesian("l_sole", *_model, "l_sole", "world", _q));
        l_sole->getReference(_l_sole_ref);
        std::cout<<"_l_sole_initial: \n"<<_l_sole_ref.matrix()<<std::endl;
        r_sole.reset(
            new OpenSoT::tasks::acceleration::Cartesian("r_sole", *_model, "r_sole", "world", _q));
        r_sole->getReference(_r_sole_ref);
        std::cout<<"_r_sole_initial: \n"<<_r_sole_ref.matrix()<<std::endl;
        OpenSoT::tasks::acceleration::Postural::Ptr postural(new
            OpenSoT::tasks::acceleration::Postural(*_model,_q.size()));


        OpenSoT::AffineHelper var = OpenSoT::AffineHelper::Identity(_q.size());
        Eigen::VectorXd ddq_max = 1.*Eigen::VectorXd::Ones(_q.size());
        Eigen::VectorXd ddq_min = -ddq_max;
        acc_lims.reset(
            new OpenSoT::constraints::GenericConstraint("acc_lims", var, ddq_max, ddq_min,
                                                        OpenSoT::constraints::GenericConstraint::Type::BOUND));





        autostack = ((l_sole + r_sole)/(com)/(postural))<<acc_lims;
        autostack->update(_q);

        iHQP.reset(new OpenSoT::solvers::iHQP(autostack->getStack(), autostack->getBounds()));

    }

    virtual ~testCoMTask() {

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
    Eigen::Affine3d _l_sole_ref, _r_sole_ref;
    Eigen::Vector3d _com_ref;
    OpenSoT::tasks::acceleration::Cartesian::Ptr l_sole, r_sole;
    OpenSoT::tasks::acceleration::CoM::Ptr com;
    OpenSoT::constraints::GenericConstraint::Ptr acc_lims;
    OpenSoT::AutoStack::Ptr autostack;
    OpenSoT::solvers::iHQP::Ptr iHQP;
};

TEST_F(testCoMTask, testCoMTask_)
{
    _com_ref[2] -= 0.1;
    std::cout<<"_com_ref: \n"<<_com_ref<<std::endl;
    std::cout<<"_r_sole_ref: \n"<<_r_sole_ref.matrix()<<std::endl;
    std::cout<<"_l_sole_ref: \n"<<_l_sole_ref.matrix()<<std::endl;
    this->com->setReference(_com_ref);

    double dt = 0.001;
    Eigen::VectorXd ddq(_q.size());
    ddq.setZero(ddq.size());
    std::cout<<"q: "<<_q<<std::endl;
    for(unsigned int i = 0; i < 10000; ++i)
    {

        this->_model->setJointPosition(_q);
        this->_model->setJointVelocity(_dq);
        this->_model->update();

        this->autostack->update(_q);


        if(!(iHQP->solve(ddq)))
            std::cout<<"CAN NOT SOLVE"<<std::endl;

        _dq += ddq*dt;
        _q += _dq*dt + 0.5*ddq*dt*dt;
    }

    Eigen::Affine3d r_actual, l_actual;
    this->r_sole->getActualPose(r_actual);
    this->l_sole->getActualPose(l_actual);
    std::cout<<"r_actual: \n"<<r_actual.matrix()<<std::endl;
    std::cout<<"l_actual: \n"<<l_actual.matrix()<<std::endl;

    for(unsigned int i = 0; i < 4; ++i)
    {
        for(unsigned int j = 0; j < 4; ++j)
            EXPECT_LE(r_actual.matrix()(i,j) - _r_sole_ref.matrix()(i,j), 1e-4);
    }

    for(unsigned int i = 0; i < 4; ++i)
    {
        for(unsigned int j = 0; j < 4; ++j)
            EXPECT_LE(l_actual.matrix()(i,j) - _l_sole_ref.matrix()(i,j), 1e-4);
    }

    Eigen::Vector3d com_actual;
    this->com->getActualPose(com_actual);
    std::cout<<"com_actual \n"<<com_actual<<std::endl;

    for(unsigned int i = 0; i < 3; ++i)
        EXPECT_LE(com_actual[i] - _com_ref[i], 1e-4);
}


}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

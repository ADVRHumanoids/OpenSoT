#include <gtest/gtest.h>
#include <OpenSoT/tasks/velocity/Postural.h>
#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <OpenSoT/tasks/velocity/CoM.h>
#include <OpenSoT/constraints/velocity/VelocityLimits.h>
#include <OpenSoT/tasks/floating_base/Contact.h>
#include <OpenSoT/solvers/iHQP.h>
#include <OpenSoT/SubTask.h>
#include <OpenSoT/utils/AutoStack.h>
#include <xbot2_interface/xbotinterface2.h>
#include <OpenSoT/constraints/GenericConstraint.h>
#include <OpenSoT/utils/Affine.h>
#include "../../common.h"


using namespace OpenSoT::tasks::velocity;
using namespace OpenSoT::constraints::velocity;


namespace name {

class testPosturalTask: public TestBase
{
protected:

    testPosturalTask() : TestBase("coman_floating_base")
    {
        dT = 0.002;


        q = _model_ptr->getNeutralQ();
        setGoodInitialPosition();
        dq.setZero(_model_ptr->getNv());

        _model_ptr->setJointPosition(q);
        _model_ptr->setJointVelocity(dq);
        _model_ptr->update();

        Eigen::Affine3d l_sole_T_Waist;
        _model_ptr->getPose("Waist", "l_sole", l_sole_T_Waist);

        l_sole_T_Waist.translation()[0] = 0.0;
        l_sole_T_Waist.translation()[1] = 0.0;

        this->setWorld(l_sole_T_Waist, q);

        l_sole.reset(new Cartesian("l_sole", *_model_ptr, "l_sole", "world"));
        r_sole.reset(new Cartesian("r_sole", *_model_ptr, "r_sole", "world"));
        com.reset(new CoM(*_model_ptr));
        postural.reset(new Postural(*_model_ptr));
        vel_lims.reset(new VelocityLimits(*_model_ptr, M_PI, dT));

        autostack = ((l_sole + r_sole)/com/postural)<<vel_lims;
        autostack->update();

        ik_solver.reset(
            new OpenSoT::solvers::iHQP(autostack->getStack(), autostack->getBounds(), 1e6));


    }

    virtual ~testPosturalTask() {

    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }

    void setWorld(const Eigen::Affine3d& l_sole_T_Waist, Eigen::VectorXd& q)
    {
        _model_ptr->setFloatingBasePose(l_sole_T_Waist);
        _model_ptr->update();
        _model_ptr->getJointPosition(q);
    }

    void setGoodInitialPosition() {
        q[_model_ptr->getDofIndex("RHipSag")+1 ] = -25.0*M_PI/180.0;
        q[_model_ptr->getDofIndex("RKneeSag")+1 ] = 50.0*M_PI/180.0;
        q[_model_ptr->getDofIndex("RAnkSag")+1 ] = -25.0*M_PI/180.0;

        q[_model_ptr->getDofIndex("LHipSag")+1 ] = -25.0*M_PI/180.0;
        q[_model_ptr->getDofIndex("LKneeSag")+1 ] = 50.0*M_PI/180.0;
        q[_model_ptr->getDofIndex("LAnkSag")+1 ] = -25.0*M_PI/180.0;

        q[_model_ptr->getDofIndex("LShSag")+1 ] =  20.0*M_PI/180.0;
        q[_model_ptr->getDofIndex("LShLat")+1 ] = 20.0*M_PI/180.0;
        q[_model_ptr->getDofIndex("LShYaw")+1 ] = -15.0*M_PI/180.0;
        q[_model_ptr->getDofIndex("LElbj")+1 ] = -80.0*M_PI/180.0;

        q[_model_ptr->getDofIndex("RShSag")+1 ] =  20.0*M_PI/180.0;
        q[_model_ptr->getDofIndex("RShLat")+1 ] = -20.0*M_PI/180.0;
        q[_model_ptr->getDofIndex("RShYaw")+1 ] = 15.0*M_PI/180.0;
        q[_model_ptr->getDofIndex("RElbj")+1 ] = -80.0*M_PI/180.0;

    }

    Cartesian::Ptr l_sole, r_sole;
    CoM::Ptr com;
    Postural::Ptr postural;
    VelocityLimits::Ptr vel_lims;
    OpenSoT::AutoStack::Ptr autostack;
    OpenSoT::solvers::iHQP::Ptr ik_solver;


    Eigen::VectorXd q, dq;

    double dT;

};

TEST_F(testPosturalTask, floating_base_open_loop)
{
    Eigen::VectorXd qm = this->q;
    Eigen::VectorXd dqm = this->dq;

    auto robot = GetTestModel("coman_floating_base");
    robot->setJointPosition(qm);
    robot->setJointVelocity(dqm/this->dT);
    robot->update();

    std::cout<<"q initial fb model: \n"<<q.segment(0,6).transpose()<<std::endl;

    std::cout<<"q initial fb robot: \n"<<qm.segment(0,6).transpose()<<std::endl;


    OpenSoT::tasks::floating_base::Contact::Ptr l_sole_fb(
        new OpenSoT::tasks::floating_base::Contact(*robot, "l_sole"));
    OpenSoT::tasks::floating_base::Contact::Ptr r_sole_fb(
        new OpenSoT::tasks::floating_base::Contact(*robot, "r_sole"));

    OpenSoT::AffineHelper var = OpenSoT::AffineHelper::Identity(6);
    Eigen::VectorXd ub = 10.*Eigen::VectorXd::Ones(6);
    Eigen::VectorXd lb = -ub;
    OpenSoT::constraints::GenericConstraint::Ptr fb_lims(
        new OpenSoT::constraints::GenericConstraint("fb_limits", var, ub, lb,
            OpenSoT::constraints::GenericConstraint::Type::BOUND));

    OpenSoT::AutoStack::Ptr autostack_fb(
                new OpenSoT::AutoStack((l_sole_fb + r_sole_fb)));
    autostack_fb<<fb_lims;
    autostack_fb->update();

    OpenSoT::solvers::iHQP::Ptr solver_fb(
        new OpenSoT::solvers::iHQP(autostack_fb->getStack(), autostack_fb->getBounds(), 1e6));

    Eigen::Vector3d com_ref = com->getActualPosition();
    com_ref[2] -= 0.1;
    com->setReference(com_ref);

    Eigen::VectorXd dQ(6); dQ.setZero(6);
    Eigen::VectorXd Q = qm.segment(0,7);
    std::cout<<"Q: "<<Q.transpose()<<std::endl;
    for(unsigned int i = 0; i < 1000; ++i)
    {
        qm = q;
        dqm = dq;
        robot->setJointPosition(qm);
        robot->setJointVelocity(dqm/dT);
        robot->update();

        autostack_fb->update();
        if(!solver_fb->solve(dQ)){
            std::cout<<"solver ik can not solve!"<<std::endl;
            dQ.setZero(dq.size());}


        Eigen::VectorXd Qfull(q.size()); Qfull.setZero();
        Eigen::VectorXd dQfull(dq.size()); dQfull.setZero();
        Qfull.segment(0,7) = Q;
        dQfull.segment(0,6) = dQ;

        Qfull = robot->sum(Qfull, dQfull*dT);

        Q = Qfull.segment(0,7);
        dQ = dQfull.segment(0,6);

        _model_ptr->setJointPosition(q);
        _model_ptr->setJointVelocity(dq/dT);
        _model_ptr->update();

        autostack->update();

        if(!ik_solver->solve(dq)){
            std::cout<<"solver ik can not solve!"<<std::endl;
            dq.setZero(dq.size());}
        q = _model_ptr->sum(q, dq);
    }

    for(unsigned int  i = 0 ; i < 3; ++i)
        EXPECT_NEAR(com->getActualPosition()[i], com_ref[i], 1e-4);

    std::cout<<"q final fb model: \n"<<q.segment(0,7).transpose()<<std::endl;
    std::cout<<"Q final fb robot: \n"<<Q.transpose()<<std::endl;

    std::cout<<"error: \n"<<(q.segment(0,7) - Q).array().abs()<<std::endl;

    for(unsigned int i = 0; i < 7; ++i)
        EXPECT_NEAR(Q[i], q.segment(0,7)[i], 1e-3);
}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

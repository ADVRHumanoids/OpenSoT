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

using namespace OpenSoT::tasks::velocity;
using namespace OpenSoT::constraints::velocity;

std::string relative_path = OPENSOT_TEST_PATH "configs/coman/configs/config_coman_floating_base.yaml";
std::string _path_to_cfg = relative_path;

namespace name {

class testPosturalTask: public ::testing::Test
{
protected:

    testPosturalTask()
    {
        dT = 0.002;

        model = XBot::ModelInterface::getModel(_path_to_cfg);

        q.setZero(model->getJointNum());
        setGoodInitialPosition();
        dq.setZero(model->getJointNum());

        model->setJointPosition(q);
        model->setJointVelocity(dq);
        model->update();

        KDL::Frame l_sole_T_Waist;
        model->getPose("Waist", "l_sole", l_sole_T_Waist);

        l_sole_T_Waist.p.x(0.0);
        l_sole_T_Waist.p.y(0.0);

        this->setWorld(l_sole_T_Waist, q);

        l_sole.reset(new Cartesian("l_sole", q, *model, "l_sole", "world"));
        r_sole.reset(new Cartesian("r_sole", q, *model, "r_sole", "world"));
        com.reset(new CoM(q, *model));
        postural.reset(new Postural(q));
        vel_lims.reset(new VelocityLimits(M_PI, dT, q.size()));

        autostack = ((l_sole + r_sole)/com/postural)<<vel_lims;
        autostack->update(q);

        ik_solver.reset(
            new OpenSoT::solvers::iHQP(autostack->getStack(), autostack->getBounds(), 1e6));


    }

    virtual ~testPosturalTask() {

    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }

    void setWorld(const KDL::Frame& l_sole_T_Waist, Eigen::VectorXd& q)
    {
        model->setFloatingBasePose(l_sole_T_Waist);
        model->update();
        model->getJointPosition(q);

    }

    void setGoodInitialPosition() {
        q[model->getDofIndex("RHipSag")] = -25.0*M_PI/180.0;
        q[model->getDofIndex("RKneeSag")] = 50.0*M_PI/180.0;
        q[model->getDofIndex("RAnkSag")] = -25.0*M_PI/180.0;

        q[model->getDofIndex("LHipSag")] = -25.0*M_PI/180.0;
        q[model->getDofIndex("LKneeSag")] = 50.0*M_PI/180.0;
        q[model->getDofIndex("LAnkSag")] = -25.0*M_PI/180.0;

        q[model->getDofIndex("LShSag")] =  20.0*M_PI/180.0;
        q[model->getDofIndex("LShLat")] = 20.0*M_PI/180.0;
        q[model->getDofIndex("LShYaw")] = -15.0*M_PI/180.0;
        q[model->getDofIndex("LElbj")] = -80.0*M_PI/180.0;

        q[model->getDofIndex("RShSag")] =  20.0*M_PI/180.0;
        q[model->getDofIndex("RShLat")] = -20.0*M_PI/180.0;
        q[model->getDofIndex("RShYaw")] = 15.0*M_PI/180.0;
        q[model->getDofIndex("RElbj")] = -80.0*M_PI/180.0;

    }

    Cartesian::Ptr l_sole, r_sole;
    CoM::Ptr com;
    Postural::Ptr postural;
    VelocityLimits::Ptr vel_lims;
    OpenSoT::AutoStack::Ptr autostack;
    OpenSoT::solvers::iHQP::Ptr ik_solver;

    XBot::ModelInterface::Ptr model;

    Eigen::VectorXd q, dq;

    double dT;

};

TEST_F(testPosturalTask, floating_base_open_loop)
{
    Eigen::VectorXd qm = this->q;
    Eigen::VectorXd dqm = this->dq;

    XBot::ModelInterface::Ptr robot = XBot::ModelInterface::getModel(_path_to_cfg);
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
    autostack_fb->update(qm);

    OpenSoT::solvers::iHQP::Ptr solver_fb(
        new OpenSoT::solvers::iHQP(autostack_fb->getStack(), autostack_fb->getBounds(), 0.));

    Eigen::Vector3d com_ref = com->getActualPosition();
    com_ref[2] -= 0.1;
    com->setReference(com_ref);

    Eigen::VectorXd dQ(6); dQ.setZero(6);
    Eigen::VectorXd Q = qm.segment(0,6);
    std::cout<<"Q: "<<Q.transpose()<<std::endl;
    for(unsigned int i = 0; i < 1000; ++i)
    {
        qm = q;
        dqm = dq;
        robot->setJointPosition(qm);
        robot->setJointVelocity(dqm/dT);
        robot->update();

        autostack_fb->update(qm);
        if(!solver_fb->solve(dQ)){
            std::cout<<"solver ik can not solve!"<<std::endl;
            dQ.setZero(dq.size());}
        Q += dQ*dT;

        model->setJointPosition(q);
        model->setJointVelocity(dq/dT);
        model->update();

        autostack->update(q);

        if(!ik_solver->solve(dq)){
            std::cout<<"solver ik can not solve!"<<std::endl;
            dq.setZero(dq.size());}
        q += dq;
    }

    for(unsigned int  i = 0 ; i < 3; ++i)
        EXPECT_NEAR(com->getActualPosition()[i], com_ref[i], 1e-6);

    std::cout<<"q final fb model: \n"<<q.segment(0,6).transpose()<<std::endl;
    std::cout<<"Q final fb robot: \n"<<Q.transpose()<<std::endl;

    std::cout<<"error: \n"<<(q.segment(0,6) - Q).array().abs()<<std::endl;

    for(unsigned int i = 0; i < 6; ++i)
        EXPECT_NEAR(Q[i], q.segment(0,6)[i], 1e-3);
}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

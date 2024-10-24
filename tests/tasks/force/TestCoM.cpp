#include <gtest/gtest.h>
#include <OpenSoT/solvers/iHQP.h>
#include <OpenSoT/tasks/force/CoM.h>
#include <cmath>
#include <fstream>
#include <OpenSoT/tasks/Aggregated.h>
#include <OpenSoT/utils/cartesian_utils.h>
#include <OpenSoT/constraints/force/FrictionCone.h>
#include <ros/ros.h>
#include <qpOASES.hpp>
#include <OpenSoT/constraints/force/WrenchLimits.h>
#include <xbot2_interface/xbotinterface2.h>
#include <OpenSoT/tasks/force/Force.h>
#include <OpenSoT/utils/InverseDynamics.h>
#include "../../common.h"


namespace{

class testForceCoM : public TestBase {

 protected:

  testForceCoM() : TestBase("coman_floating_base")
  {

  }

  virtual ~testForceCoM() {
  }

  virtual void SetUp() {
  }

  virtual void TearDown() {
  }


};

class testWrench : public TestBase {

 protected:

  testWrench() : TestBase("coman_floating_base")
  {

  }

  virtual ~testWrench() {
  }

  virtual void SetUp() {
  }

  virtual void TearDown() {
  }


};

Eigen::VectorXd getGoodInitialPosition(XBot::ModelInterface::Ptr _model_ptr) {
    Eigen::VectorXd _q = _model_ptr->getNeutralQ();

    _q[_model_ptr->getQIndex("RHipSag") ] = -25.0*M_PI/180.0;
    _q[_model_ptr->getQIndex("RKneeSag") ] = 50.0*M_PI/180.0;
    _q[_model_ptr->getQIndex("RAnkSag") ] = -25.0*M_PI/180.0;

    _q[_model_ptr->getQIndex("LHipSag") ] = -25.0*M_PI/180.0;
    _q[_model_ptr->getQIndex("LKneeSag") ] = 50.0*M_PI/180.0;
    _q[_model_ptr->getQIndex("LAnkSag") ] = -25.0*M_PI/180.0;

    _q[_model_ptr->getQIndex("LShSag") ] =  -90.0*M_PI/180.0;
    _q[_model_ptr->getQIndex("LForearmPlate") ] = -90.0*M_PI/180.0;

    _q[_model_ptr->getQIndex("RShSag") ] =  -90.0*M_PI/180.0;
    _q[_model_ptr->getQIndex("RForearmPlate") ] = -90.0*M_PI/180.0;

    return _q;
}


TEST_F(testForceCoM, testForceCoM_StaticCase) {
    std::cout<<"#nq: "<<_model_ptr->getNq()<<std::endl;
    std::cout<<"#nv: "<<_model_ptr->getNv()<<std::endl;
    std::cout<<"#joint_num: "<<_model_ptr->getJointNum()<<std::endl;
    std::cout<<"#joint_name size: "<<_model_ptr->getJointNames().size()<<std::endl;
    for(auto jn : _model_ptr->getJointNames())
        std::cout<<jn<<": "<<_model_ptr->getDofIndex(jn)<<std::endl;


    Eigen::VectorXd q = getGoodInitialPosition(_model_ptr);

    _model_ptr->setJointPosition(q);
    _model_ptr->update();

    std::vector<std::string> links_in_contact;
    links_in_contact.push_back("r_sole");
    links_in_contact.push_back("l_sole");
    links_in_contact.push_back("LSoftHand");

    std::vector<std::string>::iterator it;
    for(it = links_in_contact.begin();
        it != links_in_contact.end(); it++)
        std::cout<<"link in contact "<<":"<<*it<<std::endl;

    std::vector<OpenSoT::AffineHelper> contact_wrenches;
    for(unsigned int i = 0; i < links_in_contact.size(); ++i)
        contact_wrenches.push_back(OpenSoT::AffineHelper::Identity(6));

    OpenSoT::tasks::force::CoM::Ptr force_com_task(
        new OpenSoT::tasks::force::CoM(contact_wrenches, links_in_contact, *(_model_ptr.get())));
    force_com_task->update();

    Eigen::MatrixXd A = force_com_task->getA();
    EXPECT_EQ(A.rows(), 6);
    EXPECT_EQ(A.cols(), 6*links_in_contact.size());
    std::cout<<"A = [ "<<A<<" ]"<<std::endl;

    Eigen::VectorXd b = force_com_task->getb();
    EXPECT_EQ(b.rows(), 6);
    std::cout<<"b = [ "<<b<<" ]"<<std::endl;
    std::cout<<"force_com_task A: ["<<force_com_task->getA().rows()<<" x "<<force_com_task->getA().cols()<<" ]"<<std::endl;
    std::cout<<"force_com_task b: ["<<force_com_task->getb().size()<<" ]"<<std::endl;


    OpenSoT::constraints::force::FrictionCones::friction_cones friction_cones;
    OpenSoT::constraints::force::FrictionCone::friction_cone friction_cone;
    double mu = 2.;
    Eigen::Affine3d T;
    for(unsigned int i = 0; i < links_in_contact.size(); ++i)
    {
        _model_ptr->getPose(links_in_contact[i], T);
        friction_cone.first = T.linear();
        friction_cone.second = mu;
        friction_cones.push_back(friction_cone);
    }

    OpenSoT::OptvarHelper::VariableVector vv = {
                                                        {"r_sole", 6},
                                                        {"l_sole", 6},
                                                        {"LSoftHand", 6}};

    OpenSoT::OptvarHelper opt_v(vv);
    std::vector<OpenSoT::AffineHelper> wrenches;
    wrenches.push_back(opt_v.getVariable("r_sole"));
    wrenches.push_back(opt_v.getVariable("l_sole"));
    wrenches.push_back(opt_v.getVariable("LSoftHand"));


    OpenSoT::constraints::force::FrictionCones::Ptr fc(
                new OpenSoT::constraints::force::FrictionCones(
                    links_in_contact, wrenches, *(_model_ptr.get()), friction_cones));
    std::cout<<"Aineq = ["<<fc->getAineq()<<" ]"<<std::endl;
    std::cout<<"bUpper = ["<<fc->getbUpperBound()<<" "<<std::endl;
    std::cout<<"bLower = ["<<fc->getbLowerBound()<<" "<<std::endl;
    std::cout<<"fc Aineq: ["<<fc->getAineq().rows()<<" x "<<fc->getAineq().cols()<<" ]"<<std::endl;
    std::cout<<"fc getbUpperBound(): ["<<fc->getbUpperBound().size()<<"]"<<std::endl;
    std::cout<<"fc getbLowerBound(): ["<<fc->getbLowerBound().size()<<"]"<<std::endl;


    Eigen::VectorXd wrench_lims;
    wrench_lims.setOnes(contact_wrenches.size()*6); wrench_lims *= 300.;

    OpenSoT::OptvarHelper::VariableVector vars = {{"wrench", contact_wrenches.size()*6}};

    OpenSoT::OptvarHelper opt(vars);

    OpenSoT::AffineHelper wrench = opt.getVariable("wrench");

    OpenSoT::constraints::force::WrenchLimits::Ptr wrench_limits(
                new OpenSoT::constraints::force::WrenchLimits("all_contacts",
                                                              -wrench_lims,
                                                              wrench_lims,wrench));

    std::cout<<"wrench_limits Aineq: ["<<wrench_limits->getAineq().rows()<<" x "<<wrench_limits->getAineq().cols()<<" ]"<<std::endl;
    std::cout<<"wrench_limits getbUpperBound(): ["<<wrench_limits->getbUpperBound().size()<<"]"<<std::endl;
    std::cout<<"wrench_limits getbLowerBound(): ["<<wrench_limits->getbLowerBound().size()<<"]"<<std::endl;


    force_com_task->getConstraints().push_back(fc);
    force_com_task->getConstraints().push_back(wrench_limits);
    force_com_task->update();

    Eigen::VectorXd wrench_reference(contact_wrenches.size()*6);
    wrench_reference.setZero();
    wrench_reference(2) = 100.;
    wrench_reference(8) = 100.;
    wrench_reference(14) = 100.;


    OpenSoT::solvers::iHQP::Stack stack_of_tasks;
    stack_of_tasks.push_back(force_com_task);


    OpenSoT::solvers::iHQP::Ptr sot(
                new OpenSoT::solvers::iHQP(stack_of_tasks,1.));
    std::cout<<"Solver started"<<std::endl;

    _model_ptr->setJointPosition(q);
    _model_ptr->update();
    force_com_task->update();

    Eigen::VectorXd contact_wrenches_d;
    EXPECT_TRUE(sot->solve(contact_wrenches_d));
    std::cout<<"contact_wrenches_d = [ "<<contact_wrenches_d<<" ]"<<std::endl;


    double m = _model_ptr->getMass();

    EXPECT_NEAR(contact_wrenches_d[2] + contact_wrenches_d[8]
            + contact_wrenches_d[14], m*9.81, 1E-6);

    Eigen::VectorXd Ax = A*contact_wrenches_d;
    std::cout<<"A*x = \n"<<Ax<<std::endl;
    std::cout<<"b = \n"<<b<<std::endl;

    for(unsigned int i = 0; i < b.rows(); ++i)
        EXPECT_NEAR(Ax[i],b[i], 1E-6);


    std::cout<<"Wrench desired for contact "<<links_in_contact[0]<<": ["<<
               contact_wrenches_d.segment(0, 6)<<" ]"<<std::endl;

    std::cout<<"Wrench desired for contact "<<links_in_contact[1]<<": ["<<
               contact_wrenches_d.segment(6, 6)<<" ]"<<std::endl;

    std::cout<<"Wrench desired for contact "<<links_in_contact[2]<<": ["<<
               contact_wrenches_d.segment(12, 6)<<" ]"<<std::endl;
}

TEST_F(testWrench, testWrench_) {
    Eigen::Vector6d wrench_desired;
    wrench_desired.setRandom();

    OpenSoT::tasks::force::Wrench::Ptr _wrench_task;
    OpenSoT::OptvarHelper::VariableVector vars = {{"wrench", 6}};

    OpenSoT::OptvarHelper opt(vars);

    OpenSoT::AffineHelper wrench = opt.getVariable("wrench");

    std::cout<<"wrench->getM"<<wrench.getM()<<std::endl;
    std::cout<<"wrench->getq"<<wrench.getq()<<std::endl;

    _wrench_task = std::make_shared<OpenSoT::tasks::force::Wrench>("l_sole_wrench", "l_sole","world", wrench);
    _wrench_task->update();

    Eigen::VectorXd tmp;
    _wrench_task->getReference(tmp);
    EXPECT_TRUE(tmp.size() == 6)<<"tmp.size(): "<<tmp.size()<<std::endl;
    for(unsigned int i = 0; i < 6; ++i)
        EXPECT_DOUBLE_EQ(tmp[i], 0.0);

    _wrench_task->setReference(wrench_desired);
    _wrench_task->update();
    _wrench_task->getReference(tmp);
    EXPECT_TRUE(tmp.size() == 6)<<"tmp.size(): "<<tmp.size()<<std::endl;
    for(unsigned int i = 0; i < 6; ++i){
        EXPECT_DOUBLE_EQ(tmp[i], wrench_desired[i]);
        EXPECT_DOUBLE_EQ(_wrench_task->getb()[i], wrench_desired[i]);}

    OpenSoT::solvers::iHQP::Stack stack_of_tasks;
    stack_of_tasks.push_back(_wrench_task);


    OpenSoT::solvers::iHQP::Ptr sot(new OpenSoT::solvers::iHQP(stack_of_tasks,0.));
    Eigen::VectorXd x;
    EXPECT_TRUE(sot->solve(x));
    std::cout<<"wrench desired: ["<<wrench_desired.transpose()<<"]"<<std::endl;
    std::cout<<"x: ["<<x.transpose()<<"]"<<std::endl;

    for(unsigned int i = 0; i < 6; ++i)
        EXPECT_DOUBLE_EQ(x[i], wrench_desired[i]);

    wrench_desired.setRandom();
    _wrench_task->setReference(wrench_desired);
    _wrench_task->update();

    EXPECT_TRUE(sot->solve(x));
    std::cout<<"wrench desired: ["<<wrench_desired.transpose()<<"]"<<std::endl;
    std::cout<<"x: ["<<x.transpose()<<"]"<<std::endl;

    for(unsigned int i = 0; i < 6; ++i)
        EXPECT_DOUBLE_EQ(x[i], wrench_desired[i]);

    Eigen::VectorXd upperLims(6);
    Eigen::VectorXd lowerLims(6);
    upperLims<<Eigen::Vector3d::Ones(), Eigen::Vector3d::Zero();
    lowerLims = -upperLims;
    std::cout<<"upperLims: "<<upperLims.transpose()<<std::endl;
    std::cout<<"lowerLims: "<<lowerLims.transpose()<<std::endl;
    OpenSoT::constraints::force::WrenchLimits::Ptr wrench_lims =
            std::make_shared<OpenSoT::constraints::force::WrenchLimits>
            ("l_sole", lowerLims, upperLims,wrench);

    OpenSoT::AutoStack::Ptr autostack = std::make_shared<OpenSoT::AutoStack>(_wrench_task);
    autostack<<wrench_lims;


    wrench_desired.setRandom();
    _wrench_task->setReference(wrench_desired);

    autostack->update();

    OpenSoT::solvers::iHQP::Ptr sot2(new OpenSoT::solvers::iHQP(autostack->getStack(),
                                                                autostack->getBounds(),0));
    EXPECT_TRUE(sot2->solve(x));

    std::cout<<"wrench desired: ["<<wrench_desired.transpose()<<"]"<<std::endl;
    std::cout<<"x: ["<<x.transpose()<<"]"<<std::endl;

    for(unsigned int i = 0; i < 3; ++i)
        EXPECT_DOUBLE_EQ(x[i], wrench_desired[i]);
    for(unsigned int i = 3; i < 6; ++i)
        EXPECT_DOUBLE_EQ(x[i], 0.0);

    wrench_lims->releaseContact(true);

    autostack->update();

    EXPECT_TRUE(sot2->solve(x));

    std::cout<<"wrench desired: ["<<wrench_desired.transpose()<<"]"<<std::endl;
    std::cout<<"x: ["<<x.transpose()<<"]"<<std::endl;

    for(unsigned int i = 0; i < 6; ++i)
        EXPECT_DOUBLE_EQ(x[i], 0.0);

    wrench_lims->releaseContact(false);

    autostack->update();

    EXPECT_TRUE(sot2->solve(x));

    std::cout<<"wrench desired: ["<<wrench_desired.transpose()<<"]"<<std::endl;
    std::cout<<"x: ["<<x.transpose()<<"]"<<std::endl;

    for(unsigned int i = 0; i < 3; ++i)
        EXPECT_DOUBLE_EQ(x[i], wrench_desired[i]);
    for(unsigned int i = 3; i < 6; ++i)
        EXPECT_DOUBLE_EQ(x[i], 0.0);
}

TEST_F(testWrench, testWrenchLim)
{
    OpenSoT::OptvarHelper::VariableVector vars = {{"wrench1", 3},
                                                  {"wrench2", 3},
                                                  {"qddot", 6}};

    OpenSoT::OptvarHelper opt(vars);

    OpenSoT::AffineHelper wrench1 = opt.getVariable("wrench1");



    Eigen::Vector3d upperLims;
    Eigen::Vector3d lowerLims;
    upperLims.setOnes(); upperLims*=100.;
    lowerLims = -upperLims;
    std::cout<<"upperLims: "<<upperLims.transpose()<<std::endl;
    std::cout<<"lowerLims: "<<lowerLims.transpose()<<std::endl;
    OpenSoT::constraints::force::WrenchLimits::Ptr wrench_lims =
            std::make_shared<OpenSoT::constraints::force::WrenchLimits>
            ("wrench1", lowerLims, upperLims,wrench1);
    wrench_lims->update();

    std::vector<OpenSoT::AffineHelper> wrenches;
    wrenches.push_back(opt.getVariable("wrench1"));
    wrenches.push_back(opt.getVariable("wrench2"));

    std::vector<std::string> contacts;
    contacts.push_back("wrench1");
    contacts.push_back("wrench2");
    OpenSoT::constraints::force::WrenchesLimits::Ptr wrenches_lims =
            std::make_shared<OpenSoT::constraints::force::WrenchesLimits>
            (contacts, lowerLims, upperLims,wrenches);
    wrenches_lims->update();


}

TEST_F(testWrench, testWrenches) {

    OpenSoT::OptvarHelper::VariableVector vars = {{"wrench1", 6},
                                                  {"wrench2", 6},
                                                  {"qddot", 12}};
    OpenSoT::OptvarHelper opt(vars);
    OpenSoT::AffineHelper wrench1 = opt.getVariable("wrench1");
    OpenSoT::AffineHelper wrench2 = opt.getVariable("wrench2");
    OpenSoT::AffineHelper qddot = opt.getVariable("qddot");

    std::vector<OpenSoT::AffineHelper> wrenches;
    wrenches.push_back(wrench1);
    wrenches.push_back(wrench2);

    std::cout<<"wrench1.getInputSize(): "<<wrench1.getInputSize()<<std::endl;
    std::cout<<"wrench1.getOutputSize(): "<<wrench1.getOutputSize()<<std::endl;

    std::vector<std::string> contacts;
    contacts.push_back("wrench1");
    contacts.push_back("wrench2");

    std::vector<std::string> base_links;
    base_links.push_back("world");
    base_links.push_back("world");

    OpenSoT::tasks::force::Wrenches::Ptr wrenches_task = std::make_shared<OpenSoT::tasks::force::Wrenches>
            ("wrenches", contacts, base_links, wrenches);
    wrenches_task->update();

    std::cout<<"wrenches_task->getA(): \n"<<wrenches_task->getA()<<std::endl;
    std::cout<<"wrenches_task->getb(): \n"<<wrenches_task->getb().transpose()<<std::endl;

    EXPECT_FALSE(wrenches_task->getWrenchTask("sossio"));

    Eigen::Vector6d wrench_desired;
    wrench_desired.setRandom();

    wrenches_task->getWrenchTask("wrench2")->setReference(wrench_desired);
    wrenches_task->update();


    OpenSoT::solvers::iHQP::Stack stack_of_tasks;
    stack_of_tasks.push_back(wrenches_task);


    OpenSoT::solvers::iHQP::Ptr sot(new OpenSoT::solvers::iHQP(stack_of_tasks,0.));
    Eigen::VectorXd x;
    EXPECT_TRUE(sot->solve(x));
    std::cout<<"wrench desired: ["<<wrench_desired.transpose()<<"]"<<std::endl;
    std::cout<<"x: ["<<x.transpose()<<"]"<<std::endl;

    for(unsigned int i = 0; i < 6; ++i)
        EXPECT_DOUBLE_EQ(x[i+6], wrench_desired[i]);


    Eigen::VectorXd upperLims(6);
    Eigen::VectorXd lowerLims(6);
    upperLims<<Eigen::Vector3d::Ones(), Eigen::Vector3d::Zero();
    lowerLims = -upperLims;
    std::cout<<"upperLims: "<<upperLims.transpose()<<std::endl;
    std::cout<<"lowerLims: "<<lowerLims.transpose()<<std::endl;

    OpenSoT::constraints::force::WrenchesLimits::Ptr wrenches_lims =
            std::make_shared<OpenSoT::constraints::force::WrenchesLimits>
            (contacts, lowerLims, upperLims, wrenches);
    std::cout<<"wrenches_lims->getbLowerBound()"<<wrenches_lims->getbLowerBound().transpose()<<std::endl;
    std::cout<<"wrenches_lims->getbUpperBound()"<<wrenches_lims->getbUpperBound().transpose()<<std::endl;
    std::cout<<"wrenches_lims->getAineq() \n"<<wrenches_lims->getAineq()<<std::endl;

    OpenSoT::AutoStack::Ptr autostack;
    autostack /= wrenches_task;
    autostack<<wrenches_lims;

    Eigen::VectorXd wrench_desired_1(6); wrench_desired_1.setOnes(6); wrench_desired_1 *= 2;
    Eigen::VectorXd wrench_desired_2(6); wrench_desired_2.setOnes(6); wrench_desired_2 *= -2;

    wrenches_task->getWrenchTask("wrench1")->setReference(wrench_desired_1);
    wrenches_task->getWrenchTask("wrench2")->setReference(wrench_desired_2);

    autostack->update();


    OpenSoT::solvers::iHQP::Ptr sot2(new OpenSoT::solvers::iHQP(autostack->getStack(),
                                                                autostack->getBounds(),1.));
    EXPECT_TRUE(sot2->solve(x));
    std::cout<<"wrench desired: ["<<wrench_desired_1.transpose()<<" "<<wrench_desired_2.transpose()<<"]"<<std::endl;
    std::cout<<"x: ["<<x.transpose()<<"]"<<std::endl;

    for(unsigned int i = 0; i < 6; ++i){
        EXPECT_NEAR(x[i], upperLims[i], 1e-9);
        EXPECT_NEAR(x[i+6], lowerLims[i], 1e-9);}

    wrenches_lims->getWrenchLimits("wrench2")->releaseContact(true);
    autostack->update();

    std::cout<<"wrenches_lims->getbLowerBound()"<<wrenches_lims->getbLowerBound().transpose()<<std::endl;
    std::cout<<"wrenches_lims->getbUpperBound()"<<wrenches_lims->getbUpperBound().transpose()<<std::endl;
    std::cout<<"wrenches_lims->getAineq() \n"<<wrenches_lims->getAineq()<<std::endl;



    EXPECT_TRUE(sot2->solve(x));
    std::cout<<"wrench desired: ["<<wrench_desired_1.transpose()<<" "<<wrench_desired_2.transpose()<<"]"<<std::endl;
    std::cout<<"x: ["<<x.transpose()<<"]"<<std::endl;

    for(unsigned int i = 0; i < 6; ++i){
        EXPECT_NEAR(x[i], upperLims[i], 1e-9);
        EXPECT_NEAR(x[i+6], 0.0, 1e-9);}

    wrenches_lims->getWrenchLimits("wrench2")->releaseContact(false);
    autostack->update();

    EXPECT_TRUE(sot2->solve(x));
    std::cout<<"wrench desired: ["<<wrench_desired_1.transpose()<<" "<<wrench_desired_2.transpose()<<"]"<<std::endl;
    std::cout<<"x: ["<<x.transpose()<<"]"<<std::endl;

    for(unsigned int i = 0; i < 6; ++i){
        EXPECT_NEAR(x[i], upperLims[i], 1e-9);
        EXPECT_NEAR(x[i+6], lowerLims[i], 1e-9);}
}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

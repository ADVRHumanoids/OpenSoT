#include <gtest/gtest.h>
#include <OpenSoT/solvers/iHQP.h>
#include <OpenSoT/solvers/eHQP.h>
#include <OpenSoT/tasks/force/CoM.h>
#include <OpenSoT/constraints/force/FrictionCone.h>
#include <OpenSoT/constraints/force/WrenchLimits.h>
#include <cmath>
#include <fstream>
#include <xbot2_interface/xbotinterface2.h>
#include <xbot2_interface/xbotinterface2.h>
#include <ros/master.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_kdl.h>

#include "../../common.h"


namespace{

class testFrictionCones : public TestBase {
 protected:

  testFrictionCones() : TestBase("coman_floating_base")
  {

      int number_of_dofs = _model_ptr->getNq();
      std::cout<<"#DoFs: "<<number_of_dofs<<std::endl;

      _q.resize(number_of_dofs);
      _q = _model_ptr->getNeutralQ();

      _model_ptr->setJointPosition(_q);
      _model_ptr->update();

      //Update world according this new configuration:
      Eigen::Affine3d l_sole_T_Waist;
      this->_model_ptr->getPose("Waist", "l_sole", l_sole_T_Waist);
      std::cout<<"l_sole_T_Waist:\n"<<l_sole_T_Waist.matrix()<<std::endl;

      l_sole_T_Waist.translation()[0] = 0.0;
      l_sole_T_Waist.translation()[1] = 0.0;

      this->setWorld(l_sole_T_Waist, this->_q);
      this->_model_ptr->setJointPosition(this->_q);
      this->_model_ptr->update();


      Eigen::Affine3d world_T_bl;
      _model_ptr->getPose("Waist",world_T_bl);

      std::cout<<"world_T_bl: \n "<<world_T_bl.matrix()<<std::endl;

      _q[_model_ptr->getDofIndex("RAnkLat") + 1] = -45.0*M_PI/180.0;
      _q[_model_ptr->getDofIndex("LAnkLat") + 1] = 45.0*M_PI/180.0;
      _model_ptr->setJointPosition(_q);
      _model_ptr->update();
      //Update world according this new configuration:
      world_T_bl.translation()[2] += 0.05;
      this->setWorld(l_sole_T_Waist, this->_q);
      this->_model_ptr->setJointPosition(this->_q);
      this->_model_ptr->update();
      //
  }

  void setWorld(const Eigen::Affine3d& l_sole_T_Waist, Eigen::VectorXd& q)
  {
      this->_model_ptr->setFloatingBasePose(l_sole_T_Waist);

      this->_model_ptr->update();

      this->_model_ptr->getJointPosition(q);
  }


  virtual ~testFrictionCones() {
  }

  virtual void SetUp() {
  }

  virtual void TearDown() {
  }




public:

  Eigen::VectorXd _q;
  OpenSoT::tasks::force::CoM::Ptr com;

  OpenSoT::constraints::force::WrenchLimits::Ptr wrench_limits;
  OpenSoT::constraints::force::FrictionCones::Ptr friction_cones;
  OpenSoT::solvers::iHQP::Ptr QPsolver;
  OpenSoT::solvers::eHQP::Ptr SVDsolver;

  Eigen::MatrixXd A;
  Eigen::VectorXd b;
  Eigen::VectorXd Ax;

  Eigen::MatrixXd Aineq;
  Eigen::VectorXd bUpperBoud;
  Eigen::VectorXd Aineqx;

};


TEST_F(testFrictionCones, testFrictionCones_) {

    std::vector<std::string> links_in_contact;
    links_in_contact.push_back("r_sole");
    links_in_contact.push_back("l_sole");

    Eigen::Affine3d w_T_r_sole;
    _model_ptr->getPose("r_sole", w_T_r_sole);
    Eigen::Affine3d w_T_l_sole;
    _model_ptr->getPose("l_sole", w_T_l_sole);


    Eigen::VectorXd QPcontact_wrenches_d(6*links_in_contact.size());
    QPcontact_wrenches_d.setZero(QPcontact_wrenches_d.rows());
    Eigen::VectorXd SVDcontact_wrenches_d(6*links_in_contact.size());
    SVDcontact_wrenches_d.setZero(SVDcontact_wrenches_d.rows());

    com.reset(new OpenSoT::tasks::force::CoM(QPcontact_wrenches_d, links_in_contact, *_model_ptr));
    com->update(QPcontact_wrenches_d);


    Eigen::VectorXd wrench_lims;
    wrench_lims.setOnes(6*links_in_contact.size()); wrench_lims *= 300.;

    OpenSoT::OptvarHelper::VariableVector vars = {{"wrench", 6*links_in_contact.size()}};

    OpenSoT::OptvarHelper opt(vars);

    OpenSoT::AffineHelper wrench = opt.getVariable("wrench");

    OpenSoT::constraints::force::WrenchLimits::Ptr wrench_limits(
                new OpenSoT::constraints::force::WrenchLimits("all_contacts",
                                                              -wrench_lims,
                                                              wrench_lims,wrench));

    OpenSoT::solvers::iHQP::Stack stack_of_tasks;
    stack_of_tasks.push_back(com);

    QPsolver.reset(new OpenSoT::solvers::iHQP(stack_of_tasks,wrench_limits,2E5));

    std::cout<<"QP Solver started"<<std::endl;
    bool solved = false;
    do{
        solved = QPsolver->solve(QPcontact_wrenches_d);
    }while(!solved);
    std::cout<<"contact_wrenches_d w/o constraint :"<<std::endl;
    std::cout<<"    r_sole = ["<<QPcontact_wrenches_d.segment(0,6)<<"]"<<std::endl;
    std::cout<<"    l_sole = ["<<QPcontact_wrenches_d.segment(6,6)<<"]"<<std::endl;

    SVDsolver.reset(new OpenSoT::solvers::eHQP(stack_of_tasks));
    std::cout<<"SVD Solver started"<<std::endl;
    SVDsolver->solve(SVDcontact_wrenches_d);
    std::cout<<"contact_wrenches_d w/o constraint :"<<std::endl;
    std::cout<<"    r_sole = ["<<SVDcontact_wrenches_d.segment(0,6)<<"]"<<std::endl;
    std::cout<<"    l_sole = ["<<SVDcontact_wrenches_d.segment(6,6)<<"]"<<std::endl;

    for(unsigned int i = 0; i < 6*links_in_contact.size(); ++i)
        EXPECT_NEAR(QPcontact_wrenches_d[i],
                    SVDcontact_wrenches_d[i], 1e-3);


    std::vector<std::pair<Eigen::Matrix3d, double> > friction__cones;
    double mu = 0.5;

    Eigen::Affine3d T;
    _model_ptr->getPose(links_in_contact[0],T);
    friction__cones.push_back(std::pair<Eigen::Matrix3d, double>(T.linear(), mu));
    _model_ptr->getPose(links_in_contact[1],T);
    friction__cones.push_back(std::pair<Eigen::Matrix3d, double>(T.linear(), mu));


    OpenSoT::OptvarHelper::VariableVector vv = {
                                                        {"r_sole", 6},
                                                        {"l_sole", 6}};

    OpenSoT::OptvarHelper opt_v(vv);
    std::vector<OpenSoT::AffineHelper> wrenches;
    wrenches.push_back(opt_v.getVariable("r_sole"));
    wrenches.push_back(opt_v.getVariable("l_sole"));



    friction_cones.reset(new OpenSoT::constraints::force::FrictionCones(
                            links_in_contact, wrenches, *(_model_ptr.get()), friction__cones));
    friction_cones->update(QPcontact_wrenches_d);




    EXPECT_EQ(friction_cones->getbUpperBound().rows(), 5*friction__cones.size());
    EXPECT_EQ(friction_cones->getAineq().rows(), 5*friction__cones.size());
    EXPECT_EQ(friction_cones->getAineq().cols(), 6*friction__cones.size());

    std::cout<<"friction_cones lb: "<<friction_cones->getbLowerBound()<<std::endl;
    std::cout<<"friction_cones ub: "<<friction_cones->getbUpperBound()<<std::endl;
    std::cout<<"friction_cones Aineq: "<<friction_cones->getAineq()<<std::endl;



    QPcontact_wrenches_d.setZero(QPcontact_wrenches_d.size());
    QPsolver.reset(new OpenSoT::solvers::iHQP(stack_of_tasks,wrench_limits,friction_cones));
    std::cout<<"QP Solver started"<<std::endl;
    solved = false;
    do{
        solved = QPsolver->solve(QPcontact_wrenches_d);
    }while(!solved);
    std::cout<<"contact_wrenches_d w constraint :"<<std::endl;
    std::cout<<"    r_sole = ["<<QPcontact_wrenches_d.segment(0,6)<<"]"<<std::endl;
    std::cout<<"    l_sole = ["<<QPcontact_wrenches_d.segment(6,6)<<"]"<<std::endl;

    std::cout<<std::endl;

    Eigen::VectorXd wrench_in_l_sole(6); wrench_in_l_sole.setZero(6);
    Eigen::VectorXd wrench_in_r_sole(6); wrench_in_r_sole.setZero(6);
    Eigen::Affine3d w_T_lsole;
    _model_ptr->getPose("l_sole", w_T_lsole);
    wrench_in_l_sole = w_T_lsole.rotation().transpose()*QPcontact_wrenches_d.segment(6,3);
    std::cout<<"forces in l_sole = ["<<wrench_in_l_sole<<std::endl;
    Eigen::Affine3d w_T_rsole;
    _model_ptr->getPose("r_sole", w_T_rsole);
    wrench_in_r_sole = w_T_rsole.rotation().transpose()*QPcontact_wrenches_d.segment(0,3);
    std::cout<<"forces in r_sole = ["<<wrench_in_r_sole<<std::endl;

    double c = std::sqrt(2.*mu)/2.;
    std::cout<<"left friction cones constr: "<<std::endl;
    std::cout<<-c*wrench_in_l_sole[2]<<" <= "<<wrench_in_l_sole[0]<<" <= "<<c*wrench_in_l_sole[2]<<std::endl;
    std::cout<<-c*wrench_in_l_sole[2]<<" <= "<<wrench_in_l_sole[1]<<" <= "<<c*wrench_in_l_sole[2]<<std::endl;
    std::cout<<-wrench_in_l_sole[2]<<" <= "<<0.0<<std::endl;
    for(unsigned int i = 0; i < 2; ++i){
        EXPECT_LE(-c*wrench_in_l_sole[2]-wrench_in_l_sole[i], 1e-6);
        EXPECT_LE(-c*wrench_in_l_sole[2]+wrench_in_l_sole[i], 1e-6);
    }
    EXPECT_LE(-wrench_in_l_sole[2],0.0);

    std::cout<<"right friction cones constr: "<<std::endl;
    std::cout<<-c*wrench_in_r_sole[2]<<" <= "<<wrench_in_r_sole[0]<<" <= "<<c*wrench_in_r_sole[2]<<std::endl;
    std::cout<<-c*wrench_in_r_sole[2]<<" <= "<<wrench_in_r_sole[1]<<" <= "<<c*wrench_in_r_sole[2]<<std::endl;
    std::cout<<-wrench_in_r_sole[2]<<" <= "<<0.0<<std::endl;
    for(unsigned int i = 0; i < 2; ++i){
        EXPECT_LE(-c*wrench_in_r_sole[2]-wrench_in_r_sole[i], 1e-6);
        EXPECT_LE(-c*wrench_in_r_sole[2]+wrench_in_r_sole[i], 1e-6);
    }
    EXPECT_LE(-wrench_in_r_sole[2],0.0);


    std::cout<<"USING SVD:"<<std::endl;
    wrench_in_l_sole = w_T_lsole.rotation().transpose()*SVDcontact_wrenches_d.segment(6,3);
    std::cout<<"forces in l_sole = ["<<wrench_in_l_sole<<std::endl;
    wrench_in_r_sole = w_T_rsole.rotation().transpose()*SVDcontact_wrenches_d.segment(0,3);
    std::cout<<"forces in r_sole = ["<<wrench_in_r_sole<<std::endl;

    std::cout<<"robot mass is: "<<_model_ptr->getMass()<<" kg"<<std::endl;


}
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

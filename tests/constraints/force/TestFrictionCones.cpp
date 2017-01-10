#include <gtest/gtest.h>
#include <OpenSoT/solvers/QPOases.h>
#include <OpenSoT/tasks/force/CoM.h>
#include <OpenSoT/constraints/force/FrictionCone.h>
#include <cmath>
#include <idynutils/tests_utils.h>
#include <idynutils/RobotUtils.h>
#include <fstream>
#include <idynutils/cartesian_utils.h>

namespace{

class testFrictionCones : public ::testing::Test {
 protected:

  testFrictionCones():
      robot("coman",
            std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.urdf",
            std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.srdf")
  {

      yarp::sig::Vector q = getGoodInitialPosition(robot);

      robot.updateiDyn3Model(q, true);
  }

  virtual ~testFrictionCones() {
  }

  virtual void SetUp() {
  }

  virtual void TearDown() {
  }

  yarp::sig::Vector getGoodInitialPosition(iDynUtils& idynutils) {
      yarp::sig::Vector q(idynutils.iDyn3_model.getNrOfDOFs(), 0.0);
      yarp::sig::Vector leg(idynutils.left_leg.getNrOfDOFs(), 0.0);
      leg[0] = -25.0 * M_PI/180.0;
      leg[3] =  50.0 * M_PI/180.0;
      leg[5] = -25.0 * M_PI/180.0;
      idynutils.fromRobotToIDyn(leg, q, idynutils.left_leg);
      idynutils.fromRobotToIDyn(leg, q, idynutils.right_leg);
      yarp::sig::Vector arm(idynutils.left_arm.getNrOfDOFs(), 0.0);
      arm[0] = -90.0 * M_PI/180.0;
      arm[1] = 0.0 * M_PI/180.0;
      arm[3] = 0.0 * M_PI/180.0;
      idynutils.fromRobotToIDyn(arm, q, idynutils.left_arm);

      return q;
  }


public:
  iDynUtils robot;
  OpenSoT::tasks::force::CoM::Ptr com;
  OpenSoT::constraints::force::FrictionCone::Ptr friction_cones;
  OpenSoT::solvers::QPOases_sot::Ptr solver;

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
    links_in_contact.push_back("LSoftHand");

    // l_sole and r_sole are on the plane
    Eigen::Matrix3d w_R_r_sole; w_R_r_sole.setIdentity();
    Eigen::Matrix3d w_R_l_sole; w_R_l_sole.setIdentity();
    // LSoftHand is on a plane that is rotated of -45. deg wrt the world
    KDL::Frame w_T_LSoftHand; w_T_LSoftHand.Identity();
    w_T_LSoftHand.M.DoRotY(-45.*M_PI/180.);
    Eigen::Matrix3d w_R_LSoftHand;
    for(unsigned int i = 0; i < 3; ++i){
        for(unsigned int j = 0; j < 3; ++j)
            w_R_LSoftHand(i,j) = w_T_LSoftHand.M(i,j);
    }

    std::vector<std::pair<Eigen::Matrix3d, double> > friction__cones;
    friction__cones.push_back(std::pair<Eigen::Matrix3d, double>(w_R_r_sole, 1.));
    friction__cones.push_back(std::pair<Eigen::Matrix3d, double>(w_R_l_sole, 1.));
    friction__cones.push_back(std::pair<Eigen::Matrix3d, double>(w_R_LSoftHand, 1.0));

    Eigen::VectorXd contact_wrenches_d(6*links_in_contact.size());
    contact_wrenches_d.setZero(contact_wrenches_d.rows());
    com.reset(new OpenSoT::tasks::force::CoM(contact_wrenches_d, links_in_contact, robot));
    com->update(contact_wrenches_d);

    friction_cones.reset(new OpenSoT::constraints::force::FrictionCone(contact_wrenches_d,
                            robot, friction__cones));

    EXPECT_EQ(friction_cones->getNumberOfContacts(), 3);

    EXPECT_EQ(friction_cones->getbUpperBound().rows(), 5*friction__cones.size());
    EXPECT_EQ(friction_cones->getAineq().rows(), 5*friction__cones.size());
    EXPECT_EQ(friction_cones->getAineq().cols(), 6*friction__cones.size());

    OpenSoT::solvers::QPOases_sot::Stack stack_of_tasks;
    stack_of_tasks.push_back(com);

    solver.reset(new OpenSoT::solvers::QPOases_sot(stack_of_tasks,2E3));
    std::cout<<"Solver started"<<std::endl;
    bool solved = false;
    do{
        solved = solver->solve(contact_wrenches_d);
    }while(!solved);
    std::cout<<"contact_wrenches_d w/o constraint :"<<std::endl;
    std::cout<<"    r_sole = ["<<contact_wrenches_d.segment(0,6)<<"]"<<std::endl;
    std::cout<<"    l_sole = ["<<contact_wrenches_d.segment(6,6)<<"]"<<std::endl;
    std::cout<<"    LSoftHand = ["<<contact_wrenches_d.segment(12,6)<<"]"<<std::endl;


    A = com->getA();
    b = com->getb();
    Ax = A*contact_wrenches_d;
    std::cout<<"A = \n"<<A<<std::endl;
    std::cout<<"A*x = \n"<<Ax<<std::endl;
    std::cout<<"b = \n"<<b<<std::endl;

    for(unsigned int i = 0; i < b.rows(); ++i)
        EXPECT_NEAR(Ax[i],b[i], 1E-6);

    std::cout<<std::endl;



    com->getConstraints().push_back(friction_cones);
    solved = false;
    solver.reset(new OpenSoT::solvers::QPOases_sot(stack_of_tasks,2E3));
    std::cout<<"Solver started"<<std::endl;
    do{
        solved = solver->solve(contact_wrenches_d);
    }while(!solved);
    std::cout<<"contact_wrenches_d w constraint 1:"<<std::endl;
    std::cout<<"    r_sole = ["<<contact_wrenches_d.segment(0,6)<<"]"<<std::endl;
    std::cout<<"    l_sole = ["<<contact_wrenches_d.segment(6,6)<<"]"<<std::endl;
    std::cout<<"    LSoftHand = ["<<contact_wrenches_d.segment(12,6)<<"]"<<std::endl;

    Ax = A*contact_wrenches_d;
    std::cout<<"A = \n"<<A<<std::endl;
    std::cout<<"A*x = \n"<<Ax<<std::endl;
    std::cout<<"b = \n"<<b<<std::endl;

    for(unsigned int i = 0; i < b.rows(); ++i)
        EXPECT_NEAR(Ax[i],b[i], 1E-6);

    Aineq = friction_cones->getAineq();
    bUpperBoud = friction_cones->getbUpperBound();
    Aineqx = Aineq*contact_wrenches_d;
    std::cout<<"Aineq = \n"<<Aineq<<std::endl;
    std::cout<<"Aineq*x = \n"<<Aineqx<<std::endl;
    std::cout<<"bUpperBound = \n"<<bUpperBoud<<std::endl;

    for(unsigned int i = 0; i < bUpperBoud.rows(); ++i)
        EXPECT_LE(Aineqx[i],bUpperBoud[i]);


    friction__cones.clear();
    friction__cones.push_back(std::pair<Eigen::Matrix3d, double>(w_R_r_sole, 1.));
    friction__cones.push_back(std::pair<Eigen::Matrix3d, double>(w_R_l_sole, 1.));
    friction__cones.push_back(std::pair<Eigen::Matrix3d, double>(w_R_LSoftHand, 0.5));
    friction_cones->setMu(friction__cones);
    friction_cones->update(contact_wrenches_d);
    com->update(contact_wrenches_d);

    solver.reset(new OpenSoT::solvers::QPOases_sot(stack_of_tasks,2E3));
    std::cout<<"Solver started"<<std::endl;
    solved = false;
    do{
        solved = solver->solve(contact_wrenches_d);
    }while(!solved);
    std::cout<<"contact_wrenches_d w constraint 2:"<<std::endl;
    std::cout<<"    r_sole = ["<<contact_wrenches_d.segment(0,6)<<"]"<<std::endl;
    std::cout<<"    l_sole = ["<<contact_wrenches_d.segment(6,6)<<"]"<<std::endl;
    std::cout<<"    LSoftHand = ["<<contact_wrenches_d.segment(12,6)<<"]"<<std::endl;

    Ax = A*contact_wrenches_d;
    std::cout<<"A = \n"<<A<<std::endl;
    std::cout<<"A*x = \n"<<Ax<<std::endl;
    std::cout<<"b = \n"<<b<<std::endl;

    for(unsigned int i = 0; i < b.rows(); ++i)
        EXPECT_NEAR(Ax[i],b[i], 1E-6);

    Aineq = friction_cones->getAineq();
    bUpperBoud = friction_cones->getbUpperBound();
    Aineqx = Aineq*contact_wrenches_d;
    std::cout<<"Aineq = \n"<<Aineq<<std::endl;
    std::cout<<"Aineq*x = \n"<<Aineqx<<std::endl;
    std::cout<<"bUpperBound = \n"<<bUpperBoud<<std::endl;

    for(unsigned int i = 0; i < bUpperBoud.rows(); ++i)
        EXPECT_LE(Aineqx[i],bUpperBoud[i]);

}
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

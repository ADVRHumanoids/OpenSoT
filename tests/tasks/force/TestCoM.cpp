#include <gtest/gtest.h>
#include <OpenSoT/solvers/QPOases.h>
#include <OpenSoT/tasks/force/CoM.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <cmath>
#include <idynutils/tests_utils.h>
#include <idynutils/RobotUtils.h>
#include <fstream>
#include <OpenSoT/tasks/velocity/Interaction.h>
#include <OpenSoT/tasks/velocity/Postural.h>
#include <OpenSoT/constraints/velocity/all.h>
#include <OpenSoT/tasks/Aggregated.h>
#include <idynutils/cartesian_utils.h>
#include <OpenSoT/constraints/force/FrictionCone.h>
#include <OpenSoT/constraints/velocity/Dynamics.h>
#include <ros/ros.h>


using namespace yarp::math;

namespace{

class testForceCoM : public ::testing::Test {
 protected:

  testForceCoM()
  {

  }

  virtual ~testForceCoM() {
  }

  virtual void SetUp() {
  }

  virtual void TearDown() {
  }


};

yarp::sig::Vector getGoodInitialPosition1(iDynUtils& idynutils) {
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
    arm[1] = -arm[1];
    idynutils.fromRobotToIDyn(arm, q, idynutils.right_arm);
    return q;
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
    arm[0] = 0.0 * M_PI/180.0;
    arm[1] = 10.0 * M_PI/180.0;
    arm[3] = 0.0 * M_PI/180.0;
    idynutils.fromRobotToIDyn(arm, q, idynutils.left_arm);
    arm[1] = -arm[1];
    idynutils.fromRobotToIDyn(arm, q, idynutils.right_arm);
    return q;
}

TEST_F(testForceCoM, testForceCoM_StaticCase) {
    iDynUtils coman("coman",
                    std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.urdf",
                    std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.srdf");
    yarp::sig::Vector q = getGoodInitialPosition1(coman);

    coman.updateiDyn3Model(q, true);

    std::vector<std::string> links_in_contact;
    links_in_contact.push_back("r_sole");
    links_in_contact.push_back("l_sole");
    links_in_contact.push_back("LSoftHand");

    std::vector<std::string>::iterator it;
    for(it = links_in_contact.begin();
        it != links_in_contact.end(); it++)
        std::cout<<"link in contact "<<":"<<*it<<std::endl;


    Eigen::VectorXd contact_wrenches_d(6*links_in_contact.size());
    contact_wrenches_d.setZero(contact_wrenches_d.rows());
    OpenSoT::tasks::force::CoM::Ptr force_com_task(
        new OpenSoT::tasks::force::CoM(contact_wrenches_d, links_in_contact, coman));
    force_com_task->update(contact_wrenches_d);

    Eigen::MatrixXd A = force_com_task->getA();
    EXPECT_EQ(A.rows(), 6);
    EXPECT_EQ(A.cols(), 6*links_in_contact.size());
    std::cout<<"A = [ "<<A<<" ]"<<std::endl;

    Eigen::VectorXd b = force_com_task->getb();
    EXPECT_EQ(b.rows(), 6);
    std::cout<<"b = [ "<<b<<" ]"<<std::endl;

    OpenSoT::solvers::QPOases_sot::Stack stack_of_tasks;
    stack_of_tasks.push_back(force_com_task);

    OpenSoT::solvers::QPOases_sot::Ptr sot(
                new OpenSoT::solvers::QPOases_sot(stack_of_tasks,2E3));
    std::cout<<"Solver started"<<std::endl;
    sot->solve(contact_wrenches_d);
    std::cout<<"contact_wrenches_d = [ "<<contact_wrenches_d<<" ]"<<std::endl;

    yarp::sig::Matrix M(6+coman.iDyn3_model.getNrOfDOFs(),
                        6+coman.iDyn3_model.getNrOfDOFs());
    coman.iDyn3_model.getFloatingBaseMassMatrix(M);
    double m = M(0,0);

    EXPECT_NEAR(contact_wrenches_d[2] + contact_wrenches_d[8]
            + contact_wrenches_d[14], m*9.81, 1E-6);

    Eigen::VectorXd Ax = A*contact_wrenches_d;
    std::cout<<"A*x = \n"<<Ax<<std::endl;
    std::cout<<"b = \n"<<b<<std::endl;

    for(unsigned int i = 0; i < b.rows(); ++i)
        EXPECT_NEAR(Ax[i],b[i], 1E-6);

}


}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

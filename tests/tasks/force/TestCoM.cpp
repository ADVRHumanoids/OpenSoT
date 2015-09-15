#include <gtest/gtest.h>
#include <OpenSoT/solvers/QPOases.h>
#include <OpenSoT/tasks/force/CoM.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <cmath>
#include <idynutils/tests_utils.h>
#include <idynutils/RobotUtils.h>
#include <fstream>

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
    arm[1] = -arm[1];
    idynutils.fromRobotToIDyn(arm, q, idynutils.right_arm);
    return q;
}

TEST_F(testForceCoM, testForceCoM1) {
    iDynUtils coman("coman",
                    std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.urdf",
                    std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.srdf");
    yarp::sig::Vector q = getGoodInitialPosition(coman);

    coman.updateiDyn3Model(q, true);
    std::list<std::string> links_in_contact = coman.getLinksInContact();
    links_in_contact.push_back("l_hand_upper_right_link");
    coman.setLinksInContact(links_in_contact);
    links_in_contact.clear();
    links_in_contact = coman.getLinksInContact();
    std::list<std::string>::iterator it;
    for(it = links_in_contact.begin();
        it != links_in_contact.end(); it++)
        std::cout<<"link in contact "<<":"<<*it<<std::endl;

    yarp::sig::Matrix M(6+coman.iDyn3_model.getNrOfDOFs(), 6+coman.iDyn3_model.getNrOfDOFs());
    coman.iDyn3_model.getFloatingBaseMassMatrix(M);
    double m = M(0,0);

    yarp::sig::Vector wrench_d(18,0.0);
    OpenSoT::tasks::force::CoM::Ptr force_com_task(
                new OpenSoT::tasks::force::CoM(wrench_d, coman));
    force_com_task->update(wrench_d);
    yarp::sig::Vector com_d = force_com_task->getActualPosition();
    com_d(0) += 0.01;
    force_com_task->setReference(com_d);

    yarp::sig::Matrix A = force_com_task->getA();
    EXPECT_EQ(A.rows(), 3);
    EXPECT_EQ(A.cols(), 2*3*3);
    std::cout<<"A = [ "<<A.toString()<<" ]"<<std::endl;

    yarp::sig::Vector b = force_com_task->getb();
    EXPECT_DOUBLE_EQ(b(2), m*9.81);
    std::cout<<"b = [ "<<b.toString()<<" ]"<<std::endl;

    OpenSoT::solvers::QPOases_sot::Stack stack_of_tasks;
    stack_of_tasks.push_back(force_com_task);

    OpenSoT::solvers::QPOases_sot::Ptr sot(
                new OpenSoT::solvers::QPOases_sot(stack_of_tasks,2E10));
    sot->solve(wrench_d);
    std::cout<<"wrench_d = [ "<<wrench_d.toString()<<" ]"<<std::endl;


}
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

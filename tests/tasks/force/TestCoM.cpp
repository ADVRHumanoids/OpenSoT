#include <gtest/gtest.h>
#include <OpenSoT/solvers/iHQP.h>
#include <OpenSoT/tasks/force/CoM.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <cmath>
#include <advr_humanoids_common_utils/test_utils.h>
#include <fstream>
#include <OpenSoT/tasks/Aggregated.h>
#include <OpenSoT/utils/cartesian_utils.h>
#include <OpenSoT/constraints/force/FrictionCone.h>
#include <ros/ros.h>
#include <ModelInterfaceIDYNUTILS/ModelInterfaceIDYNUTILS.h>
#include <qpOASES.hpp>
#include <OpenSoT/tasks/force/Wrench.h>
#include <OpenSoT/constraints/force/WrenchLimits.h>


using namespace yarp::math;

typedef idynutils2 iDynUtils;
static void null_deleter(iDynUtils *) {}

std::string robotology_root = std::getenv("ROBOTOLOGY_ROOT");
std::string relative_path = "/external/OpenSoT/tests/configs/coman/configs/config_coman.yaml";

std::string _path_to_cfg = robotology_root + relative_path;

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

Eigen::VectorXd getGoodInitialPosition1(iDynUtils& _robot) {
    Eigen::VectorXd _q(_robot.iDynTree_model.getNrOfDOFs());
    _q.setZero(_q.size());
    _q[_robot.iDynTree_model.getDOFIndex("RHipSag")] = -25.0*M_PI/180.0;
    _q[_robot.iDynTree_model.getDOFIndex("RKneeSag")] = 50.0*M_PI/180.0;
    _q[_robot.iDynTree_model.getDOFIndex("RAnkSag")] = -25.0*M_PI/180.0;

    _q[_robot.iDynTree_model.getDOFIndex("LHipSag")] = -25.0*M_PI/180.0;
    _q[_robot.iDynTree_model.getDOFIndex("LKneeSag")] = 50.0*M_PI/180.0;
    _q[_robot.iDynTree_model.getDOFIndex("LAnkSag")] = -25.0*M_PI/180.0;

    _q[_robot.iDynTree_model.getDOFIndex("LShSag")] =  -90.0*M_PI/180.0;
    _q[_robot.iDynTree_model.getDOFIndex("LForearmPlate")] =  -90.0*M_PI/180.0;

    _q[_robot.iDynTree_model.getDOFIndex("RShSag")] =  -90.0*M_PI/180.0;
    _q[_robot.iDynTree_model.getDOFIndex("RForearmPlate")] =  90.0*M_PI/180.0;
    return _q;
}


TEST_F(testForceCoM, testForceCoM_StaticCase) {
    iDynUtils coman("coman",
                    std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.urdf",
                    std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.srdf");

    XBot::ModelInterfaceIDYNUTILS::Ptr _model_ptr;
    _model_ptr = std::dynamic_pointer_cast<XBot::ModelInterfaceIDYNUTILS>
            (XBot::ModelInterface::getModel(_path_to_cfg));
    _model_ptr->loadModel(boost::shared_ptr<iDynUtils>(&coman, &null_deleter));

    if(_model_ptr)
        std::cout<<"pointer address: "<<_model_ptr.get()<<std::endl;
    else
        std::cout<<"pointer is NULL "<<_model_ptr.get()<<std::endl;



    Eigen::VectorXd q = getGoodInitialPosition1(coman);

    coman.updateiDynTreeModel(q, true);

    std::vector<std::string> links_in_contact;
    links_in_contact.push_back("r_sole");
    links_in_contact.push_back("l_sole");
    links_in_contact.push_back("LSoftHand");

    std::vector<std::string>::iterator it;
    for(it = links_in_contact.begin();
        it != links_in_contact.end(); it++)
        std::cout<<"link in contact "<<":"<<*it<<std::endl;


    Eigen::VectorXd contact_wrenches_d(6*links_in_contact.size());
    contact_wrenches_d.setZero(contact_wrenches_d.size());
    OpenSoT::tasks::force::CoM::Ptr force_com_task(
        new OpenSoT::tasks::force::CoM(contact_wrenches_d, links_in_contact, *(_model_ptr.get())));
    force_com_task->update(contact_wrenches_d);

    Eigen::MatrixXd A = force_com_task->getA();
    EXPECT_EQ(A.rows(), 6);
    EXPECT_EQ(A.cols(), 6*links_in_contact.size());
    std::cout<<"A = [ "<<A<<" ]"<<std::endl;

    Eigen::VectorXd b = force_com_task->getb();
    EXPECT_EQ(b.rows(), 6);
    std::cout<<"b = [ "<<b<<" ]"<<std::endl;


    OpenSoT::constraints::force::FrictionCone::friction_cones friction_cones;
    OpenSoT::constraints::force::FrictionCone::friction_cone friction_cone;
    double mu = 2.;
    for(unsigned int i = 0; i < links_in_contact.size(); ++i)
    {
        friction_cone.first = links_in_contact[i];
        friction_cone.second = mu;
        friction_cones.push_back(friction_cone);
    }
    OpenSoT::constraints::force::FrictionCone::Ptr fc(
                new OpenSoT::constraints::force::FrictionCone(contact_wrenches_d, *(_model_ptr.get()),
                                                              friction_cones));
    std::cout<<"Aineq = ["<<fc->getAineq()<<" ]"<<std::endl;
    std::cout<<"bUpper = ["<<fc->getbUpperBound()<<" "<<std::endl;
    std::cout<<"bLower = ["<<fc->getbLowerBound()<<" "<<std::endl;

    OpenSoT::constraints::force::WrenchLimits::Ptr wrench_limits(
                new OpenSoT::constraints::force::WrenchLimits(300., contact_wrenches_d.size()));

    force_com_task->getConstraints().push_back(fc);
    force_com_task->getConstraints().push_back(wrench_limits);
    force_com_task->update(contact_wrenches_d);

    OpenSoT::tasks::force::Wrench::Ptr wrench(
                new OpenSoT::tasks::force::Wrench(contact_wrenches_d));
    Eigen::VectorXd wrench_reference(contact_wrenches_d.size());
    wrench_reference.setZero(contact_wrenches_d.size());
    wrench_reference(2) = 100.;
    wrench_reference(8) = 100.;
    wrench_reference(14) = 100.;
    wrench->setReference(wrench_reference);
    wrench->getConstraints().push_back(fc);
    wrench->getConstraints().push_back(wrench_limits);
    wrench->update(contact_wrenches_d);
    std::cout<<"A = [ "<<wrench->getA()<<" ]"<<std::endl;
    std::cout<<"b = [ "<<wrench->getb()<<" ]"<<std::endl;





    OpenSoT::solvers::iHQP::Stack stack_of_tasks;
    stack_of_tasks.push_back(force_com_task);
    stack_of_tasks.push_back(wrench);


    OpenSoT::solvers::iHQP::Ptr sot(
                new OpenSoT::solvers::iHQP(stack_of_tasks,1E7));
    std::cout<<"Solver started"<<std::endl;


    coman.updateiDynTreeModel(q, true);
    force_com_task->update(contact_wrenches_d);
    wrench->update(contact_wrenches_d);




    EXPECT_TRUE(sot->solve(contact_wrenches_d));
    std::cout<<"contact_wrenches_d = [ "<<contact_wrenches_d<<" ]"<<std::endl;

    yarp::sig::Matrix M(6+coman.iDynTree_model.getNrOfDOFs(),
                        6+coman.iDynTree_model.getNrOfDOFs());
    coman.iDynTree_model.getFloatingBaseMassMatrix(M);
    double m = M(0,0);

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


}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

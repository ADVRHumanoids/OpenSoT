#include <idynutils/tests_utils.h>
#include <ModelInterfaceIDYNUTILS/ModelInterfaceIDYNUTILS.h>
#include <gtest/gtest.h>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <OpenSoT/constraints/Aggregated.h>
#include <OpenSoT/tasks/Aggregated.h>
#include <OpenSoT/solvers/QPOases.h>
#include <OpenSoT/tasks/velocity/MinimizeAcceleration.h>
#include <OpenSoT/tasks/velocity/Manipulability.h>
#include <OpenSoT/tasks/velocity/Postural.h>
#include <OpenSoT/constraints/velocity/JointLimits.h>
#include <OpenSoT/constraints/velocity/VelocityLimits.h>
#include <qpOASES.hpp>
#include <yarp/math/Math.h>
#include <yarp/sig/all.h>
#include <fstream>
#include <advr_humanoids_common_utils/conversion_utils_YARP.h>

using namespace OpenSoT::constraints::velocity;
using namespace OpenSoT::tasks::velocity;
using namespace yarp::math;

typedef idynutils2 iDynUtils;
static void null_deleter(iDynUtils *) {}

namespace {

class testManipolability: public ::testing::Test
{
protected:
    std::ofstream _log;

    testManipolability()
    {
         //_log.open("testMinimizeAcceleration.m");
    }

    virtual ~testManipolability() {
         //_log.close();
    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }

};

TEST_F(testManipolability, testManipolabilityTask)
{
    iDynUtils idynutils("coman",
                            std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.urdf",
                            std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.srdf");

    XBot::ModelInterfaceIDYNUTILS::Ptr _model_ptr;
    std::string robotology_root = std::getenv("ROBOTOLOGY_ROOT");
    std::string relative_path = "/external/OpenSoT/tests/configs/coman/configs/config_coman.yaml";

    std::string _path_to_cfg = robotology_root + relative_path;

    _model_ptr = std::dynamic_pointer_cast<XBot::ModelInterfaceIDYNUTILS>
            (XBot::ModelInterface::getModel(_path_to_cfg));
    _model_ptr->loadModel(boost::shared_ptr<iDynUtils>(&idynutils, &null_deleter));

    if(_model_ptr)
        std::cout<<"pointer address: "<<_model_ptr.get()<<std::endl;
    else
        std::cout<<"pointer is NULL "<<_model_ptr.get()<<std::endl;


    yarp::sig::Vector q(idynutils.iDynTree_model.getNrOfDOFs(), 0.0);

    idynutils.updateiDynTreeModel(conversion_utils_YARP::toEigen(q), true);

    /// Cartesian Tasks
    yarp::sig::Matrix TL_init = idynutils.iDynTree_model.getPosition(
                          idynutils.iDynTree_model.getLinkIndex("Waist"),
                          idynutils.iDynTree_model.getLinkIndex("l_wrist"));
    yarp::sig::Matrix TR_init = idynutils.iDynTree_model.getPosition(
                          idynutils.iDynTree_model.getLinkIndex("Waist"),
                          idynutils.iDynTree_model.getLinkIndex("r_wrist"));
    Cartesian::Ptr cartesian_task_L(new Cartesian("cartesian::left_wrist",
        conversion_utils_YARP::toEigen(q), *(_model_ptr.get()),"l_wrist", "Waist"));
    Cartesian::Ptr cartesian_task_R(new Cartesian("cartesian::right_wrist",
        conversion_utils_YARP::toEigen(q), *(_model_ptr.get()),"r_wrist", "Waist"));
    std::list< OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr > task_list;
    task_list.push_back(cartesian_task_L);
    task_list.push_back(cartesian_task_R);
    OpenSoT::tasks::Aggregated::Ptr cartesian_task(
                new OpenSoT::tasks::Aggregated(task_list, q.size()));


    /// Postural Task
    Postural::Ptr postural_task(new Postural(conversion_utils_YARP::toEigen(q)));

    /// Manipulability task
    Manipulability::Ptr manipulability_task_L(new Manipulability(
                                                  conversion_utils_YARP::toEigen(q), *(_model_ptr.get()), cartesian_task_L));
    Manipulability::Ptr manipulability_task_R(new Manipulability(
                                                  conversion_utils_YARP::toEigen(q), *(_model_ptr.get()), cartesian_task_R));
    std::list< OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr > manip_list;
    manip_list.push_back(manipulability_task_L);
    manip_list.push_back(manipulability_task_R);
    OpenSoT::tasks::Aggregated::Ptr manipulability_task(
                new OpenSoT::tasks::Aggregated(manip_list, q.size()));

    /// Constraints set to the Cartesian Task
    int t = 1000;
    JointLimits::Ptr joint_limits(
        new JointLimits(conversion_utils_YARP::toEigen(q), idynutils.getJointBoundMax(),
                           idynutils.getJointBoundMin(), 0.2));
    VelocityLimits::Ptr joint_velocity_limits(
                new VelocityLimits(M_PI/2.0, (double)(1.0/t), q.size()));

    std::list< OpenSoT::Constraint<Eigen::MatrixXd, Eigen::VectorXd>::ConstraintPtr > joint_constraints_list;
    joint_constraints_list.push_back(joint_limits);
    joint_constraints_list.push_back(joint_velocity_limits);
    OpenSoT::constraints::Aggregated::Ptr joint_constraints(
                new OpenSoT::constraints::Aggregated(joint_constraints_list, q.size()));

    std::vector< OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr > stack_of_tasks;
    stack_of_tasks.push_back(cartesian_task);
    stack_of_tasks.push_back(postural_task);

    OpenSoT::solvers::QPOases_sot sot(stack_of_tasks, joint_constraints);

    yarp::sig::Vector dq(q.size(), 0.0);

    for(unsigned int i = 0; i < 3*t; ++i)
    {
        idynutils.updateiDynTreeModel(conversion_utils_YARP::toEigen(q), true);

        cartesian_task->update(conversion_utils_YARP::toEigen(q));
        postural_task->update(conversion_utils_YARP::toEigen(q));
        manipulability_task->update(conversion_utils_YARP::toEigen(q));
        joint_constraints->update(conversion_utils_YARP::toEigen(q));

        Eigen::VectorXd _dq(dq.size()); _dq.setZero(dq.size());
        sot.solve(_dq);
        dq = conversion_utils_YARP::toYARP(_dq);
        q += dq;

        double manip_index_L = sqrt(det(
        conversion_utils_YARP::toYARP(cartesian_task_L->getA())*
        conversion_utils_YARP::toYARP(cartesian_task_L->getA()).transposed()));
        double manip_index_R = sqrt(det(
        conversion_utils_YARP::toYARP(cartesian_task_R->getA())*
        conversion_utils_YARP::toYARP(cartesian_task_R->getA()).transposed()));

        ASSERT_NEAR(manipulability_task_L->ComputeManipulabilityIndex(), manip_index_L, 1e-10);
        ASSERT_NEAR(manipulability_task_R->ComputeManipulabilityIndex(), manip_index_R, 1e-10);
    }
    double manip_index_L = sqrt(det(
    conversion_utils_YARP::toYARP(cartesian_task_L->getA())*
    conversion_utils_YARP::toYARP(cartesian_task_L->getA()).transposed()));
    double manip_index_R = sqrt(det(
    conversion_utils_YARP::toYARP(cartesian_task_R->getA())*
    conversion_utils_YARP::toYARP(cartesian_task_R->getA()).transposed()));

    yarp::sig::Vector q_init(idynutils.iDynTree_model.getNrOfDOFs(), 0.0);
    q_init = q;
    idynutils.updateiDynTreeModel(conversion_utils_YARP::toEigen(q));

    std::cout<<"INITIAL CONFIG: "<<TL_init.toString()<<std::endl;
    yarp::sig::Matrix TL = idynutils.iDynTree_model.getPosition(
                idynutils.iDynTree_model.getLinkIndex("Waist"),
                idynutils.iDynTree_model.getLinkIndex("l_wrist"));
    std::cout<<"FINAL CONFIG: "<<TL.toString()<<std::endl;

    for(unsigned int i = 0; i <= 3; ++i){
        for(unsigned int j = 0; j <= 3; ++j)
            EXPECT_NEAR(TL_init(i,j), TL(i,j), 1E-3);
    }

    std::cout<<"INITIAL CONFIG: "<<TR_init.toString()<<std::endl;
    yarp::sig::Matrix TR = idynutils.iDynTree_model.getPosition(
                idynutils.iDynTree_model.getLinkIndex("Waist"),
                idynutils.iDynTree_model.getLinkIndex("r_wrist"));
    std::cout<<"FINAL CONFIG: "<<TR.toString()<<std::endl;

    for(unsigned int i = 0; i <= 3; ++i){
        for(unsigned int j = 0; j <= 3; ++j)
            EXPECT_NEAR(TR_init(i,j), TR(i,j), 1E-3);
    }


    stack_of_tasks.clear();
    stack_of_tasks.push_back(cartesian_task);
    stack_of_tasks.push_back(manipulability_task);
    OpenSoT::solvers::QPOases_sot sot_manip(stack_of_tasks, joint_constraints);

    for(unsigned int i = 0; i < 3*t; ++i)
    {
        idynutils.updateiDynTreeModel(conversion_utils_YARP::toEigen(q), true);

        cartesian_task->update(conversion_utils_YARP::toEigen(q));
        postural_task->update(conversion_utils_YARP::toEigen(q));
        manipulability_task->update(conversion_utils_YARP::toEigen(q));
        joint_constraints->update(conversion_utils_YARP::toEigen(q));

        Eigen::VectorXd _dq(dq.size()); _dq.setZero(dq.size());
        sot_manip.solve(_dq);
        dq = conversion_utils_YARP::toYARP(_dq);
        q += dq;

        double manip_index_L = sqrt(det(
                    conversion_utils_YARP::toYARP(cartesian_task_L->getA())*
                    conversion_utils_YARP::toYARP(cartesian_task_L->getA()).transposed()));
        double manip_index_R = sqrt(det(
                    conversion_utils_YARP::toYARP(cartesian_task_R->getA())*
                    conversion_utils_YARP::toYARP(cartesian_task_R->getA()).transposed()));

        EXPECT_NEAR(manipulability_task_L->ComputeManipulabilityIndex(), manip_index_L, 1e-10);
        EXPECT_NEAR(manipulability_task_R->ComputeManipulabilityIndex(), manip_index_R, 1e-10);
    }
    double new_manip_index_L = sqrt(det(
        conversion_utils_YARP::toYARP(cartesian_task_L->getA())*
        conversion_utils_YARP::toYARP(cartesian_task_L->getA()).transposed()));
    double new_manip_index_R = sqrt(det(
        conversion_utils_YARP::toYARP(cartesian_task_R->getA())*
        conversion_utils_YARP::toYARP(cartesian_task_R->getA()).transposed()));

    idynutils.updateiDynTreeModel(conversion_utils_YARP::toEigen(q));

    std::cout<<"INITIAL CONFIG: "<<TL_init.toString()<<std::endl;
    TL = idynutils.iDynTree_model.getPosition(
                idynutils.iDynTree_model.getLinkIndex("Waist"),
                idynutils.iDynTree_model.getLinkIndex("l_wrist"));
    std::cout<<"FINAL CONFIG: "<<TL.toString()<<std::endl;

    for(unsigned int i = 0; i <= 3; ++i){
        for(unsigned int j = 0; j <= 3; ++j)
            EXPECT_NEAR(TL_init(i,j), TL(i,j), 1E-3);
    }

    std::cout<<"INITIAL CONFIG: "<<TR_init.toString()<<std::endl;
    TR = idynutils.iDynTree_model.getPosition(
                idynutils.iDynTree_model.getLinkIndex("Waist"),
                idynutils.iDynTree_model.getLinkIndex("r_wrist"));
    std::cout<<"FINAL CONFIG: "<<TR.toString()<<std::endl;

    for(unsigned int i = 0; i <= 3; ++i){
        for(unsigned int j = 0; j <= 3; ++j)
            EXPECT_NEAR(TR_init(i,j), TR(i,j), 1E-3);
    }

    std::cout<<std::endl;

    std::cout<<"INITIAL MANIP INDEX L: "<<manip_index_L<<std::endl;
    std::cout<<"FINAL MANIP INDEX L: "<<new_manip_index_L<<std::endl;
    std::cout<<"INITIAL MANIP INDEX R: "<<manip_index_R<<std::endl;
    std::cout<<"FINAL MANIP INDEX R: "<<new_manip_index_R<<std::endl;

    EXPECT_TRUE(manip_index_L <= new_manip_index_L);
    EXPECT_TRUE(manip_index_R <= new_manip_index_R);

    std::cout<<std::endl;

//    std::cout<<"q_init left_arm: ";
//    for(unsigned int i = 0; i < idynutils.left_arm.getNrOfDOFs(); ++i)
//        std::cout<<q_init[idynutils.left_arm.joint_numbers[i]]<<" ";
//    std::cout<<std::endl;
//    std::cout<<"q_final left_arm: ";
//    for(unsigned int i = 0; i < idynutils.left_arm.getNrOfDOFs(); ++i)
//        std::cout<<q[idynutils.left_arm.joint_numbers[i]]<<" ";
//    std::cout<<std::endl;

//    std::cout<<std::endl;

//    std::cout<<"q_init right_arm: ";
//    for(unsigned int i = 0; i < idynutils.right_arm.getNrOfDOFs(); ++i)
//        std::cout<<q_init[idynutils.right_arm.joint_numbers[i]]<<" ";
//    std::cout<<std::endl;
//    std::cout<<"q_final right_arm: ";
//    for(unsigned int i = 0; i < idynutils.right_arm.getNrOfDOFs(); ++i)
//        std::cout<<q[idynutils.right_arm.joint_numbers[i]]<<" ";
//    std::cout<<std::endl;

//    std::cout<<std::endl;

//    std::cout<<"q_init torso: ";
//    for(unsigned int i = 0; i < idynutils.torso.getNrOfDOFs(); ++i)
//        std::cout<<q_init[idynutils.torso.joint_numbers[i]]<<" ";
//    std::cout<<std::endl;
//    std::cout<<"q_final torso: ";
//    for(unsigned int i = 0; i < idynutils.torso.getNrOfDOFs(); ++i)
//        std::cout<<q[idynutils.torso.joint_numbers[i]]<<" ";
//    std::cout<<std::endl;

}
}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

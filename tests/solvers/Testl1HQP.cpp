#include <gtest/gtest.h>
#include <OpenSoT/solvers/QPOasesBackEnd.h>
#include <OpenSoT/solvers/l1HQP.h>
#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <OpenSoT/tasks/velocity/CoM.h>
#include <OpenSoT/tasks/velocity/Postural.h>
#include <OpenSoT/tasks/GenericTask.h>
#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/constraints/velocity/JointLimits.h>

std::string robotology_root = std::getenv("ROBOTOLOGY_ROOT");
std::string relative_path = "/external/OpenSoT/tests/configs/coman/configs/config_coman_RBDL.yaml";
std::string _path_to_cfg = robotology_root + relative_path;

namespace {
class testl1HQP: public ::testing::Test
{
protected:

    testl1HQP()
    {

    }

    virtual ~testl1HQP() {

    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }

    Eigen::VectorXd getGoodInitialPosition(XBot::ModelInterface::Ptr _model_ptr) {
        Eigen::VectorXd _q(_model_ptr->getJointNum());
        _q.setZero(_q.size());
        _q[_model_ptr->getDofIndex("RHipSag")] = -25.0*M_PI/180.0;
        _q[_model_ptr->getDofIndex("RKneeSag")] = 50.0*M_PI/180.0;
        _q[_model_ptr->getDofIndex("RAnkSag")] = -25.0*M_PI/180.0;

        _q[_model_ptr->getDofIndex("LHipSag")] = -25.0*M_PI/180.0;
        _q[_model_ptr->getDofIndex("LKneeSag")] = 50.0*M_PI/180.0;
        _q[_model_ptr->getDofIndex("LAnkSag")] = -25.0*M_PI/180.0;

        _q[_model_ptr->getDofIndex("LShSag")] =  20.0*M_PI/180.0;
        _q[_model_ptr->getDofIndex("LShLat")] = 10.0*M_PI/180.0;
        _q[_model_ptr->getDofIndex("LElbj")] = -80.0*M_PI/180.0;

        _q[_model_ptr->getDofIndex("RShSag")] =  20.0*M_PI/180.0;
        _q[_model_ptr->getDofIndex("RShLat")] = -10.0*M_PI/180.0;
        _q[_model_ptr->getDofIndex("RElbj")] = -80.0*M_PI/180.0;

        _model_ptr->setJointPosition(_q);
        _model_ptr->update();

        return _q;
    }

};


TEST_F(testl1HQP, testContructor)
{
    XBot::ModelInterface::Ptr model_ptr;
    model_ptr = XBot::ModelInterface::getModel(_path_to_cfg);

    Eigen::VectorXd q = this->getGoodInitialPosition(model_ptr);

    OpenSoT::tasks::velocity::Cartesian::Ptr l_sole =
            boost::make_shared<OpenSoT::tasks::velocity::Cartesian>("l_sole", q, *model_ptr, "l_sole", "world");
    OpenSoT::tasks::velocity::Cartesian::Ptr r_sole =
            boost::make_shared<OpenSoT::tasks::velocity::Cartesian>("r_sole", q, *model_ptr, "r_sole", "world");
    OpenSoT::tasks::velocity::CoM::Ptr CoM =
            boost::make_shared<OpenSoT::tasks::velocity::CoM>(q, *model_ptr);

    Eigen::VectorXd qmin, qmax;
    model_ptr->getJointLimits(qmin, qmax);
    OpenSoT::constraints::velocity::JointLimits::Ptr joint_limits =
            boost::make_shared<OpenSoT::constraints::velocity::JointLimits>(q, qmax, qmin);

    OpenSoT::AutoStack::Ptr stack = ((l_sole + r_sole)/CoM)<<joint_limits;
    stack->update(q);

    OpenSoT::solvers::l1HQP::Ptr l1_solver =
            boost::make_shared<OpenSoT::solvers::l1HQP>(*stack);

    std::map<std::string, OpenSoT::tasks::GenericLPTask::Ptr> linear_tasks = l1_solver->getTasks();
    EXPECT_EQ(linear_tasks.size(), stack->getStack().size());
    std::cout<<"linear_tasks.size(): "<<linear_tasks.size()<<std::endl;
    for(std::map<std::string, OpenSoT::tasks::GenericLPTask::Ptr>::iterator it = linear_tasks.begin();
        it != linear_tasks.end(); it++)
    {
        std::cout<<it->first<<" gain: "<<it->second->getc().transpose()<<std::endl;
    }

    std::map<std::string, OpenSoT::solvers::task_to_constraint_helper::Ptr> constraints = l1_solver->getConstraints();
    std::cout<<"constraints.size(): "<<constraints.size()<<std::endl;
    EXPECT_EQ(constraints.size(), stack->getStack().size());
    std::map<std::string, OpenSoT::solvers::task_to_constraint_helper::Ptr>::iterator it2;
    unsigned int i = 0;
    OpenSoT::utils::MatrixPiler b_lower(1);
    OpenSoT::utils::MatrixPiler b_upper(1);
    OpenSoT::utils::MatrixPiler A_ineq(l1_solver->getVariableSize());
    for(it2 = constraints.begin(); it2 != constraints.end(); it2++)
    {
        OpenSoT::solvers::task_to_constraint_helper::Ptr constraint = it2->second;

        // These checks are done considering that the task_to_constraint_heper is implemented
        // such that:
        //          -t <= Ax -b <= t
        //           t >= 0

        std::cout<<"constraint->getbLowerBound(): "<<constraint->getbLowerBound().transpose()<<std::endl;
        EXPECT_EQ(constraint->getbLowerBound().size(), stack->getStack()[i]->getb().size()*3);

        std::cout<<"constraint->getbUpperBound(): "<<constraint->getbUpperBound().transpose()<<std::endl;
        EXPECT_EQ(constraint->getbUpperBound().size(), stack->getStack()[i]->getb().size()*3);

        //std::cout<<"constraint->getAineq()"<<constraint->getAineq()<<std::endl;
        EXPECT_EQ(constraint->getAineq().rows(), stack->getStack()[i]->getA().rows()*3);
        EXPECT_EQ(constraint->getAineq().cols(), l1_solver->getVariableSize());

        i += 1;
        b_lower.pile(constraint->getbLowerBound());
        b_upper.pile(constraint->getbUpperBound());
        A_ineq.pile(constraint->getAineq());
    }

    OpenSoT::solvers::constraint_helper::Ptr hard_constraints = l1_solver->getHardConstraints();
    EXPECT_TRUE(hard_constraints->getbLowerBound() == joint_limits->getLowerBound());
    EXPECT_TRUE(hard_constraints->getbUpperBound() == joint_limits->getUpperBound());

    b_lower.pile(hard_constraints->getbLowerBound());
    b_upper.pile(hard_constraints->getbUpperBound());
    A_ineq.pile(hard_constraints->getAineq());



    OpenSoT::AutoStack::Ptr internal_problem = l1_solver->getInternalProblem();

    std::cout<<"internal_problem->getStack()[0].size(): "<<internal_problem->getStack().size()<<std::endl;
    EXPECT_EQ(internal_problem->getStack().size(), 1);
    std::cout<<"internal_problem->getStack()[0]->getA(): "<<internal_problem->getStack()[0]->getA()<<std::endl;
    EXPECT_EQ(internal_problem->getStack()[0]->getA().rows(), 0);
    std::cout<<"internal_problem->getStack()[0]->getb(): "<<internal_problem->getStack()[0]->getb()<<std::endl;
    EXPECT_EQ(internal_problem->getStack()[0]->getb().size(), 0);
    std::cout<<"internal_problem->getStack()[0]->getc(): "<<internal_problem->getStack()[0]->getc().transpose()<<std::endl;
    EXPECT_EQ(internal_problem->getStack()[0]->getc().size(), linear_tasks.begin()->second->getc().size());
    std::cout<<"Hessian Type: "<<internal_problem->getStack()[0]->getHessianAtype()<<std::endl;
    EXPECT_EQ(internal_problem->getStack()[0]->getHessianAtype(), OpenSoT::HST_ZERO);

    EXPECT_TRUE(internal_problem->getBounds()->getbLowerBound() == b_lower.generate_and_get());
    EXPECT_TRUE(internal_problem->getBounds()->getbUpperBound() == b_upper.generate_and_get());
    EXPECT_TRUE(internal_problem->getBounds()->getAineq() == A_ineq.generate_and_get());

}
}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

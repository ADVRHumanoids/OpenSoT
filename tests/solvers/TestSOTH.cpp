#include <gtest/gtest.h>
//#include <soth/debug.hpp>
//#include <soth/Bound.hpp>
//#include <soth/HCOD.hpp>
#include <OpenSoT/solvers/HCOD.h>
#include <OpenSoT/tasks/GenericTask.h>
#include <OpenSoT/constraints/GenericConstraint.h>



namespace {


class hcod
{
public:
    hcod(const std::vector<Eigen::MatrixXd>& J, const std::vector<soth::VectorBound>& b,
         unsigned int NC, const double eps = 0.0):
        _solver(NC, J.size())
    {
        for (unsigned int i = 0; i < J.size(); ++i)
        {
            _solver.pushBackStage(J[i], b[i]);
            _solver.setNameByOrder("level_");
        }

        _solver.setDamping(eps);
        _solver.setInitialActiveSet();

    }

    void solve(Eigen::VectorXd& solution)
    {
        _solver.activeSearch(solution);
    }



    soth::HCOD& solver(){return _solver;}

private:
    soth::HCOD _solver;

};

class testSOTH: public ::testing::Test
{
protected:

    testSOTH()
    {

    }

    virtual ~testSOTH() {

    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }

};

TEST_F(testSOTH, linearSystem)
{
    std::srand(std::time(nullptr));

    std::vector<Eigen::MatrixXd> J(1);
    std::vector<soth::VectorBound> b(1);

//    J[0].resize(3, 3);
//    b[0].resize(3);
//    J[0].setIdentity();
//    b[0].fill(soth::Bound(0, soth::Bound::BOUND_INF));

    J[0] = Eigen::MatrixXd::Random(3,3);
    Eigen::VectorXd bb(3);
    bb << 1., 1., 1.;
    b[0].resize(3);
    b[0][0] = bb[0];
    b[0][1] = bb[1];
    b[0][2] = bb[2];

    hcod solver(J, b, 3);

    Eigen::VectorXd solution(3);
    solver.solve(solution);

    std::cout<<"SOLVER SHOW:"<<std::endl;
    std::stringstream stream;
    solver.solver().show(stream);
    solver.solver().showActiveSet(stream);
    std::cout<<stream.str()<<std::endl;

    std::cout<<"solution: "<<solution.transpose()<<std::endl;

    auto bsol = J[0]*solution;
    std::cout<<"J*solution: "<<bsol<<std::endl;

    for(unsigned int i = 0; i < 3; ++i)
        EXPECT_NEAR(bsol(i,0), bb[i], 1e-6);

//    /// OPENSOT
//    OpenSoT::AutoStack stack(std::make_shared<OpenSoT::tasks::GenericTask>("0", J[0], bb));

//    OpenSoT::solvers::HCOD hcod(stack, 0.);
//    Eigen::VectorXd solution2;
//    hcod.solve(solution2);

//    for(unsigned int i = 0; i < 3; ++i)
//        EXPECT_NEAR(solution[i], solution2[i], 1e-6);

//    std::cout<<"solution2: "<<solution2.transpose()<<std::endl;
}



TEST_F(testSOTH, constrainedLinearSystem)
{
    std::srand(std::time(nullptr));

    std::vector<Eigen::MatrixXd> J(2);
    std::vector<soth::VectorBound> b(2);

    J[0].resize(3, 3);
    b[0].resize(3);
    J[0].setIdentity();
    b[0].fill(soth::Bound(-0.5, 0.5));
    Eigen::VectorXd lb(3); lb << -0.5,-0.5,-0.5;
    Eigen::VectorXd ub(3); ub << 0.5,0.5,0.5;

    J[1].setIdentity(3,3);
    Eigen::VectorXd bb(3);
    bb << 1., 1., 1.;
    b[1].resize(3);
    b[1][0] = bb[0];
    b[1][1] = bb[1];
    b[1][2] = bb[2];

    hcod solver(J, b, 3);

    Eigen::VectorXd solution(3);
    solver.solve(solution);

    std::cout<<"SOLVER SHOW:"<<std::endl;
    std::stringstream stream;
    solver.solver().show(stream);
    solver.solver().showActiveSet(stream);
    std::cout<<stream.str()<<std::endl;

    std::cout<<"solution: "<<solution.transpose()<<std::endl;

    auto bsol = J[0]*solution;
    std::cout<<"J*solution: "<<bsol<<std::endl;

    for(unsigned int i = 0; i < 3; ++i)
        EXPECT_NEAR(bsol(i,0), 0.5, 1e-6);

//    /// OPENSOT
//    auto constr = std::make_shared<OpenSoT::constraints::GenericConstraint>(
//                    "constr", ub, lb, 3);

//    OpenSoT::AutoStack::Ptr stack = std::make_shared<OpenSoT::AutoStack>
//            (std::make_shared<OpenSoT::tasks::GenericTask>("0", J[1], bb));
//    stack<<constr;



//    OpenSoT::solvers::HCOD hcod(*stack, 0.);
//    Eigen::VectorXd solution2;
//    hcod.solve(solution2);

//    for(unsigned int i = 0; i < 3; ++i)
//        EXPECT_NEAR(solution[i], solution2[i], 1e-6);

//    std::cout<<"solution2: "<<solution2.transpose()<<std::endl;
}

TEST_F(testSOTH, hierarchicalLinearSystem)
{
    std::srand(std::time(nullptr));

    std::vector<Eigen::MatrixXd> J(2);
    std::vector<soth::VectorBound> b(2);

    J[0] = Eigen::MatrixXd::Random(2,3);
    b[0].resize(2);
    Eigen::VectorXd v0 = Eigen::VectorXd::Random(2);
    b[0][0] = v0[0];
    b[0][1] = v0[1];

    J[1] = Eigen::MatrixXd::Random(2,3);
    b[1].resize(2);
    Eigen::VectorXd v1 = Eigen::VectorXd::Random(2);
    b[1][0] = v1[0];
    b[1][1] = v1[1];

    hcod solver(J, b, 3);

    Eigen::VectorXd solution(3);
    solver.solve(solution);

    std::cout<<"SOLVER SHOW:"<<std::endl;
    std::stringstream stream;
    solver.solver().show(stream);
    solver.solver().showActiveSet(stream);
    std::cout<<stream.str()<<std::endl;

    std::cout<<"solution: "<<solution.transpose()<<std::endl;

    auto bsol = J[0]*solution;
    std::cout<<"level 1: J*solution: "<<bsol<<std::endl;

    for(unsigned int i = 0; i < v0.size(); ++i)
        EXPECT_NEAR(bsol(i,0), v0[i], 1e-6);

    auto bsol2 = J[1]*solution;
    std::cout<<"level 2: J*solution: "<<bsol2<<std::endl;

//    /// OPENSOT
//    auto task0 = std::make_shared<OpenSoT::tasks::GenericTask>("0", J[0], v0);
//    auto task1 = std::make_shared<OpenSoT::tasks::GenericTask>("1", J[1], v1);

//    OpenSoT::AutoStack::Ptr stack = (task0/task1);



//    OpenSoT::solvers::HCOD hcod(*stack, 0.);
//    Eigen::VectorXd solution2;
//    hcod.solve(solution2);

//    for(unsigned int i = 0; i < 3; ++i)
//        EXPECT_NEAR(solution[i], solution2[i], 1e-6);

//    std::cout<<"solution2: "<<solution2.transpose()<<std::endl;
}

//TEST_F(testSOTH, constrainedVariableLinearSystem)
//{
//    std::srand(std::time(nullptr));

//    std::vector<Eigen::MatrixXd> J(2);
//    std::vector<soth::VectorBound> b(2);

//    J[0].resize(3, 3);
//    b[0].resize(3);
//    J[0].setIdentity();
//    b[0].fill(soth::Bound(-0.5, 0.5));

//    J[1].setIdentity(3,3);
//    b[1].resize(3);
//    b[1][0] = 1.;
//    b[1][1] = 1.;
//    b[1][2] = 1.;

//    hcod solver(J, b, 3);

//    Eigen::VectorXd solution(3);
//    solver.solve(solution);

//    std::cout<<"SOLVER SHOW:"<<std::endl;
//    std::stringstream stream;
//    solver.solver().show(stream);
//    solver.solver().showActiveSet(stream);
//    std::cout<<stream.str()<<std::endl;

//    std::cout<<"solution: "<<solution.transpose()<<std::endl;

//    auto bsol = J[0]*solution;
//    std::cout<<"J*solution: "<<bsol<<std::endl;

//    for(unsigned int i = 0; i < 3; ++i)
//        EXPECT_NEAR(bsol(i,0), 0.5, 1e-6);


//    b[0].fill(soth::Bound(-1., 1.));
//    solver.solver().stages[0]->set(J[0], b[0]);

//    solver.solve(solution);
//    std::stringstream stream2;
//    solver.solver().show(stream2);
//    std::cout<<std::endl;
//    std::cout<<std::endl;
//    std::cout<<stream2.str()<<std::endl;
//    std::cout<<"solution: "<<solution.transpose()<<std::endl;


//    auto bsol2 = J[0]*solution;
//    std::cout<<"J*solution: "<<bsol2<<std::endl;

//    for(unsigned int i = 0; i < 3; ++i)
//        EXPECT_NEAR(bsol2(i,0), 1., 1e-6);
//}

//TEST_F(testSOTH, constrainedVariableLinearSystemOpenSoT)
//{
//    std::srand(std::time(nullptr));

//    Eigen::VectorXd lb(3); lb << -0.5,-0.5,-0.5;
//    Eigen::VectorXd ub(3); ub << 0.5,0.5,0.5;

//    Eigen::MatrixXd J(3,3);
//    J.setIdentity(3,3);
//    Eigen::VectorXd b(3);
//    b[0] = 1.;
//    b[1] = 1.;
//    b[2] = 1.;

//    OpenSoT::constraints::GenericConstraint::Ptr constr = std::make_shared<OpenSoT::constraints::GenericConstraint>(
//                    "constr", ub, lb, 3);

//    OpenSoT::AutoStack::Ptr stack = std::make_shared<OpenSoT::AutoStack>
//            (std::make_shared<OpenSoT::tasks::GenericTask>("0", J, b));
//    stack<<constr;



//    OpenSoT::solvers::HCOD hcod(*stack, 0.);

//    Eigen::VectorXd solution(3);
//    hcod.solve(solution);

//    std::cout<<"solution: "<<solution.transpose()<<std::endl;

//    auto bsol = J*solution;
//    std::cout<<"J*solution: "<<bsol<<std::endl;

//    for(unsigned int i = 0; i < 3; ++i)
//        EXPECT_NEAR(bsol(i,0), 0.5, 1e-6);


//    lb[0] = -1.; lb[1] = -1.; lb[2] = -1.;
//    ub[0] = 1.; ub[1] = 1.; ub[2] = 1.;

//    constr->setBounds(ub, lb);
//    stack->update(Eigen::VectorXd(1));


//    hcod.solve(solution);
//    std::cout<<"solution: "<<solution.transpose()<<std::endl;


//    auto bsol2 = J*solution;
//    std::cout<<"J*solution: "<<bsol2<<std::endl;

//    for(unsigned int i = 0; i < 3; ++i)
//        EXPECT_NEAR(bsol2(i,0), 1., 1e-6);
//}


}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

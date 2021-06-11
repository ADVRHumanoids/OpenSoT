#include <gtest/gtest.h>
#include <XBotInterface/ModelInterface.h>
#include <soth/HCOD.hpp>
#include <soth/debug.hpp>

class NotificationToCout {
public:
  void operator()(std::string stage, soth::ConstraintRef cst,
                  std::string event) {
    sotDEBUG(0) << "At " << stage << ", " << cst << ": " << event << std::endl;
    count++;
  }
  int count;
  NotificationToCout() { count = 0; }
};

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
    NotificationToCout _coutListen;

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
    b[0].resize(3);
    b[0][0] = 1.;
    b[0][1] = 1.;
    b[0][2] = 1.;

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
        EXPECT_NEAR(bsol(i,0), 1., 1e-6);
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

    J[1].setIdentity(3,3);
    b[1].resize(3);
    b[1][0] = 1.;
    b[1][1] = 1.;
    b[1][2] = 1.;

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
}

TEST_F(testSOTH, constrainedVariableLinearSystem)
{
    std::srand(std::time(nullptr));

    std::vector<Eigen::MatrixXd> J(2);
    std::vector<soth::VectorBound> b(2);

    J[0].resize(3, 3);
    b[0].resize(3);
    J[0].setIdentity();
    b[0].fill(soth::Bound(-0.5, 0.5));

    J[1].setIdentity(3,3);
    b[1].resize(3);
    b[1][0] = 1.;
    b[1][1] = 1.;
    b[1][2] = 1.;

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


    b[0].fill(soth::Bound(-1., 1.));
    solver.solver().stages[0]->set(J[0], b[0]);

    solver.solve(solution);
    std::stringstream stream2;
    solver.solver().show(stream2);
    std::cout<<std::endl;
    std::cout<<std::endl;
    std::cout<<stream2.str()<<std::endl;
    std::cout<<"solution: "<<solution.transpose()<<std::endl;


    auto bsol2 = J[0]*solution;
    std::cout<<"J*solution: "<<bsol2<<std::endl;

    for(unsigned int i = 0; i < 3; ++i)
        EXPECT_NEAR(bsol2(i,0), 1., 1e-6);
}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

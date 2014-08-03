#include <gtest/gtest.h>
#include <wb_sot/solvers/QPOasis.h>
#include <yarp/sig/all.h>

namespace {

class testSolverQPOases: public ::testing::Test,
        public wb_sot::solvers::QPOases<yarp::sig::Matrix, yarp::sig::Vector>
{
protected:

    testSolverQPOases()
    {

    }

    virtual ~testSolverQPOases() {

    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }

};


TEST_F(testSolverQPOases, testSolverInitialization){
    EXPECT_EQ(0, this->getNumberOfConstraints()) <<"Initial number of constraints has to be initialized to 0"<<std::endl;
    EXPECT_EQ(0, this->getNumberOfBounds()) <<"Initial number of bounds has to be initialized to 0"<<std::endl;
    EXPECT_EQ(0, this->getNumberOfStacks()) <<"Initial number of stacks has to be initialized to 0"<<std::endl;
    EXPECT_EQ(0, this->_options.size()) <<"Initial number of options has to be initialized to 0"<<std::endl;
    EXPECT_FALSE(this->_initial_guess) <<"_initial_guess has to be initialized to false"<<std::endl;
}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

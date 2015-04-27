#include <gtest/gtest.h>
#include <OpenSoT/constraints/velocity/CartesianPositionConstraint.h>
#include <OpenSoT/constraints/velocity/SelfCollisionAvoidance.h>
#include <idynutils/idynutils.h>
#include <idynutils/tests_utils.h>
#include <iCub/iDynTree/yarp_kdl.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>
#include <cmath>

#define  s                1.0
#define  dT               0.001* s
#define  m_s              1.0
#define toRad(X) (X * M_PI/180.0)


namespace{


class testSelfCollisionAvoidanceConstraint : public ::testing::Test{
 protected:

  testSelfCollisionAvoidanceConstraint():
      robot("coman",
            std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.urdf",
            std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.srdf"),
      q(robot.iDyn3_model.getNrOfDOFs(), 0.0),
      sc_constraint(q, robot)
  {}

  virtual ~testSelfCollisionAvoidanceConstraint() {
  }

  virtual void SetUp() {
  }

  virtual void TearDown() {
  }

  iDynUtils robot;
  yarp::sig::Vector q;
  OpenSoT::constraints::velocity::SelfCollisionAvoidance sc_constraint;
};


  TEST_F(testSelfCollisionAvoidanceConstraint, testConversions) {

    yarp::sig::Matrix testMatrix(6,6);
    for(unsigned int i = 0; i < testMatrix.rows(); ++i)
        for(unsigned int j = 0; j < testMatrix.cols(); ++j)
            testMatrix(i,j) = i*j+1;

    Eigen::MatrixXd testEigenMatrix = sc_constraint.from_yarp_to_Eigen_matrix(testMatrix);
    yarp::sig::Matrix resultMatrix = sc_constraint.from_Eigen_to_Yarp_matrix(testEigenMatrix);

    for(unsigned int i = 0; i < testMatrix.rows(); ++i)
        for(unsigned int j = 0; j < testMatrix.cols(); ++j)
            EXPECT_DOUBLE_EQ(testMatrix(i,j), resultMatrix(i,j));

  }


}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

#include <gtest/gtest.h>
#include <OpenSoT/constraints/velocity/Dynamics.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <cmath>


using namespace OpenSoT::constraints::velocity;
using namespace yarp::math;

namespace{

class testDynamicsConstr : public ::testing::Test {
 protected:

  testDynamicsConstr()  :
      coman("coman",
            std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.urdf",
            std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.srdf"),
      q(coman.iDyn3_model.getNrOfDOFs()),
      q_dot(q.size())
  {
        q.zero();
        q_dot.zero();
  }

  virtual ~testDynamicsConstr() {
  }

  virtual void SetUp() {
  }

  virtual void TearDown() {
  }

  iDynUtils coman;
  yarp::sig::Vector q;
  yarp::sig::Vector q_dot;
};

TEST_F(testDynamicsConstr, ID_from_iDynThree) {
    coman.updateiDyn3Model(q, true);

    yarp::sig::Vector tau_g = coman.iDyn3_model.getTorques();
    std::cout<<"tau_g @ q = zeros: ["<<tau_g.toString()<<"]"<<std::endl;
    EXPECT_EQ(tau_g.size(), coman.iDyn3_model.getNrOfDOFs());

    yarp::sig::Matrix M(6+coman.iDyn3_model.getNrOfDOFs(), 6+coman.iDyn3_model.getNrOfDOFs());
    coman.iDyn3_model.getFloatingBaseMassMatrix(M);
    EXPECT_DOUBLE_EQ(M.rows(), 6+coman.iDyn3_model.getNrOfDOFs());
    EXPECT_DOUBLE_EQ(M.cols(), 6+coman.iDyn3_model.getNrOfDOFs());
    std::cout<<"M @ q = zeros: ["<<M.toString()<<"]"<<std::endl;

    for(unsigned int i = 0; i < q_dot.size(); ++i)
        q_dot[i] = 0.01;
    coman.updateiDyn3Model(q, q_dot, true);
    yarp::sig::Vector tau_g_C = coman.iDyn3_model.getTorques();

    bool a = false;
    for(unsigned int i = 0; i < tau_g.size(); ++i){
        if(fabs(tau_g[i]-tau_g_C[i])>1e-9){ //at least one is different!
            a = true;
            break;}
    }
    EXPECT_TRUE(a);
    std::cout<<"tau_g @ q = zeros: \n ["<<tau_g.toString()<<"]"<<std::endl;
    std::cout<<"tau_g_C @ q = zeros, dq = 0.01: \n ["<<tau_g_C.toString()<<"]"<<std::endl;

    yarp::sig::Matrix M2(6+coman.iDyn3_model.getNrOfDOFs(), 6+coman.iDyn3_model.getNrOfDOFs());
    coman.iDyn3_model.getFloatingBaseMassMatrix(M2);
    for(unsigned int i = 0; i < M.rows(); ++i)
        for(unsigned int j = 0; j < M.cols(); ++j)
            EXPECT_DOUBLE_EQ(M(i,j), M2(i,j));
}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

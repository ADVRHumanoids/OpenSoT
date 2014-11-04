#include <drc_shared/tests_utils.h>
#include <drc_shared/cartesian_utils.h>
#include <drc_shared/idynutils.h>
#include <gtest/gtest.h>
#include <OpenSoT/tasks/velocity/CoM.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>

using namespace yarp::math;

namespace {

class testCoMTask: public ::testing::Test
{
protected:
    iDynUtils _robot;
    iDynUtils _fixed_robot;
    iDynUtils _normal_robot;

    testCoMTask()
    {

    }

    virtual ~testCoMTask() {

    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }

};

TEST_F(testCoMTask, testCoMTask_)
{
    // setting initial position with bent legs
    yarp::sig::Vector q_leg(6, 0.0),
                      q_whole(_robot.iDyn3_model.getNrOfDOFs(), 1E-4);
    q_leg[0] = -25.0*M_PI/180.0;
    q_leg[3] =  50.0*M_PI/180.0;
    q_leg[5] = -25.0*M_PI/180.0;

    _robot.fromRobotToIDyn(q_leg, q_whole, _robot.left_leg);
    _robot.fromRobotToIDyn(q_leg, q_whole, _robot.right_leg);

    _robot.iDyn3_model.setFloatingBaseLink(_robot.left_leg.index);
    _robot.updateiDyn3Model(q_whole, true);

    _fixed_robot.iDyn3_model.setFloatingBaseLink(_fixed_robot.left_leg.index);
    _fixed_robot.updateiDyn3Model(q_whole);

    _normal_robot.updateiDyn3Model(q_whole);

    std::cout << "_robot.getCoM() is: " << _robot.iDyn3_model.getCOM().toString() << std::endl;
    std::cout << "_robot.getCoM(_robot.left_leg.index) is: " << _robot.iDyn3_model.getCOM("",_robot.left_leg.end_effector_index).toString() << std::endl;

    std::cout << "_fixed_robot.getCoM() is: " << _fixed_robot.iDyn3_model.getCOM().toString() << std::endl;
    std::cout << "_fixed_robot.getCoM(_fixed_robot.left_leg.index) is: " << _fixed_robot.iDyn3_model.getCOM("",_fixed_robot.left_leg.end_effector_index).toString() << std::endl;

    std::cout << "computing _normal_robot with world in waist.." << std::endl;
    std::cout << "_normal_robot.getCoM() is: " << _normal_robot.iDyn3_model.getCOM().toString() << std::endl;
    std::cout << "_normal_robot.getCoM(_normal_robot.left_leg.index) is: " << _normal_robot.iDyn3_model.getCOM("",_normal_robot.left_leg.end_effector_index).toString() << std::endl;

    _normal_robot.updateiDyn3Model(q_whole, true);

    std::cout << "computing _normal_robot with proper world.." << std::endl;
    std::cout << "_normal_robot.getCoM() is: " << _normal_robot.iDyn3_model.getCOM().toString() << std::endl;
    std::cout << "_normal_robot.getCoM(_normal_robot.left_leg.index) is: " << _normal_robot.iDyn3_model.getCOM("",_normal_robot.left_leg.end_effector_index).toString() << std::endl;


    OpenSoT::tasks::velocity::CoM CoM(q_whole, _robot);

    EXPECT_TRUE(CoM.getb() == yarp::sig::Vector(3,0.0)) << "b = " << CoM.getb().toString();

    // setting x_ref with a delta offset along the z axis (+2cm)
    yarp::sig::Vector delta_x(3,0.0);
                      delta_x(2) = 0.02;
    yarp::sig::Vector x = _robot.iDyn3_model.getCOM();
    yarp::sig::Vector x_ref = x + delta_x;

    yarp::sig::Matrix J;
    // hack! we need to compute world position in a smarter way....
    _fixed_robot.updateiDyn3Model(q_whole,true);
    _fixed_robot.iDyn3_model.getCOMJacobian(J);
    J.removeCols(0,6);
    J.removeRows(3,3);
    EXPECT_TRUE(CoM.getA() == J);
    EXPECT_EQ(CoM.getA().rows(), 3);
    EXPECT_EQ(CoM.getb().size(), 3);

    EXPECT_TRUE(CoM.getWeight() == yarp::sig::Matrix(3,3).eye());

    EXPECT_TRUE(CoM.getConstraints().size() == 0);

    double K = 0.7;
    CoM.setLambda(K);
    EXPECT_DOUBLE_EQ(CoM.getLambda(), K);

    CoM.setReference(x);
    for(unsigned int i = 0; i < 3; ++i)
        EXPECT_NEAR(CoM.getb()[i],0,1E-12) << "b[i] = " << CoM.getb()[i];

    CoM.setReference(x_ref);
    yarp::sig::Vector positionError = x_ref - x;
    for(unsigned int i = 0; i < 3; ++i)
        EXPECT_NEAR(CoM.getb()[i],positionError[i],1E-12) << "b[i] = " << CoM.getb()[i];

    _robot.updateiDyn3Model(q_whole, true);
    for(unsigned int i = 0; i < 3; ++i)
        EXPECT_NEAR(CoM.getb()[i],positionError[i],1E-12) << "b[i] = " << CoM.getb()[i];

    yarp::sig::Vector x_now;
    for(unsigned int i = 0; i < 100; ++i)
    {
        _robot.updateiDyn3Model(q_whole, true);

        CoM.update(q_whole);

        q_whole += pinv(CoM.getA(),1E-6)*CoM.getLambda()*CoM.getb();

        _robot.updateiDyn3Model(q_whole, true);
        x_now = _robot.iDyn3_model.getCOM();
        std::cout << "Current error after iteration " << i << " is " << x_ref(2) - x_now(2) << std::endl;
    }


    EXPECT_LT( findMax((x_ref - x_now)), 1E-3 ) << "x_ref:" << x_ref.toString() << std::endl
                                                << "x_now:" << x_now.toString() << std::endl;
    EXPECT_LT( abs(findMin((x_ref - x_now))), 1E-3 );

    // checking for the position
    for(unsigned int i = 0; i < 3; ++i) {
        EXPECT_NEAR(x_ref(i),x_now(i),1E-4);
    }

}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

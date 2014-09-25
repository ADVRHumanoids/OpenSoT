#include <drc_shared/tests_utils.h>
#include <drc_shared/cartesian_utils.h>
#include <drc_shared/idynutils.h>
#include <gtest/gtest.h>
#include <wb_sot/tasks/velocity/CoM.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>

using namespace yarp::math;

namespace {

class testCoMTask: public ::testing::Test
{
protected:
    iDynUtils _robot;
    iDynUtils _fixed_robot;

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
                      q_whole(_robot.coman_iDyn3.getNrOfDOFs(), 1E-4);
    q_leg[0] = -25.0*M_PI/180.0;
    q_leg[3] =  50.0*M_PI/180.0;
    q_leg[5] = -25.0*M_PI/180.0;

    _robot.fromRobotToIDyn(q_leg, q_whole, _robot.left_leg);
    _robot.fromRobotToIDyn(q_leg, q_whole, _robot.right_leg);

    _robot.updateiDyn3Model(q_whole);

    _fixed_robot.updateiDyn3Model(q_whole);
    _fixed_robot.coman_iDyn3.setFloatingBaseLink(_fixed_robot.left_leg.end_effector_index);
    _fixed_robot.updateiDyn3Model(q_whole);

    wb_sot::tasks::velocity::CoM CoM(q_whole);

    EXPECT_TRUE(CoM.getb() == yarp::sig::Vector(3,0.0)) << "b = " << CoM.getb().toString();

    // setting x_ref with a delta offset along the z axis (+2cm)
    yarp::sig::Vector delta_x(3,0.0);
                      delta_x(2) = 0.02;
    yarp::sig::Vector x = _robot.coman_iDyn3.getCOM("",_robot.left_leg.end_effector_index);
    yarp::sig::Vector x_ref = x + delta_x;

    yarp::sig::Matrix J;
    _fixed_robot.coman_iDyn3.getCOMJacobian(J);
    J.removeCols(0,6);
    J.removeRows(3,3);
    EXPECT_TRUE(CoM.getA() == J);
    EXPECT_EQ(CoM.getA().rows(), 3);
    EXPECT_EQ(CoM.getb().size(), 3);

    EXPECT_TRUE(CoM.getWeight() == yarp::sig::Matrix(3,3).eye());

    EXPECT_TRUE(CoM.getConstraints().size() == 0);

    double K = 0.8;
    CoM.setAlpha(K);
    EXPECT_DOUBLE_EQ(CoM.getAlpha(), K);

    CoM.setReference(x);
    EXPECT_TRUE(CoM.getb() == yarp::sig::Vector(3,0.0)) << "b = " << CoM.getb().toString();

    CoM.setReference(x_ref);
    yarp::sig::Vector positionError = x_ref - x;
    EXPECT_TRUE(CoM.getb() == positionError) << "b = " << CoM.getb().toString();

    yarp::sig::Vector x_now;
    for(unsigned int i = 0; i < 25; ++i)
    {
        CoM.update(q_whole);

        q_whole += pinv(CoM.getA(),1E-6)*CoM.getAlpha()*CoM.getb();

        _robot.updateiDyn3Model(q_whole);
        x_now = _robot.coman_iDyn3.getCOM("",_robot.left_leg.end_effector_index);
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

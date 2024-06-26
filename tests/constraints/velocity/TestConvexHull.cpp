#include <gtest/gtest.h>
#include <OpenSoT/constraints/velocity/ConvexHull.h>
#include <OpenSoT/utils/convex_hull_utils.h>
#include <cmath>
#include <xbot2_interface/xbotinterface2.h>
#define  s                1.0
#define  dT               0.001* s
#define  m_s              1.0
#define  CoMVelocityLimit 0.03 * m_s
#define toRad(X) (X * M_PI/180.0)
#include "../../common.h"

using namespace OpenSoT::constraints::velocity;


namespace {

// The fixture for testing class ConvexHull.
class testConvexHull : public TestBase{
protected:


  // You can remove any or all of the following functions if its body
  // is empty.

  testConvexHull(): TestBase("coman_floating_base")
  {
      // You can do set-up work for each test here.

      _links_in_contact.push_back("l_foot_lower_left_link");
      _links_in_contact.push_back("l_foot_lower_right_link");
      _links_in_contact.push_back("l_foot_upper_left_link");
      _links_in_contact.push_back("l_foot_upper_right_link");
      _links_in_contact.push_back("r_foot_lower_left_link");
      _links_in_contact.push_back("r_foot_lower_right_link");
      _links_in_contact.push_back("r_foot_upper_left_link");
      _links_in_contact.push_back("r_foot_upper_right_link");

      velocityLimits.setZero(3); velocityLimits<<CoMVelocityLimit,CoMVelocityLimit,CoMVelocityLimit;
      _convexHull = new ConvexHull(*(_model_ptr.get()), _links_in_contact);
  }

  virtual ~testConvexHull() {
    // You can do clean-up work that doesn't throw exceptions here.
      if(_convexHull != NULL) {
        delete _convexHull;
        _convexHull = NULL;
      }
  }

  // If the constructor and destructor are not enough for setting up
  // and cleaning up each test, you can define the following methods:

  virtual void SetUp() {
    // Code here will be called immediately after the constructor (right
    // before each test).
      _convexHull->update();
      _model_ptr->setJointPosition(_model_ptr->getNeutralQ());
      _model_ptr->update();
  }

  virtual void TearDown() {
    // Code here will be called immediately after each test (right
    // before the destructor).
  }

  // Objects declared here can be used by all tests in the test case for ConvexHull.

  OpenSoT::constraints::velocity::ConvexHull* _convexHull;

  Eigen::VectorXd velocityLimits;
  Eigen::VectorXd q;

  std::list<std::string> _links_in_contact;
};

void updateModel(const Eigen::VectorXd& q, XBot::ModelInterface::Ptr model)
{
    model->setJointPosition(q);
    model->update();
}

//void getPointsFromConstraints(const yarp::sig::Matrix &A_ch,
//                              const yarp::sig::Vector& b_ch,
//                              std::vector<KDL::Vector>& points) {
//    unsigned int nRects = A_ch.rows();

//    for(unsigned int j = 0; j < nRects; ++j) {
//        unsigned int i = (j-1)%nRects;

//        // get coefficients for i-th rect
//        double a_i = A_ch(i,0);
//        double b_i = A_ch(i,1);
//        double c_i = -1.0*b_ch(i);

//        // get coefficients for rect nect to i-th
//        double a_j = A_ch(j,0);
//        double b_j = A_ch(j,1);
//        double c_j = -1.0*b_ch(j);

//        /** Kramer rule to find intersection between two rects by Valerio Varricchio */
//        double x = (-b_j*c_i+b_i*c_j)/(a_i*b_j-b_i*a_j);
//        double y = (-a_i*c_j+c_i*a_j)/(a_i*b_j-b_i*a_j);
//        points.push_back(KDL::Vector(x,y,0.0));
//    }
//}

// we need to check the old implementation with the new.
// notice how the two implementatios are equal only when boundScaling = 0.0
// In fact, the old implementation was bogus...
TEST_F(testConvexHull, checkImplementation) {
    // ------- Set The robot in a certain configuration ---------
    Eigen::VectorXd q = _model_ptr->getNeutralQ();
    q[_model_ptr->getQIndex("LHipSag")] = toRad(-23.5);
    q[_model_ptr->getQIndex("LHipLat")] = toRad(2.0);
    q[_model_ptr->getQIndex("LHipYaw")] = toRad(-4.0);
    q[_model_ptr->getQIndex("LKneeSag")] = toRad(50.1);
    q[_model_ptr->getQIndex("LAnkLat")] = toRad(-2.0);
    q[_model_ptr->getQIndex("LAnkSag")] = toRad(-26.6);

    q[_model_ptr->getQIndex("RHipSag")] = toRad(-23.5);
    q[_model_ptr->getQIndex("RHipLat")] = toRad(-2.0);
    q[_model_ptr->getQIndex("RHipYaw")] = toRad(0.0);
    q[_model_ptr->getQIndex("RKneeSag")] = toRad(50.1);
    q[_model_ptr->getQIndex("RAnkLat")] = toRad(2.0);
    q[_model_ptr->getQIndex("RAnkSag")] = toRad(-26.6);

    std::cout<<"---------------TEST-------------"<<std::endl;

    updateModel(q, _model_ptr);
    OpenSoT::constraints::velocity::ConvexHull localConvexHull(*(_model_ptr.get()), _links_in_contact, 0.00);
    localConvexHull.update();

    std::list<Eigen::Vector3d> points;
    std::vector<Eigen::Vector3d> ch;
    convex_hull huller;
    Eigen::MatrixXd A_JCoM;


    huller.getSupportPolygonPoints(points,_links_in_contact,*(_model_ptr.get()),"COM");
    huller.getConvexHull(points, ch);
    Eigen::MatrixXd A(_links_in_contact.size(),2);
    Eigen::VectorXd b(_links_in_contact.size());
    OpenSoT::constraints::velocity::ConvexHull::getConstraints(ch, A, b, 0.00);

    //EXPECT_EQ(ch.size(),A.rows());
    EXPECT_EQ(_links_in_contact.size(),A.rows());
    EXPECT_EQ(b.size(), A.rows());
    EXPECT_EQ(A.cols(), 2);

    Eigen::MatrixXd Aineq = localConvexHull.getAineq();
    Eigen::VectorXd bUpperBound = localConvexHull.getbUpperBound();


    // multiplying A by JCoM
    Eigen::MatrixXd _JCoM;
    _model_ptr->getCOMJacobian(_JCoM);
    Eigen::MatrixXd JCoM = _JCoM.block(0,0,2,_JCoM.cols());


    std::cout<<"A.cols(): "<<A.cols()<<std::endl;
    std::cout<<"JCoM.rows(): "<<JCoM.rows()<<std::endl;

    assert(A.cols() == JCoM.rows());
    std::cout<<"test JCoM: "<<JCoM<<std::endl;
    A_JCoM = A * JCoM;

    EXPECT_EQ(A_JCoM.rows(), Aineq.rows());
    EXPECT_EQ(A_JCoM.cols(), Aineq.cols());
    EXPECT_EQ(b.size(), bUpperBound.size());

    std::cout<<"Aineq rows: "<<Aineq.rows()<<std::endl;
    std::cout<<"Aineq cols: "<<Aineq.cols()<<std::endl;
    std::cout<<"A_JCoM rows: "<<A_JCoM.rows()<<std::endl;
    std::cout<<"A_JCoM cols: "<<A_JCoM.cols()<<std::endl;
    std::cout<<"Aineq: "<<Aineq<<std::endl; std::cout<<std::endl;
    std::cout<<"A_JCoM: "<<A_JCoM<<std::endl;
    for(unsigned int i = 0; i < A_JCoM.rows(); ++i)
    {
        for(unsigned j = 0; j < A_JCoM.cols(); ++j){
            std::cout<<"i: "<<i<<", j: "<<j<<std::endl;
            EXPECT_DOUBLE_EQ(A_JCoM(i,j), Aineq(i,j));}
    }

    for(unsigned int i = 0; i < b.size(); ++i)
        EXPECT_DOUBLE_EQ(b[i], bUpperBound[i]);

    std::cout<<"A: "<<A<<std::endl;
    std::cout<<"Aineq: "<<Aineq<<std::endl;
    std::cout<<"b: "<<b<<std::endl;
    std::cout<<"bUpperBound: "<<bUpperBound<<std::endl;
}

TEST_F(testConvexHull, checkBoundsScaling) {
    // ------- Set The robot in a certain configuration ---------

    std::list<Eigen::Vector3d> chPoints;
    convex_hull huller_tmp;
    huller_tmp.getSupportPolygonPoints(chPoints,_links_in_contact,*(_model_ptr.get()),"COM");

    convex_hull idyn_convex_hull;
    std::vector<Eigen::Vector3d> ch;
    idyn_convex_hull.getConvexHull(chPoints, ch);

    Eigen::MatrixXd A_ch(_links_in_contact.size(),2);
    Eigen::VectorXd b_ch(_links_in_contact.size());
    Eigen::MatrixXd A_ch_1cm_scaling(_links_in_contact.size(),2);
    Eigen::VectorXd b_ch_1cm_scaling(_links_in_contact.size());
    OpenSoT::constraints::velocity::ConvexHull::getConstraints(ch, A_ch, b_ch, 0.0);
    OpenSoT::constraints::velocity::ConvexHull::getConstraints(ch, A_ch_1cm_scaling, b_ch_1cm_scaling, 0.01);

    EXPECT_TRUE(A_ch == A_ch_1cm_scaling);

    for(unsigned int i = 0; i < ch.size(); ++i) {
        double norm_i = sqrt(A_ch(i,0)*A_ch(i,0) + A_ch(i,1)*A_ch(i,1));
        double distance_i = fabs(b_ch_1cm_scaling[i]-b_ch[i])/norm_i;
        EXPECT_NEAR(distance_i,.01,1e-16);
    }
}

TEST_F(testConvexHull, sizesAreCorrect) {

    std::list<Eigen::Vector3d> points;
    std::vector<Eigen::Vector3d> ch;
    convex_hull huller;
    huller.getSupportPolygonPoints(points,_links_in_contact,*(_model_ptr.get()),"COM");
    huller.getConvexHull(points, ch);


    EXPECT_EQ(0, _convexHull->getLowerBound().size()) << "lowerBound should have size 0"
                                                      << "but has size"
                                                      <<  _convexHull->getLowerBound().size();
    EXPECT_EQ(0, _convexHull->getUpperBound().size()) << "upperBound should have size 0"
                                                      << "but has size"
                                                      << _convexHull->getUpperBound().size();

    EXPECT_EQ(0, _convexHull->getAeq().rows()) << "Aeq should have size 0"
                                               << "but has size"
                                               << _convexHull->getAeq().rows();

    EXPECT_EQ(0, _convexHull->getbeq().size()) << "beq should have size 0"
                                               << "but has size"
                                               <<  _convexHull->getbeq().size();


    EXPECT_EQ(_model_ptr->getNv(),_convexHull->getAineq().cols()) <<  " Aineq should have number of columns equal to "
                                                                              << _model_ptr->getNv()
                                                                              << " but has has "
                                                                              << _convexHull->getAeq().cols()
                                                                              << " columns instead";

    EXPECT_EQ(_links_in_contact.size(),_convexHull->getbLowerBound().size()) << "beq should have size 3"
                                                      << "but has size"
                                                      << _convexHull->getbLowerBound().size();




    EXPECT_EQ(_links_in_contact.size(),_convexHull->getAineq().rows()) << "Aineq should have size "
                                                       << _links_in_contact.size()
                                                       << " but has size"
                                                       << _convexHull->getAineq().rows();


    EXPECT_EQ(_links_in_contact.size(),_convexHull->getbUpperBound().size()) << "beq should have size "
                                                             << _links_in_contact.size()
                                                             << " but has size"
                                                             << _convexHull->getbUpperBound().size();
}


TEST_F(testConvexHull, NoZeroRowsPreset) {
    Eigen::VectorXd q(_model_ptr->getNq());
    q<<0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, -0.431797,	 0.005336,	 0.000954,	 0.878479,	-0.000438,	-0.417689,	-0.435283,	-0.000493,	 0.000097,	 0.873527,	-0.000018,	-0.436310,	 0.000606,	-0.002125,	 0.000050,	 0.349666,	 0.174536,	 0.000010,	-1.396576,	-0.000000,	-0.000029,	-0.000000,	 0.349665,	-0.174895,	-0.000196,	-1.396547,	-0.000000,	-0.000026,	-0.000013;

    // TODO implement a test that checks, for this specific configuration,
    // that the solution for the convex null does not contain a row full of zeroes
    for(unsigned int i = 0; i < 10000; ++i)
    {
        if(i>1){
            q = _model_ptr->generateRandomQ();
            q.head(7)<< 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1;}

        _model_ptr->setJointPosition(q);
        _model_ptr->update();
        _convexHull->update();
        std::vector<Eigen::Vector3d> ch;
        _convexHull->getConvexHull(ch);
        Eigen::MatrixXd A_ch(_links_in_contact.size(),2);
        Eigen::VectorXd b_ch(_links_in_contact.size());
        _convexHull->getConstraints(ch,A_ch,b_ch,0.01);
        for(unsigned int i = 0; i < ch.size(); ++i)
            EXPECT_GT(A_ch.row(i).norm(),1E-5);
    }
}

// Tests that the Foo::getLowerBounds() are zero at the bounds
TEST_F(testConvexHull, BoundsAreCorrect) {

    // ------- Set The robot in a certain configuration ---------
    Eigen::VectorXd q = _model_ptr->getNeutralQ();
    q[_model_ptr->getQIndex("LHipSag")] = toRad(-23.5);
    q[_model_ptr->getQIndex("LHipLat")] = toRad(2.0);
    q[_model_ptr->getQIndex("LHipYaw")] = toRad(-4.0);
    q[_model_ptr->getQIndex("LKneeSag")] = toRad(50.1);
    q[_model_ptr->getQIndex("LAnkLat")] = toRad(-2.0);
    q[_model_ptr->getQIndex("LAnkSag")] = toRad(-26.6);

    q[_model_ptr->getQIndex("RHipSag")] = toRad(-23.5);
    q[_model_ptr->getQIndex("RHipLat")] = toRad(-2.0);
    q[_model_ptr->getQIndex("RHipYaw")] = toRad(0.0);
    q[_model_ptr->getQIndex("RKneeSag")] = toRad(50.1);
    q[_model_ptr->getQIndex("RAnkLat")] = toRad(2.0);
    q[_model_ptr->getQIndex("RAnkSag")] = toRad(-26.6);


    updateModel(q, _model_ptr);
    _convexHull->update();

    // Get Vector of CH's points from coman
    std::list<Eigen::Vector3d> points;
    convex_hull huller_tmp;
    huller_tmp.getSupportPolygonPoints(points,_links_in_contact,*(_model_ptr.get()),"COM");

    // Compute CH from previous points
    std::vector<Eigen::Vector3d> ch;
    convex_hull huller;
    huller.getConvexHull(points, ch);

    //Compute CH from internal
    std::vector<Eigen::Vector3d> ch2;
    _convexHull->getConvexHull(ch2);

    std::cout << "CH:"<<std::endl;
    for(unsigned int i = 0; i < ch.size(); ++i)
        std::cout << ch[i].x() << " " << ch[i].y() << std::endl;

    std::cout << "CH2:"<<std::endl;
    for(unsigned int i = 0; i < ch2.size(); ++i)
        std::cout << ch2[i].x() << " " << ch2[i].y() << std::endl;

    ASSERT_EQ(ch.size(), ch2.size());
    for(unsigned int i = 0; i < ch.size(); ++i){
        ASSERT_DOUBLE_EQ(ch[i].x(), ch2[i].x());
        ASSERT_DOUBLE_EQ(ch[i].y(), ch2[i].y());
    }

}

}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

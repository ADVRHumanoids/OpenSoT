#include <gtest/gtest.h>
#include <OpenSoT/constraints/velocity/ConvexHull.h>
#include <advr_humanoids_common_utils/idynutils.h>
#include <OpenSoT/utils/convex_hull_utils.h>
#include <idynutils/tests_utils.h>
#include <iCub/iDynTree/yarp_kdl.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>
#include <cmath>
#include<ModelInterfaceIDYNUTILS/ModelInterfaceIDYNUTILS.h>
#include <advr_humanoids_common_utils/conversion_utils_YARP.h>
#define  s                1.0
#define  dT               0.001* s
#define  m_s              1.0
#define  CoMVelocityLimit 0.03 * m_s
#define toRad(X) (X * M_PI/180.0)

using namespace OpenSoT::constraints::velocity;
using namespace yarp::math;

namespace {

// The fixture for testing class ConvexHull.
class testConvexHull : public ::testing::Test{
public:
    typedef idynutils2 iDynUtils;
    static void null_deleter(iDynUtils *) {}
 protected:

  // You can remove any or all of the following functions if its body
  // is empty.

  testConvexHull() :
      coman("coman",
            std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.urdf",
            std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.srdf")
  {
      std::string robotology_root = std::getenv("ROBOTOLOGY_ROOT");
      std::string relative_path = "/external/OpenSoT/tests/configs/coman/configs/config_coman.yaml";

      _path_to_cfg = robotology_root + relative_path;

      _model_ptr = std::dynamic_pointer_cast<XBot::ModelInterfaceIDYNUTILS>
              (XBot::ModelInterface::getModel(_path_to_cfg));
      _model_ptr->loadModel(boost::shared_ptr<iDynUtils>(&coman, &null_deleter));

      if(_model_ptr)
          std::cout<<"pointer address: "<<_model_ptr.get()<<std::endl;
      else
          std::cout<<"pointer is NULL "<<_model_ptr.get()<<std::endl;
    // You can do set-up work for each test here.

      _links_in_contact.push_back("l_foot_lower_left_link");
      _links_in_contact.push_back("l_foot_lower_right_link");
      _links_in_contact.push_back("l_foot_upper_left_link");
      _links_in_contact.push_back("l_foot_upper_right_link");
      _links_in_contact.push_back("r_foot_lower_left_link");
      _links_in_contact.push_back("r_foot_lower_right_link");
      _links_in_contact.push_back("r_foot_upper_left_link");
      _links_in_contact.push_back("r_foot_upper_right_link");

      velocityLimits.resize(3,CoMVelocityLimit);
      zeros.resize(coman.iDynTree_model.getNrOfDOFs(),0.0);
      _convexHull = new ConvexHull(  conversion_utils_YARP::toEigen(zeros), *(_model_ptr.get()),
                                     _links_in_contact);
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
      _convexHull->update(conversion_utils_YARP::toEigen(zeros));
      coman.updateiDynTreeModel(conversion_utils_YARP::toEigen(zeros),true);
  }

  virtual void TearDown() {
    // Code here will be called immediately after each test (right
    // before the destructor).
  }

  // Objects declared here can be used by all tests in the test case for ConvexHull.

  iDynUtils coman;
  OpenSoT::constraints::velocity::ConvexHull* _convexHull;

  yarp::sig::Vector velocityLimits;
  yarp::sig::Vector zeros;
  yarp::sig::Vector q;

  XBot::ModelInterfaceIDYNUTILS::Ptr _model_ptr;
  std::string _path_to_cfg;
  std::list<std::string> _links_in_contact;
};

void updateiDyn3Model(const bool set_world_pose, const yarp::sig::Vector& q, idynutils2& idynutils)
{
    static Eigen::VectorXd zeroes(q.size());
    zeroes.setZero(zeroes.size());

    idynutils.updateiDynTreeModel(conversion_utils_YARP::toEigen(q),zeroes,zeroes, set_world_pose);
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
    yarp::sig::Vector q(coman.iDynTree_model.getNrOfDOFs(), 0.0);
    q[coman.iDynTree_model.getDOFIndex("LHipSag")] = toRad(-23.5);
    q[coman.iDynTree_model.getDOFIndex("LHipLat")] = toRad(2.0);
    q[coman.iDynTree_model.getDOFIndex("LHipYaw")] = toRad(-4.0);
    q[coman.iDynTree_model.getDOFIndex("LKneeSag")] = toRad(50.1);
    q[coman.iDynTree_model.getDOFIndex("LAnkLat")] = toRad(-2.0);
    q[coman.iDynTree_model.getDOFIndex("LAnkSag")] = toRad(-26.6);

    q[coman.iDynTree_model.getDOFIndex("RHipSag")] = toRad(-23.5);
    q[coman.iDynTree_model.getDOFIndex("RHipLat")] = toRad(-2.0);
    q[coman.iDynTree_model.getDOFIndex("RHipYaw")] = toRad(0.0);
    q[coman.iDynTree_model.getDOFIndex("RKneeSag")] = toRad(50.1);
    q[coman.iDynTree_model.getDOFIndex("RAnkLat")] = toRad(2.0);
    q[coman.iDynTree_model.getDOFIndex("RAnkSag")] = toRad(-26.6);

    std::cout<<"---------------TEST-------------"<<std::endl;

    updateiDyn3Model(true, q, coman);
    OpenSoT::constraints::velocity::ConvexHull localConvexHull( conversion_utils_YARP::toEigen(q), *(_model_ptr.get()), _links_in_contact, 0.00);
    localConvexHull.update(conversion_utils_YARP::toEigen(q));

    std::list<KDL::Vector> points;
    std::vector<KDL::Vector> ch;
    convex_hull huller;
    Eigen::MatrixXd A_JCoM;
    Eigen::MatrixXd A;
    Eigen::VectorXd b;

    huller.getSupportPolygonPoints(points,_links_in_contact,*(_model_ptr.get()),"COM");
    huller.getConvexHull(points, ch);
    OpenSoT::constraints::velocity::ConvexHull::getConstraints(ch, A, b, 0.00);

    EXPECT_EQ(ch.size(),A.rows());
    EXPECT_EQ(b.size(), A.rows());
    EXPECT_EQ(A.cols(), 2);

    yarp::sig::Matrix Aineq = conversion_utils_YARP::toYARP(localConvexHull.getAineq());
    yarp::sig::Vector bUpperBound = conversion_utils_YARP::toYARP(localConvexHull.getbUpperBound());


    // multiplying A by JCoM
    Eigen::MatrixXd _JCoM;
    coman.getCOMJacobian(_JCoM);
    yarp::sig::Matrix JCoM = conversion_utils_YARP::toYARP(_JCoM);
    JCoM = JCoM.removeCols(0,6);    // remove floating base
    JCoM = JCoM.removeRows(2,4);    // remove orientation + z
    assert(A.cols() == JCoM.rows());
    std::cout<<"test JCoM: "<<JCoM.toString()<<std::endl;
    A_JCoM = A * conversion_utils_YARP::toEigen(JCoM);

    EXPECT_EQ(A_JCoM.rows(), Aineq.rows());
    EXPECT_EQ(A_JCoM.cols(), Aineq.cols());
    EXPECT_EQ(b.size(), bUpperBound.size());

    std::cout<<"Aineq rows: "<<Aineq.rows()<<std::endl;
    std::cout<<"Aineq cols: "<<Aineq.cols()<<std::endl;
    std::cout<<"A_JCoM rows: "<<A_JCoM.rows()<<std::endl;
    std::cout<<"A_JCoM cols: "<<A_JCoM.cols()<<std::endl;
    std::cout<<"Aineq: "<<Aineq.toString()<<std::endl; std::cout<<std::endl;
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
    std::cout<<"Aineq: "<<Aineq.toString()<<std::endl;
    std::cout<<"b: "<<b<<std::endl;
    std::cout<<"bUpperBound: "<<bUpperBound.toString()<<std::endl;
}

TEST_F(testConvexHull, checkBoundsScaling) {
    // ------- Set The robot in a certain configuration ---------

    std::list<KDL::Vector> chPoints;
    convex_hull huller_tmp;
    huller_tmp.getSupportPolygonPoints(chPoints,_links_in_contact,*(_model_ptr.get()),"COM");

    convex_hull idyn_convex_hull;
    std::vector<KDL::Vector> ch;
    idyn_convex_hull.getConvexHull(chPoints, ch);

    Eigen::MatrixXd A_ch;
    Eigen::VectorXd b_ch;
    Eigen::MatrixXd A_ch_1cm_scaling;
    Eigen::VectorXd b_ch_1cm_scaling;
    OpenSoT::constraints::velocity::ConvexHull::getConstraints(ch, A_ch, b_ch, 0.0);
    OpenSoT::constraints::velocity::ConvexHull::getConstraints(ch, A_ch_1cm_scaling, b_ch_1cm_scaling, 0.01);

    EXPECT_TRUE(A_ch == A_ch_1cm_scaling);

    for(unsigned int i = 0; i < b_ch.size(); ++i) {
        double norm_i = sqrt(A_ch(i,0)*A_ch(i,0) + A_ch(i,1)*A_ch(i,1));
        double distance_i = fabs(b_ch_1cm_scaling[i]-b_ch[i])/norm_i;
        EXPECT_NEAR(distance_i,.01,1e-16);
    }
}

TEST_F(testConvexHull, sizesAreCorrect) {

    std::list<KDL::Vector> points;
    std::vector<KDL::Vector> ch;
    convex_hull huller;
    huller.getSupportPolygonPoints(points,_links_in_contact,*(_model_ptr.get()),"COM");
    huller.getConvexHull(points, ch);

    unsigned int hullSize = ch.size();

    unsigned int x_size = coman.iDynTree_model.getNrOfDOFs();

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


    EXPECT_EQ(coman.iDynTree_model.getNrOfDOFs(),_convexHull->getAineq().cols()) <<  " Aineq should have number of columns equal to "
                                                                              << coman.iDynTree_model.getNrOfDOFs()
                                                                              << " but has has "
                                                                              << _convexHull->getAeq().cols()
                                                                              << " columns instead";

    EXPECT_EQ(0,_convexHull->getbLowerBound().size()) << "beq should have size 3"
                                                      << "but has size"
                                                      << _convexHull->getbLowerBound().size();




    EXPECT_EQ(hullSize,_convexHull->getAineq().rows()) << "Aineq should have size "
                                                       << hullSize
                                                       << " but has size"
                                                       << _convexHull->getAineq().rows();


    EXPECT_EQ(hullSize,_convexHull->getbUpperBound().size()) << "beq should have size "
                                                             << hullSize
                                                             << " but has size"
                                                             << _convexHull->getbUpperBound().size();
}

TEST_F(testConvexHull, NoZeroRowsPreset) {
    double qVector[29] = {-0.431797,	 0.005336,	 0.000954,	 0.878479,	-0.000438,	-0.417689,	-0.435283,	-0.000493,	 0.000097,	 0.873527,	-0.000018,	-0.436310,	 0.000606,	-0.002125,	 0.000050,	 0.349666,	 0.174536,	 0.000010,	-1.396576,	-0.000000,	-0.000029,	-0.000000,	 0.349665,	-0.174895,	-0.000196,	-1.396547,	-0.000000,	-0.000026,	-0.000013};
    yarp::sig::Vector q(29, qVector);
    // TODO implement a test that checks, for this specific configuration,
    // that the solution for the convex null does not contain a row full of zeroes
    for(unsigned int i = 0; i < 10000; ++i)
    {
        if(i>1)
            q = tests_utils::getRandomAngles(coman.iDynTree_model.getJointBoundMin(),
                                             coman.iDynTree_model.getJointBoundMax(),
                                             coman.iDynTree_model.getNrOfDOFs());
        coman.updateiDynTreeModel(conversion_utils_YARP::toEigen(q), true);
        _convexHull->update(conversion_utils_YARP::toEigen(q));
        std::vector<KDL::Vector> ch;
        _convexHull->getConvexHull(ch);
        Eigen::MatrixXd A_ch;
        Eigen::VectorXd b_ch;
        _convexHull->getConstraints(ch,A_ch,b_ch,0.01);
        for(unsigned int i = 0; i < A_ch.rows(); ++i)
            EXPECT_GT(norm(conversion_utils_YARP::toYARP(A_ch).getRow(i)),1E-5);
    }
}

// Tests that the Foo::getLowerBounds() are zero at the bounds
TEST_F(testConvexHull, BoundsAreCorrect) {

    // ------- Set The robot in a certain configuration ---------
    yarp::sig::Vector q(coman.iDynTree_model.getNrOfDOFs(), 0.0);
    q[coman.iDynTree_model.getDOFIndex("LHipSag")] = toRad(-23.5);
    q[coman.iDynTree_model.getDOFIndex("LHipLat")] = toRad(2.0);
    q[coman.iDynTree_model.getDOFIndex("LHipYaw")] = toRad(-4.0);
    q[coman.iDynTree_model.getDOFIndex("LKneeSag")] = toRad(50.1);
    q[coman.iDynTree_model.getDOFIndex("LAnkLat")] = toRad(-2.0);
    q[coman.iDynTree_model.getDOFIndex("LAnkSag")] = toRad(-26.6);

    q[coman.iDynTree_model.getDOFIndex("RHipSag")] = toRad(-23.5);
    q[coman.iDynTree_model.getDOFIndex("RHipLat")] = toRad(-2.0);
    q[coman.iDynTree_model.getDOFIndex("RHipYaw")] = toRad(0.0);
    q[coman.iDynTree_model.getDOFIndex("RKneeSag")] = toRad(50.1);
    q[coman.iDynTree_model.getDOFIndex("RAnkLat")] = toRad(2.0);
    q[coman.iDynTree_model.getDOFIndex("RAnkSag")] = toRad(-26.6);


    updateiDyn3Model(true, q, coman);
    _convexHull->update(conversion_utils_YARP::toEigen(q));

    // Get Vector of CH's points from coman
    std::list<KDL::Vector> points;
    convex_hull huller_tmp;
    huller_tmp.getSupportPolygonPoints(points,_links_in_contact,*(_model_ptr.get()),"COM");

    // Compute CH from previous points
    std::vector<KDL::Vector> ch;
    convex_hull huller;
    huller.getConvexHull(points, ch);

    //Compute CH from internal
    std::vector<KDL::Vector> ch2;
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


//    // Reconstruct CH from A and b
//    std::vector<KDL::Vector> chReconstructed;

//    yarp::sig::Matrix Aineq = convexHull->getAineq();
//    yarp::sig::Vector bUpperBound = convexHull->getbUpperBound();

//    std::cout<<"Aineq: "<<Aineq.toString()<<std::endl;
//    std::cout<<"bUpperBound: "<<bUpperBound.toString()<<std::endl;

//    getPointsFromConstraints(Aineq, bUpperBound, chReconstructed);

//    std::cout << "CH:"<<std::endl;
//    for(unsigned int i = 0; i < ch.size(); ++i)
//        std::cout << ch[i].x() << " " << ch[i].y() << std::endl;

//    std::cout << "CH_RECONSTRUCTED:"<<std::endl;
//    for(unsigned int i = 0; i < chReconstructed.size(); ++i)
//        std::cout << chReconstructed[i].x() << " " << chReconstructed[i].y() << std::endl;

//    ASSERT_EQ(ch.size(),chReconstructed.size());


//    for(unsigned int i = 0; i < ch.size(); ++i) {
//        EXPECT_DOUBLE_EQ(ch[i].x(), chReconstructed[i].x()) << "ch.x and chReconstructed.x"
//                                                            << " should be equal!" << std::endl;
//        EXPECT_DOUBLE_EQ(ch[i].y(), chReconstructed[i].y()) << "ch.y and chReconstructed.y"
//                                                            << " should be equal!" << std::endl;
//    }
}

}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

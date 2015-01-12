#include <gtest/gtest.h>
#include <OpenSoT/constraints/velocity/ConvexHull.h>
#include <idynutils/idynutils.h>
#include <idynutils/convex_hull.h>
#include <idynutils/tests_utils.h>
#include <iCub/iDynTree/yarp_kdl.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>
#include <cmath>
#define  s                1.0
#define  dT               0.001* s
#define  m_s              1.0
#define  CoMVelocityLimit 0.03 * m_s
#define toRad(X) (X * M_PI/180.0)

using namespace OpenSoT::constraints::velocity;
using namespace yarp::math;

namespace {

class old_convex_hull
{
public:
    double _ransac_distance_thr;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _pointCloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _projectedPointCloud;

    old_convex_hull():
        _ransac_distance_thr(0.001),
        _pointCloud(new pcl::PointCloud<pcl::PointXYZ>()),
        _projectedPointCloud(new pcl::PointCloud<pcl::PointXYZ>()){}
    ~old_convex_hull(){}

    void getConvexHull(const std::list<KDL::Vector>& points, yarp::sig::Matrix& A, yarp::sig::Vector& b)
    {
        fromSTDList2PCLPointCloud(points, _pointCloud);

            //Filtering
            projectPCL2Plane(_pointCloud, _ransac_distance_thr, _projectedPointCloud);


            pcl::PointCloud<pcl::PointXYZ> pointsInConvexHull;
            std::vector<pcl::Vertices> indicesOfVertexes;

            // hullVertices.vertices is the list of vertices...
            // by taking each point and the consequent in the list
            // (i.e. vertices[1]-vertices[0] it is possible to compute
            // bounding segments for the hull
            pcl::ConvexHull<pcl::PointXYZ> huller;
            huller.setInputCloud (_projectedPointCloud);
            huller.reconstruct(pointsInConvexHull, indicesOfVertexes);
            if(indicesOfVertexes.size() != 1) {
                std::cout<<"Error: more than one polygon found!"<<std::endl;
                return;
            } else
                pcl::Vertices hullVertices = indicesOfVertexes[0];

            //printIndexAndPointsInfo(pointsInConvexHull, indicesOfVertexes);

            getConstraints(pointsInConvexHull, indicesOfVertexes, A, b);

            _pointCloud->clear();
            _projectedPointCloud->clear();
    }

    //void setRansacDistanceThr(const double x){_ransac_distance_thr = x;}

/**
  * @brief getSupportPolygonPoints
  * @param points return a list of points express in a frame F
  *        oriented like the world frame and with origin
  *        on the CoM projection in the support polygon
  */
 static void getSupportPolygonPoints( iDynUtils &robot,
                                      std::list<KDL::Vector>& points)
 {
     std::vector<std::string> names;
     names.push_back("l_foot_lower_left_link");
     names.push_back("l_foot_lower_right_link");
     names.push_back("l_foot_upper_left_link");
     names.push_back("l_foot_upper_right_link");
     names.push_back("r_foot_lower_left_link");
     names.push_back("r_foot_lower_right_link");
     names.push_back("r_foot_upper_left_link");
     names.push_back("r_foot_upper_right_link");

     KDL::Frame waist_T_CoM;
     KDL::Frame waist_T_point;
     KDL::Frame CoM_T_point;
     for(unsigned int i = 0; i < names.size(); ++i)
     {
         // get points in world frame
         waist_T_point = robot.iDyn3_model.getPositionKDL(robot.iDyn3_model.getLinkIndex(names[i]));
         // get CoM in the world frame
         YarptoKDL(robot.iDyn3_model.getCOM(), waist_T_CoM.p);

         CoM_T_point = waist_T_CoM.Inverse() * waist_T_point;
         points.push_back(CoM_T_point.p);
     }
 }

    pcl::PointXYZ fromKDLVector2PCLPointXYZ(const KDL::Vector& point)
    {
        pcl::PointXYZ p;
        p.x = point.x();
        p.y = point.y();
        p.z = point.z();
        return p;
    }

    void fromSTDList2PCLPointCloud(const std::list<KDL::Vector>& points, pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud)
    {
        for(std::list<KDL::Vector>::const_iterator i = points.begin(); i != points.end(); ++i)
               point_cloud->push_back(fromKDLVector2PCLPointXYZ(*i));
    }

    void projectPCL2Plane(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, const double ransac_distance_thr,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr projected_point_cloud)
    {
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

            //We projects ALL the points in the plane (0 0 1)
            coefficients->values.clear();
            coefficients->values.resize(4, 0.0);
            coefficients->values[0] = 0.0;
            coefficients->values[1] = 0.0;
            coefficients->values[2] = 1.0;
            coefficients->values[3] = 0.0;

        //    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        //    // Create the segmentation object
        //    pcl::SACSegmentation<pcl::PointXYZ> seg;
        //    // Optional
        //    seg.setOptimizeCoefficients (true);
        //    // Mandatory
        //    seg.setModelType (pcl::SACMODEL_PLANE);
        //    seg.setMethodType (pcl::SAC_RANSAC);
        //    seg.setDistanceThreshold (ransac_distance_thr);
        //    seg.setInputCloud (cloud);
        //    seg.segment (*inliers, *coefficients);


            pcl::ProjectInliers<pcl::PointXYZ> proj;
            proj.setModelType (pcl::SACMODEL_PLANE);
            proj.setInputCloud (cloud);
            proj.setModelCoefficients (coefficients);
            proj.filter (*projected_point_cloud);
    }

    void printIndexAndPointsInfo(const pcl::PointCloud<pcl::PointXYZ>& pointsInConvexHull, const std::vector<pcl::Vertices>& indicesOfVertexes)
    {
        std::cout<<"Indices of vertex has size "<< indicesOfVertexes.size()<<std::endl;
//        for(unsigned int i = 0; i < indicesOfVertexes.size(); ++i){
//            pcl::Vertices vertices = indicesOfVertexes[i];
//            for(unsigned int ii = 0; ii < vertices.vertices.size(); ++ii)
//                ROS_INFO("vertex %i (%f, %f, %f) has index %i ", ii,
//                         pointsInConvexHull.at(vertices.vertices[ii]).x, pointsInConvexHull.at(vertices.vertices[ii]).y, pointsInConvexHull.at(vertices.vertices[ii]).z,
//                         vertices.vertices[ii]);
//        }
    }

    void getLineCoefficients(const pcl::PointXYZ& p0, const pcl::PointXYZ& p1, double &a, double& b, double &c)
    {
        double x1 = p0.x;
        double x2 = p1.x;
        double y1 = p0.y;
        double y2 = p1.y;

        a = y1 - y2;
        b = x2 - x1;
        c = -b*y1 -a*x1;
    }

    void getConstraints(const pcl::PointCloud<pcl::PointXYZ>& pointsInConvexHull, const std::vector<pcl::Vertices>& indicesOfVertexes,
                        yarp::sig::Matrix& A, yarp::sig::Vector& b)
    {
        double _a, _b, _c;
            A.resize(pointsInConvexHull.size() ,2);
            b.resize(pointsInConvexHull.size());

            unsigned int z = 0;
            for(unsigned int i = 0; i < indicesOfVertexes.size(); ++i)
            {
                const pcl::Vertices& vs = indicesOfVertexes[i];
                for(unsigned int j = 0; j < vs.vertices.size(); ++j)
                {
                    unsigned int k = (j + 1)%vs.vertices.size();
                    getLineCoefficients(pointsInConvexHull[vs.vertices[j]], pointsInConvexHull[vs.vertices[k]], _a, _b, _c);
                    if(_c <= 0.0) { // see Moleskine
                        A(z,0) = + _a;
                        A(z,1) = + _b;
                        b[z] =   - _c;
                    } else {
                        A(z,0) = - _a;
                        A(z,1) = - _b;
                        b[z] =   + _c;
                    }
                    double ch_boundary = 0;
                    if(fabs(_c) <= ch_boundary)
                        b[z] = 0.0;
                    else
                        b[z] -= ch_boundary;
                    z++;
                }
            }
    }
};

// The fixture for testing class ConvexHull.
class testConvexHull : public ::testing::Test{
 protected:

  // You can remove any or all of the following functions if its body
  // is empty.

  testConvexHull()
  {
    // You can do set-up work for each test here.

      velocityLimits.resize(3,CoMVelocityLimit);
      zeros.resize(coman.iDyn3_model.getNrOfDOFs(),0.0);
      coman.iDyn3_model.setFloatingBaseLink(coman.left_leg.index);
      _convexHull = new ConvexHull(  zeros, coman );
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
      _convexHull->update(zeros);
      coman.updateiDyn3Model(zeros,true);
  }

  virtual void TearDown() {
    // Code here will be called immediately after each test (right
    // before the destructor).
  }

  // Objects declared here can be used by all tests in the test case for ConvexHull.

  iDynUtils coman;
  ConvexHull* _convexHull;

  yarp::sig::Vector velocityLimits;
  yarp::sig::Vector zeros;
  yarp::sig::Vector q;
};

void updateiDyn3Model(const bool set_world_pose, const yarp::sig::Vector& q, iDynUtils& idynutils)
{
    static yarp::sig::Vector zeroes(q.size(),0.0);

    idynutils.updateiDyn3Model(q,zeroes,zeroes, set_world_pose);
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
TEST_F(testConvexHull, comparisonWithOldImplementation) {
    // ------- Set The robot in a certain configuration ---------
    yarp::sig::Vector q(coman.iDyn3_model.getNrOfDOFs(), 0.0);
    q[coman.left_leg.joint_numbers[0]] = toRad(-23.5);
    q[coman.left_leg.joint_numbers[1]] = toRad(2.0);
    q[coman.left_leg.joint_numbers[2]] = toRad(-4.0);
    q[coman.left_leg.joint_numbers[3]] = toRad(50.1);
    q[coman.left_leg.joint_numbers[4]] = toRad(-2.0);
    q[coman.left_leg.joint_numbers[5]] = toRad(-26.6);

    q[coman.right_leg.joint_numbers[0]] = toRad(-23.5);
    q[coman.right_leg.joint_numbers[1]] = toRad(-2.0);
    q[coman.right_leg.joint_numbers[2]] = toRad(0.0);
    q[coman.right_leg.joint_numbers[3]] = toRad(50.1);
    q[coman.right_leg.joint_numbers[4]] = toRad(2.0);
    q[coman.right_leg.joint_numbers[5]] = toRad(-26.6);

    updateiDyn3Model(true, q, coman);
    ConvexHull localConvexHull( q, coman, 0.00);
    localConvexHull.update(q);

    old_convex_hull oldConvexHull;

    std::list<KDL::Vector> points;
    old_convex_hull::getSupportPolygonPoints(coman, points);
    yarp::sig::Matrix A;
    yarp::sig::Matrix A_JCoM;
    yarp::sig::Vector b;
    oldConvexHull.getConvexHull(points, A, b);

    yarp::sig::Matrix Aineq = localConvexHull.getAineq();
    yarp::sig::Vector bUpperBound = localConvexHull.getbUpperBound();


    // multiplying A by JCoM
    yarp::sig::Matrix JCoM;
    coman.iDyn3_model.getCOMJacobian(JCoM);
    JCoM = JCoM.removeCols(0,6);    // remove floating base
    JCoM = JCoM.removeRows(2,4);    // remove orientation + z
    assert(A.cols() == JCoM.rows());
    A_JCoM = A * JCoM;

    EXPECT_EQ(A_JCoM.rows(), Aineq.rows());
    EXPECT_EQ(A_JCoM.cols(), Aineq.cols());
    EXPECT_EQ(b.size(), bUpperBound.size());

    for(unsigned int i = 0; i < A_JCoM.rows(); ++i)
    {
        for(unsigned j = 0; j < A_JCoM.cols(); ++j)
            EXPECT_DOUBLE_EQ(A_JCoM(i,j), Aineq(i,j));
    }

    for(unsigned int i = 0; i < b.size(); ++i)
        EXPECT_DOUBLE_EQ(b[i], bUpperBound[i]);

    std::cout<<"A: "<<A.toString()<<std::endl;
    std::cout<<"Aineq: "<<Aineq.toString()<<std::endl;
    std::cout<<"b: "<<b.toString()<<std::endl;
    std::cout<<"bUpperBound: "<<bUpperBound.toString()<<std::endl;

    std::list<KDL::Vector> points2;
    coman.getSupportPolygonPoints(points2);

    idynutils::convex_hull idyn_convex_hull;
    std::vector<KDL::Vector> ch;
    idyn_convex_hull.getConvexHull(points2, ch);

    yarp::sig::Matrix A_ch;
    yarp::sig::Vector b_ch;
    ConvexHull::getConstraints(ch, A_ch, b_ch, 0.0);

    EXPECT_EQ(A.rows(), A_ch.rows());
    EXPECT_EQ(A.cols(), A_ch.cols());
    EXPECT_EQ(b.size(), b_ch.size());
    for(unsigned int i = 0; i < A.rows(); ++i)
    {
        for(unsigned j = 0; j < A.cols(); ++j)
            EXPECT_DOUBLE_EQ(A(i,j), A_ch(i,j));
    }

    for(unsigned int i = 0; i < b.size(); ++i)
        EXPECT_DOUBLE_EQ(b[i], b_ch[i]);
}

TEST_F(testConvexHull, checkBoundsScaling) {
    // ------- Set The robot in a certain configuration ---------

    std::list<KDL::Vector> chPoints;
    coman.getSupportPolygonPoints(chPoints);

    idynutils::convex_hull idyn_convex_hull;
    std::vector<KDL::Vector> ch;
    idyn_convex_hull.getConvexHull(chPoints, ch);

    yarp::sig::Matrix A_ch;
    yarp::sig::Vector b_ch;
    yarp::sig::Matrix A_ch_1cm_scaling;
    yarp::sig::Vector b_ch_1cm_scaling;
    ConvexHull::getConstraints(ch, A_ch, b_ch, 0.0);
    ConvexHull::getConstraints(ch, A_ch_1cm_scaling, b_ch_1cm_scaling, 0.01);

    EXPECT_TRUE(A_ch == A_ch_1cm_scaling);

    for(unsigned int i = 0; i < b_ch.size(); ++i) {
        double norm_i = sqrt(A_ch(i,0)*A_ch(i,0) + A_ch(i,1)*A_ch(i,1));
        double distance_i = fabs(b_ch_1cm_scaling[i]-b_ch[i])/norm_i;
        EXPECT_DOUBLE_EQ(distance_i,.01);
    }
}

TEST_F(testConvexHull, sizesAreCorrect) {

    std::list<KDL::Vector> points;
    std::vector<KDL::Vector> ch;
    idynutils::convex_hull huller;
    coman.getSupportPolygonPoints(points);
    huller.getConvexHull(points, ch);

    unsigned int hullSize = ch.size();

    unsigned int x_size = coman.iDyn3_model.getNrOfDOFs();

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


    EXPECT_EQ(coman.iDyn3_model.getNrOfDOFs(),_convexHull->getAineq().cols()) <<  " Aineq should have number of columns equal to "
                                                                              << coman.iDyn3_model.getNrOfDOFs()
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
            q = tests_utils::getRandomAngles(coman.iDyn3_model.getJointBoundMin(),
                                             coman.iDyn3_model.getJointBoundMax(),
                                             coman.iDyn3_model.getNrOfDOFs());
        coman.updateiDyn3Model(q, true);
        _convexHull->update(q);
        std::vector<KDL::Vector> ch;
        _convexHull->getConvexHull(ch);
        yarp::sig::Matrix A_ch;
        yarp::sig::Vector b_ch;
        _convexHull->getConstraints(ch,A_ch,b_ch,0.01);
        for(unsigned int i = 0; i < A_ch.rows(); ++i)
            EXPECT_GT(norm(A_ch.getRow(i)),1E-5);
    }
}

// Tests that the Foo::getLowerBounds() are zero at the bounds
TEST_F(testConvexHull, BoundsAreCorrect) {

    // ------- Set The robot in a certain configuration ---------
    yarp::sig::Vector q(coman.iDyn3_model.getNrOfDOFs(), 0.0);
    q[coman.left_leg.joint_numbers[0]] = toRad(-23.5);
    q[coman.left_leg.joint_numbers[1]] = toRad(2.0);
    q[coman.left_leg.joint_numbers[2]] = toRad(-4.0);
    q[coman.left_leg.joint_numbers[3]] = toRad(50.1);
    q[coman.left_leg.joint_numbers[4]] = toRad(-2.0);
    q[coman.left_leg.joint_numbers[5]] = toRad(-26.6);

    q[coman.right_leg.joint_numbers[0]] = toRad(-23.5);
    q[coman.right_leg.joint_numbers[1]] = toRad(-2.0);
    q[coman.right_leg.joint_numbers[2]] = toRad(0.0);
    q[coman.right_leg.joint_numbers[3]] = toRad(50.1);
    q[coman.right_leg.joint_numbers[4]] = toRad(2.0);
    q[coman.right_leg.joint_numbers[5]] = toRad(-26.6);

    updateiDyn3Model(true, q, coman);
    _convexHull->update(q);

    // Get Vector of CH's points from coman
    std::list<KDL::Vector> points;
    coman.getSupportPolygonPoints(points);

    // Compute CH from previous points
    std::vector<KDL::Vector> ch;
    idynutils::convex_hull huller;
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

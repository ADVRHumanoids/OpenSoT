#ifndef _CONVEX_HULL_H_
#define _CONVEX_HULL_H_

#include <moveit/robot_model/robot_model.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <kdl/frames.hpp>
#include <drc_shared/cartesian_utils.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>
#include <yarp/sig/Vector.h>

namespace wb_sot
{

class convex_hull
{
public:
    convex_hull();
    ~convex_hull();

    void getConvexHull(const std::list<KDL::Vector>& points, yarp::sig::Matrix& A, yarp::sig::Vector& b);
    //void setRansacDistanceThr(const double x){_ransac_distance_thr = x;}
private:
    double _ransac_distance_thr;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _pointCloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _projectedPointCloud;

    pcl::PointXYZ fromKDLVector2PCLPointXYZ(const KDL::Vector& point);
    void fromSTDList2PCLPointCloud(const std::list<KDL::Vector>& points, pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud);
    void projectPCL2Plane(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, const double ransac_distance_thr,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr projected_point_cloud);
    void printIndexAndPointsInfo(const pcl::PointCloud<pcl::PointXYZ>& pointsInConvexHull, const std::vector<pcl::Vertices>& indicesOfVertexes);
    void getLineCoefficients(const pcl::PointXYZ& p0, const pcl::PointXYZ& p1, double &a, double& b, double &c);
    void getConstraints(const pcl::PointCloud<pcl::PointXYZ>& pointsInConvexHull, const std::vector<pcl::Vertices>& indicesOfVertexes,
                        yarp::sig::Matrix& A, yarp::sig::Vector& b);
};

}

#endif

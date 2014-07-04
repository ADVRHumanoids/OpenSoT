#include "convex_hull.h"
#include <pcl/surface/convex_hull.h>

using namespace wb_sot;

convex_hull::convex_hull()
{

}

convex_hull::~convex_hull()
{

}

void convex_hull::getConvexHull(const std::list<KDL::Vector>& points) {
    pcl::PointCloud<pcl::PointXY> pointCloud = this->fromSTDList2PCLPointCloud(points);
    pcl::PointCloud<pcl::PointXY> pointsInConvexHull;
    std::vector<pcl::Vertices> indicesOfVertexes;

    // estimate the convex hull in camera frame
    pcl::ConvexHull<PointXY> huller;
    huller.setInputCloud (pointCloud);
    huller.reconstruct (pointsInConvexHull,
                        indicesOfVertexes);
    if(indicesOfVertexes.size() != 1) {
        std::cerr << "Error: more than one polygon found!" << std::endl;
        return
    } else
        pcl::Vertices hullVertices = indicesOfVertexes[0];


    // hullVertices.vertices is the list of vertices...
    // by taking each point and the consequent in the list
    // (i.e. vertices[1]-vertices[0] it is possible to compute
    // bounding segments for the hull

}

pcl::PointXY convex_hull::fromKDLVector2PCLPointXY(const KDL::Vector &point)
{
    pcl::PointXY p;
    p.x = point.x();
    p.y = point.y();
    return p;
}

pcl::PointCloud<pcl::PointXY> convex_hull::fromSTDList2PCLPointCloud(
        const std::list<KDL::Vector> &points)
{
    pcl::PointCloud<pcl::PointXY> pointCloud;
    for(std::list<KDL::Vector>::iterator i = points.begin();
        i != points.end();
        ++i)
        pointCloud.push_back(this->fromKDLVector2PCLPointXY(*i));
    return pointCloud;
}

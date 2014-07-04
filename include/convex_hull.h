#ifndef _CONVEX_HULL_H_
#define _CONVEX_HULL_H_

#include <moveit/robot_model/robot_model.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <kdl/frames.hpp>
#include <drc_shared/cartesian_utils.h>

namespace wb_sot
{

class convex_hull
{
public:
    convex_hull(){}
    ~convex_hull();

    void getConvexHull(const std::list<KDL::Vector>& points);
private:
    pcl::PointXY fromKDLVector2PCLPointXY(const KDL::Vector& point);
    pcl::PointCloud<pcl::PointXY> fromSTDList2PCLPointCloud(const std::list<KDL::Vector>& points);
};

}

#endif

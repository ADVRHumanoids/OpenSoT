/*
 * Copyright (C) 2014 Walkman
 * Author: Alessio Rocchi, Enrico Mingo
 * email:  alessio.rocchi@iit.it, enrico.mingo@iit.it
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/

#ifndef _CONVEX_HULL_H__
#define _CONVEX_HULL_H__

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <kdl/frames.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/concave_hull.h>
#include <XBotInterface/ModelInterface.h>

class convex_hull
{
public:
    convex_hull();
    ~convex_hull();

    /**
     * @brief getSupportPolygonPoints given a vector of reference frames that we consider in contact wit the ground,
     * it return a list of points express in the specified frame referenceFrame.
     * By default it is the frame COM, oriented like the world frame and with origin on the CoM
     * Notice the polygon points are not projected on the support surface.
     * @param points a list of points in the same reference frame
     * @param referenceFrame the string defining the reference frame in which to express the
     * support polygon points. The possibilities are:
     * - "COM"
     * - "world"
     * - {linkName}
     * @return false if the vector of reference frames is empty. True otherwise.
     */
    bool getSupportPolygonPoints(std::list<KDL::Vector>& points,
                                 const std::list<std::string> links_in_contact,
                                 const XBot::ModelInterface& model,
                                 const std::string referenceFrame = "COM");

    /**
     * @brief getConvexHull returns a minimum representation of the convex hull
     * @param points a list of points representing the convex hull
     * @param ch a list of points which are the vertices of the convex hull
     * @return true on success
     */
    bool getConvexHull(const std::list<KDL::Vector>& points,
                             std::vector<KDL::Vector>& ch);
    //void setRansacDistanceThr(const double x){_ransac_distance_thr = x;}

private:
    double _ransac_distance_thr;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _pointCloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _projectedPointCloud;

    KDL::Frame world_T_CoM;
    KDL::Frame world_T_point;
    KDL::Frame referenceFrame_T_point;
    KDL::Frame CoM_T_point;
    pcl::PointXYZ _tmp_pcl;
    pcl::PointCloud<pcl::PointXYZ> pointsInConvexHull;
    std::vector<pcl::Vertices> indicesOfVertexes;
    pcl::ConvexHull<pcl::PointXYZ> huller;
    KDL::Vector _tmp_vector;
    pcl::ProjectInliers<pcl::PointXYZ> proj;

    /**
     * @brief projectPCL2Plane projects a point cloud on a plane
     * @param cloud the input cloud
     * @param ransac_distance_thr threshold for the ransac algorithm to detect outliers
     * @param projected_point_cloud the output (projected) point cloud
     */
    void projectPCL2Plane(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                          const double ransac_distance_thr,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr projected_point_cloud);


    void printIndexAndPointsInfo(const pcl::PointCloud<pcl::PointXYZ>& pointsInConvexHull,
                                 const std::vector<pcl::Vertices>& indicesOfVertexes);
};

#endif

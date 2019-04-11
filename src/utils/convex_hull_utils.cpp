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

#include <OpenSoT/utils/convex_hull_utils.h>
#include <pcl/surface/convex_hull.h>

convex_hull::convex_hull():
    _ransac_distance_thr(0.001),
    _pointCloud(new pcl::PointCloud<pcl::PointXYZ>()),
    _projectedPointCloud(new pcl::PointCloud<pcl::PointXYZ>())
{

}

convex_hull::~convex_hull()
{

}

bool convex_hull::getConvexHull(const std::list<KDL::Vector>& points,
                                      std::vector<KDL::Vector>& convex_hull)
{
    _pointCloud->clear();
    for(auto point = points.begin(); point != points.end(); point++)
    {
        _tmp_pcl.x = point->x();
        _tmp_pcl.y = point->y();
        _tmp_pcl.z = point->z();
        _pointCloud->push_back(_tmp_pcl);
    }

    //Filtering
    projectPCL2Plane(_pointCloud, _ransac_distance_thr, _projectedPointCloud);


    pointsInConvexHull.clear();
    indicesOfVertexes.clear();

    // hullVertices.vertices is the list of vertices...
    // by taking each point and the consequent in the list
    // (i.e. vertices[1]-vertices[0] it is possible to compute
    // bounding segments for the hull
    huller.setInputCloud (_projectedPointCloud);
    huller.reconstruct(pointsInConvexHull, indicesOfVertexes);
    if(indicesOfVertexes.size() != 1)
        XBot::Logger::error("Error: more than one polygon found! \n");

    //printIndexAndPointsInfo(pointsInConvexHull, indicesOfVertexes);

    const pcl::Vertices& vs = indicesOfVertexes[0];
    convex_hull.clear();
    for(unsigned int j = 0; j < vs.vertices.size(); ++j)
    {
        _tmp_pcl = pointsInConvexHull.at(vs.vertices[j]);
        _tmp_vector.x(_tmp_pcl.x);
        _tmp_vector.y(_tmp_pcl.y);
        _tmp_vector.z(_tmp_pcl.z);
        convex_hull.push_back(_tmp_vector);
    }

    _pointCloud->clear();
    _projectedPointCloud->clear();

    return true;
}


void convex_hull::projectPCL2Plane(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, const double ransac_distance_thr,
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


    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (cloud);
    proj.setModelCoefficients (coefficients);
    proj.filter (*projected_point_cloud);
}

void convex_hull::printIndexAndPointsInfo(const pcl::PointCloud<pcl::PointXYZ>& pointsInConvexHull, const std::vector<pcl::Vertices>& indicesOfVertexes)
{
    XBot::Logger::info("Indices of vertex has size %i\n", indicesOfVertexes.size());
    for(unsigned int i = 0; i < indicesOfVertexes.size(); ++i){
        pcl::Vertices vertices = indicesOfVertexes[i];
        for(unsigned int ii = 0; ii < vertices.vertices.size(); ++ii)
            XBot::Logger::info("vertex %i [%f, %f, %f] has index %i \n",
                               ii,
                               pointsInConvexHull.at(vertices.vertices[ii]).x,
                               pointsInConvexHull.at(vertices.vertices[ii]).y,
                               pointsInConvexHull.at(vertices.vertices[ii]).z,
                               vertices.vertices[ii]);
    }
}

bool convex_hull::getSupportPolygonPoints(std::list<KDL::Vector>& points,
                                          const std::list<std::string> links_in_contact,
                                          const XBot::ModelInterface& model,
                                          const std::string referenceFrame)
{
    if(referenceFrame != "COM" && referenceFrame != "world" && model.getLinkID(referenceFrame) < 0)
        XBot::Logger::error("ERROR: trying to get support polygon points in unknown reference frame %s \n", referenceFrame.c_str());

    if(links_in_contact.empty() || (referenceFrame != "COM" && referenceFrame != "world" && model.getLinkID(referenceFrame) < 0))
        return false;

    for(std::list<std::string>::const_iterator it = links_in_contact.begin(); it != links_in_contact.end(); it++)
    {
        if(referenceFrame == "COM" || referenceFrame == "world")
            // get points in world frame
            model.getPose(*it, world_T_point);
        else
            model.getPose(*it, referenceFrame, referenceFrame_T_point);

        if(referenceFrame == "COM")
        {
            // get CoM in the world frame
            model.getCOM(world_T_CoM.p);

            CoM_T_point = world_T_CoM.Inverse() * world_T_point;
            points.push_back(CoM_T_point.p);
        } else if(referenceFrame == "world")
            points.push_back(world_T_point.p);
        else
            points.push_back(referenceFrame_T_point.p);
    }
    return true;
}

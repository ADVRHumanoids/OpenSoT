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
    }
    pcl::Vertices hullVertices = indicesOfVertexes[0];

    //printIndexAndPointsInfo(pointsInConvexHull, indicesOfVertexes);

    const pcl::Vertices& vs = indicesOfVertexes[0];
    for(unsigned int j = 0; j < vs.vertices.size(); ++j)
    {
        pcl::PointXYZ pointXYZ = pointsInConvexHull.at(vs.vertices[j]);
        convex_hull.push_back(fromPCLPointXYZ2KDLVector(pointXYZ));
    }

    _pointCloud->clear();
    _projectedPointCloud->clear();

    return true;
}

pcl::PointXYZ convex_hull::fromKDLVector2PCLPointXYZ(const KDL::Vector &point)
{
    pcl::PointXYZ p;
    p.x = point.x();
    p.y = point.y();
    p.z = point.z();
    return p;
}

KDL::Vector convex_hull::fromPCLPointXYZ2KDLVector(const pcl::PointXYZ &point)
{
    return KDL::Vector(point.x,point.y,point.z);
}

void convex_hull::fromSTDList2PCLPointCloud(const std::list<KDL::Vector> &points, pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud)
{
    for(std::list<KDL::Vector>::const_iterator i = points.begin(); i != points.end(); ++i)
        point_cloud->push_back(fromKDLVector2PCLPointXYZ(*i));
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


    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (cloud);
    proj.setModelCoefficients (coefficients);
    proj.filter (*projected_point_cloud);
}

void convex_hull::printIndexAndPointsInfo(const pcl::PointCloud<pcl::PointXYZ>& pointsInConvexHull, const std::vector<pcl::Vertices>& indicesOfVertexes)
{
    std::cout<<"Indices of vertex has size "<<indicesOfVertexes.size()<<std::endl;
    for(unsigned int i = 0; i < indicesOfVertexes.size(); ++i){
        pcl::Vertices vertices = indicesOfVertexes[i];
        for(unsigned int ii = 0; ii < vertices.vertices.size(); ++ii)
            std::cout<<"vertex" <<ii<<" ("<<pointsInConvexHull.at(vertices.vertices[ii]).x<<", "<<
                       pointsInConvexHull.at(vertices.vertices[ii]).y<<", "<<
                       pointsInConvexHull.at(vertices.vertices[ii]).z<<") has index "<<vertices.vertices[ii]<<std::endl;
    }
}

bool convex_hull::getSupportPolygonPoints(std::list<KDL::Vector>& points,
                                          const std::list<std::string> links_in_contact,
                                          const XBot::ModelInterface& model,
                                          const std::string referenceFrame)
{
    if(referenceFrame != "COM" &&
       referenceFrame != "world" &&
       model.getLinkID(referenceFrame) < 0)
        std::cerr << "ERROR: "
                  << "trying to get support polygon points in "
                  << "unknown reference frame "
                  << referenceFrame << std::endl;

    if(links_in_contact.empty() ||
       (referenceFrame != "COM" &&
        referenceFrame != "world" &&
        model.getLinkID(referenceFrame) < 0))
        return false;

    KDL::Frame world_T_CoM;
    KDL::Frame world_T_point;
    KDL::Frame referenceFrame_T_point;
    KDL::Frame CoM_T_point;
    for(std::list<std::string>::const_iterator it = links_in_contact.begin(); it != links_in_contact.end(); it++)
    {
        if(referenceFrame == "COM" ||
           referenceFrame == "world")
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

/*
 * Copyright: (C) 2014 Walkman Consortium
 * Authors: Enrico Mingo
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/

#include <drc_shared/utils/convex_hull.h>
#include <ros/ros.h>

int main()
{
    drc_shared::convex_hull ch;

    KDL::Vector p4(1.0, 0.0, -0.0006);
    ROS_WARN("Point 0 is (%f, %f %f)", p4.x(), p4.y(), p4.z());
    KDL::Vector p1(0.0, 0.0, 0.0);
    ROS_WARN("Point 1 is (%f, %f %f)", p1.x(), p1.y(), p1.z());
    KDL::Vector p2(0.0, 1.0, 0.0004);
    ROS_WARN("Point 2 is (%f, %f %f)", p2.x(), p2.y(), p2.z());
    KDL::Vector p3(1.0, 1.0, 0.0);
    ROS_WARN("Point 3 is (%f, %f %f)", p3.x(), p3.y(), p3.z());

    KDL::Vector p5(1.0, 0.0, -0.1);
    ROS_WARN("Point 4 is (%f, %f %f)", p5.x(), p5.y(), p5.z());
    KDL::Vector p6(0.5, 0.5, 0.0);
    ROS_WARN("Point 5 is (%f, %f %f)", p6.x(), p6.y(), p6.z());
    KDL::Vector p7(0.0, 0.5, 0.0);
    ROS_WARN("Point 6 is (%f, %f %f)", p7.x(), p7.y(), p7.z());
    KDL::Vector p8(0.5, 1.5, 1.0);
    ROS_WARN("Point 7 is (%f, %f %f)", p8.x(), p8.y(), p8.z());


    std::list<KDL::Vector> points;
    points.push_back(p1);
    points.push_back(p2);
    points.push_back(p3);
    points.push_back(p4);
    points.push_back(p5);
    points.push_back(p6);
    points.push_back(p7);
    points.push_back(p8);

    yarp::sig::Matrix A;
    yarp::sig::Vector b;

    ch.getConvexHull(points, A, b);
    ROS_INFO("A: %s", A.toString().c_str());
    ROS_INFO("b: %s", b.toString().c_str());

    return 0;
}

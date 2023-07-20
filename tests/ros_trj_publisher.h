/*
 * Copyright (C) 2016 Walkman
 * Author: Enrico Mingo Hoffman
 * email:  enrico.mingo@iit
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

#ifndef __ROS_TRJ_PUBLISHER_H__
#define __ROS_TRJ_PUBLISHER_H__

#include <ros/ros.h>
#include <ros/publisher.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>
#include "trajectory_utils/trajectory_utils.h"
#include <rviz_visual_tools/rviz_visual_tools.h>

namespace trajectory_utils{

class trajectory_publisher
{
public:
    /**
     * @brief trajectory_publisher
     * @param topic_name trajectories will be published using this topic name
     */
    trajectory_publisher(const std::string& topic_name);

    /**
     * @brief setTrj is used to set the next trajctory that will be displayed
     * @param trj
     * @param base_frame wrt the trajectory is displayed
     * @param distal_frame end-effector name
     */
    void setTrj(const boost::shared_ptr<KDL::Trajectory_Composite> trj,
                const std::string& base_frame,
                const std::string& distal_frame);

    /**
     * @brief setDecimation
     * @param decimation is the dt in [sec] used to print the trj
     */
    void setDecimation(const double decimation);

    /**
     * @brief setDecimation2
     * @param decimation2 is the step to print the frames
     */
    void setDecimation2(const int decimation2);

    /**
     * @brief publish trajectories and frames in rviz
     * @param delete_visual_tools if true all the visual tools and trj
     * are cleared before publish. This should be true if the
     * trajectory displayed is wrt a moving frame. Therefore also the trajectory
     * will move if the base_frame will move. Unfortunately this creates a bad
     * visualization of the marker...TO FIX
     */
    void publish(bool delete_visual_tools = false);

    /**
     * @brief deleteAllMarkers remove all the markers and trjs
     */
    void deleteAllMarkersAndTrj();

private:
    ros::NodeHandle _n;

    ros::Publisher _trj_publisher;
    nav_msgs::Path _trj_msg;

    std::string _frame;

    double _decimation;
    int _decimation2;

    bool _has_traj;

};

}


#endif

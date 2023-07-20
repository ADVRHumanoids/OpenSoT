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

#ifndef __TRAJECTORY_UTILS_H__
#define __TRAJECTORY_UTILS_H__

#include "path_circle_fix.hpp"
#include <kdl/path_line.hpp>
#include <kdl/path_point.hpp>
#include <kdl/path_composite.hpp>

#include <kdl/trajectory_composite.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/trajectory_stationary.hpp>

#include <kdl/velocityprofile_trap.hpp>
#include <kdl/velocityprofile_spline.hpp>

#include <kdl/rotational_interpolation_sa.hpp>

#include <boost/shared_ptr.hpp>

namespace trajectory_utils{

class trajectory_generator{
public:
    /**
     * @brief trajectory_generator
     * @param dt
     * @param base_frame
     * @param distal_frame
     */
    trajectory_generator(const double dt, const std::string& base_frame, const std::string& distal_frame);

    /**
     * @brief trajectory_generator constructor
     * @param dt loop time
     */
    trajectory_generator(const double dt);

    /**
     * @brief getBaseFrame
     * @return
     */
    std::string getBaseFrame(){
        return _base_frame;
    }

    /**
     * @brief getDistalFrame
     * @return
     */
    std::string getDistalFrame(){
        return _distal_frame;
    }

    /**
     * @brief getSampleTime
     * @return sample time of the trajectory
     */
    double getSampleTime(){
        return _dt;
    }

    /**
     * @brief changeBaseFrame
     * @param base_frame
     */
    void changeBaseFrame(const std::string& base_frame){
        _base_frame = base_frame;
    }

    /**
     * @brief changeTipFrame
     * @param tip_frame
     */
    void changeDistalFrame(const std::string& distal_frame){
        _distal_frame = distal_frame;
    }

    /**
     * @brief changeBaseTipFrame
     * @param base_frame
     * @param tip_frame
     */
    void changeBaseTipFrame(const std::string& base_frame, const std::string& tip_frame){
        changeBaseFrame(base_frame);
        changeDistalFrame(tip_frame);
    }

    /**
     * @brief getTrajectory
     * @return the sharep pointer to the KDL trajectory object
     */
    boost::shared_ptr<KDL::Trajectory_Composite> getTrajectory();

    /**
     * @brief resetTrajectory clear the trajectory object (all the stored trajectories are deleted)
     */
    void resetTrajectory();

    //********MinJerkTrj********
    /**
     * @brief addMinJerkTrj add a linear trajectories specified by way points, each performed in Ts secs, in
     * thich the jerk is minimized
     * @param way_points NOTE that the first way-point is the start!
     * @param T time of each sub trajectory
     * @return true if the trajectory is added
     */
    bool addMinJerkTrj(const std::vector<KDL::Frame> &way_points, const double T);

    /**
     * @brief addMinJerkTrj add a linear trajectories specified by way points, each performed in Ts secs, in
     * thich the jerk is minimized
     * @param way_points NOTE that the first way-point is the start!
     * @param Ts time of each sub trajectory
     * @return true if the trajectory is added
     */
    bool addMinJerkTrj(const std::vector<KDL::Frame> &way_points, const std::vector<double> Ts);

    /**
     * @brief addMinJerkTrj add a linear trajectory between start and end, performed in T secs, in which
     * the jerk is minimized
     * @param start frame
     * @param end frame
     * @param T time of the trajectory
     * @return true of the trajectory is added
     */
    bool addMinJerkTrj(const KDL::Frame& start, const KDL::Frame& end, const double T);
    //************************

    //******ArcTrj************
    /**
     * @brief addArcTrj add an arc trajectory with quintic spline velocity profile
     *
     * @param start_pose on the arc trajectory
     * @param final_rotation at the end of the arc trajectory
     * @param angle_of_rotation amount of rotation (arc trajectory)
     * @param circle_center center of the arc trajectory
     * @param plane_normal normal of the plane where the arc trajecotry is
     * @param T time of the trajectory
     * @param v0 initial velocity
     * @param v1 final velocity
     * @param a0 initial acceleration
     * @param a1 final acceleration
     * @return true of the trajectory is added
     */
    bool addArcTrj(const KDL::Frame &start_pose, const KDL::Rotation &final_rotation,
                   const double angle_of_rotation, const KDL::Vector &circle_center,
                   const KDL::Vector &plane_normal, const double T,
                   const double v0, const double v1, const double a0, const double a1);

    /**
     * @brief addArcTrj add an arc trajectory constituted by a arc path and a BANG_COAST_BANG
     * velocity profile. Here we assume that the BANG phases least as the COAST phase
     * @param start_pose on the arc trajectory
     * @param final_rotation at the end of the arc trajectory
     * @param angle_of_rotation amount of rotation (arc trajectory)
     * @param circle_center center of the arc trajectory
     * @param plane_normal normal of the plane where the arc trajecotry is
     * @param T time of the trajectory
     * @return true if the trajectories has been added
     */
    bool addArcTrj(const KDL::Frame &start_pose, const KDL::Rotation &final_rotation,
                   const double angle_of_rotation,
                   const KDL::Vector &circle_center, const KDL::Vector &plane_normal,
                   const double T);

    /**
     * @brief addArcTrj add an arc trajectory with Bang-Coast-Bang velocity profile
     *
     * @param start_pose on the arc trajectory
     * @param final_rotation final rotation at the end of the arc trajectory
     * @param angle_of_rotation amount of rotation (arc trajectory)
     * @param circle_center center of the arc trajectory
     * @param plane_normal normal of the plane where the arc trajecotry is
     * @param max_vel max velocity of the trajectory
     * @param max_acc max acceleration of the trajectory
     * @return true if the trajectories has been added
     */
    bool addArcTrj(const KDL::Frame& start_pose, const KDL::Rotation& final_rotation,
                   const double angle_of_rotation,
                   const KDL::Vector& circle_center, const KDL::Vector& plane_normal,
                   const double max_vel, const double max_acc);
    //************************

    //*********Linetrj********
    /**
     * @brief addLineTrj add a set of linear trajectories, specified by way-points and each
     * costituted by a BANG_COAST_BANG velocity profile. Here we assume that the BANG phases least as
     * the COAST phase
     * @param way_points NOTE that the first way-point is the start!
     * @param T all the segments least the same time
     * @return true if the trajectories has been added
     */
    bool addLineTrj(const std::vector<KDL::Frame>& way_points, double T);

    /**
     * @brief addLineTrj add a set of linear trajectories, specified by way-points and each
     * costituted by a BANG_COAST_BANG velocity profile. Here we assume that the BANG phases least as
     * the COAST phase
     * @param way_points NOTE that the first way-point is the start!
     * @param T duration of each segment of trajectory
     * @return true if the trajectories has been added
     */
    bool addLineTrj(const std::vector<KDL::Frame>& way_points, const std::vector<double> T);

    /**
     * @brief addLineTrj addLineTrj add a set of linear trajectories, specified by way-points,
     * each constituted by a linear path and a bang-coast-bang velocity profile.
     * @param way_points NOTE that the first way-point is the start!
     * @param max_vels max velocity for all the trajectories
     * @param max_accs max acceleration for all the trajectories
     * @return true if the trajectories has been added
     */
    bool addLineTrj(const std::vector<KDL::Frame>& way_points,
                    const double max_vel, const double max_acc);

    /**
     * @brief addLineTrj add a set of linear trajectories, specified by way-points,
     * each constituted by a linear path and a bang-coast-bang velocity profile.
     * @param way_points NOTE that the first way-point is the start!
     * @param max_vels max velocity for each trajectory
     * @param max_accs max acceleration for each trajectory
     * @return true if the trajectories has been added
     */
    bool addLineTrj(const std::vector<KDL::Frame>& way_points,
                     const std::vector<double> max_vels, const std::vector<double> max_accs);
    /**
     * @brief addLineTrj add a trajectory constituted by a linear path and a
     * bang-coast-bang velocity profile
     *
     * @param start Frame
     * @param end Frame
     * @param max_vel max velocity of the trajectory
     * @param max_acc max acceleration of the trajectory
     * @return true if the trajectory has been added
     */
    bool addLineTrj(const KDL::Frame& start, const KDL::Frame& end,
                    const double max_vel, const double max_acc);

    /**
     * @brief addLineTrj add a trajectory of duration T constituted by a linear path and a BANG_COAST_BANG
     * velocity profile. Here we assume that the BANG phases least as the COAST phase
     * @param start Frame
     * @param end Frame
     * @param T total time of the trajectory
     * @return true if the trajectory has been added
     */
    bool addLineTrj(const KDL::Frame& start, const KDL::Frame& end, const double T);

    /**
     * @brief addLineTrj add a trajectory constituted by a linear path and a
     * quintic spline as velocity profile.
     *
     * NOTE if v0 = v1 = a0 = a1 = 0.0 then you are using a min jerk trajectory!!!
     *
     * @param start Frame
     * @param end Frame
     * @param T time of the trajectory
     * @param v0 initial velocity
     * @param v1 final velocity
     * @param a0 initial acceleration
     * @param a1 final acceleration
     * @return true if the trajectory has been added
     */
    bool addLineTrj(const KDL::Frame& start, const KDL::Frame& end, const double T,
                    const double v0, const double v1, const double a0, const double a1);
    //**************************

    /**
     * @brief Pos return a frame from the trajectory at time t
     * @param t time
     * @return a frame
     */
    KDL::Frame Pos(double t);

    /**
     * @brief Vel a twist from the trajectory at time t.
     * @param t time
     * @return a twist
     */
    KDL::Twist Vel(double t);

    /**
     * @brief Acc a twist (representing accelerations) from the trajectory at time t.
     * @param t time
     * @return a twist
     */
    KDL::Twist Acc(double t);

    /**
     * @brief Pos a frame from the trajectory at actual (internal) time
     * @return a frame
     */
    KDL::Frame Pos();

    /**
     * @brief Vel a twist from the trajectory at actual (internal) time.
     * @return a twist
     */
    KDL::Twist Vel();

    /**
     * @brief Acc a twist (representing accelerations) from the trajectory at actual (internal) time.
     * @return a twist
     */
    KDL::Twist Acc();

    /**
     * @brief updateTrj increment internal time
     */
    void updateTrj();

    /**
     * @brief resetInternalTime set internal time to 0
     */
    void resetInternalTime();

    /**
     * @brief getTime return internal time
     * @return time
     */
    double getTime();

    /**
     * @brief Duration fot he whole trajectory
     * @return time of the trajectory
     */
    double Duration();

    /**
     * @brief isFinished
     * @return true if internal time >= duration
     */
    bool isFinished();

    /**
     * @brief isStarted
     * @return true if time > 0.0
     */
    bool isStarted();

    /**
     * @brief isRunning
     * @return true if started and not finished
     */
    bool isRunning();

    /**
     * @brief isInited
     * @return true if at least one trajectory has been added
     */
    bool isInited();


protected:
    double _dt;
    double _time;
    double _eq_radius;
    bool   _is_inited;
    std::string _base_frame;
    std::string _distal_frame;

    boost::shared_ptr<KDL::Trajectory_Composite> _trj;

    boost::shared_ptr<KDL::Path> createLinePath(const KDL::Frame& start, const KDL::Frame& end);

    boost::shared_ptr<KDL::Path> createArcPath(const KDL::Frame& start_pose,
                                               const KDL::Rotation& final_rotation,
                                               const double angle_of_rotation,
                                               const KDL::Vector& circle_center,
                                               const KDL::Vector& plane_normal);

    boost::shared_ptr<KDL::VelocityProfile> createTrapezoidalVelProfile(const double max_vel, const double max_acc,
                                                                        const double L);

    boost::shared_ptr<KDL::VelocityProfile> createSplineVelProfile(const double L, const double T);
    boost::shared_ptr<KDL::VelocityProfile> createSplineVelProfile(const double L, const double T,
                                                                   const double v0, const double v1);
    boost::shared_ptr<KDL::VelocityProfile> createSplineVelProfile(const double L, const double T,
                                                                   const double v0, const double v1,
                                                                   const double a0, const double a1);

   bool checkIfCoastPhaseExists(const double max_vel, const double max_acc, const double L);

   void normalizeQuaternion(KDL::Frame& T);

   void computeMaxVelAndMaxAccForBCB(const double T, const double L, double& max_vel, double& max_acc);
};

}

#endif

/*
 * Copyright (C) 2016 Walkman
 * Authors:Enrico Mingo Hoffman, Alessio Rocchi
 * email:  alessio.rocchi@iit.it, enrico.mingo@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU Lesser General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#ifndef __TASKS_VELOCITY_GAZE_H__
#define __TASKS_VELOCITY_GAZE_H__

#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <OpenSoT/SubTask.h>
#include <OpenSoT/utils/cartesian_utils.h>
#include <XBotInterface/ModelInterface.h>

namespace OpenSoT {
namespace tasks {
namespace velocity {


/**
 * @brief The Gaze class implement a Cartesian Task in which is controlled the gaze of
 * the robot.
 * The algorithm used is based on the paper: "Adaptive Predictive Gaze Control of a Redundant Humanoid
 * Robot Head, IROS2011".
 * Notice that the controlled distal link is always "gaze" in a certain base_link set
 * by the user.
 */
class Gaze: public OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>
{
public:
    typedef std::shared_ptr<Gaze> Ptr;

    Gaze(std::string task_id,
         const Eigen::VectorXd &x,
         XBot::ModelInterface &robot,
         std::string base_link,
         std::string distal_link = "gaze");

    ~Gaze();

    /**
     * @brief setGaze
     * @param desiredGaze pose of the object to observe in base_link
     */
    void setGaze(const Eigen::Affine3d& desiredGaze);
    [[deprecated]]
    void setGaze(const Eigen::MatrixXd& desiredGaze);
    void setGaze(const KDL::Frame& desiredGaze);

    void setOrientationErrorGain(const double& orientationErrorGain);

    const double getOrientationErrorGain() const;

    /**
     * @brief setWeight sets the task weight.
     * Note the Weight needs to be positive definite.
     * If your original intent was to get a subtask
     * (i.e., reduce the number of rows of the task Jacobian),
     * please use the class SubTask
     * @param W matrix weight
     */
    virtual void setWeight(const Eigen::MatrixXd& W);

    /**
     * @brief getConstraints return a reference to the constraint list. Use the standard list methods
     * to add, remove, clear, ... the constraints list.
     * e.g.:
     *              task.getConstraints().push_back(new_constraint)
     * Notice that in subtasks, you will get the constraint list of the father Task from which the SubTask
     * is generated.
     * @return the list of constraints to which the father Task is subject.
     */
    virtual std::list< ConstraintPtr >& getConstraints();

    /** Gets the task size.
        @return the number of rows of A */
    virtual const unsigned int getTaskSize() const;

    /** Updates the A, b, Aeq, beq, Aineq, b*Bound matrices
        @param x variable state at the current step (input) */
    virtual void _update(const Eigen::VectorXd &x);

    /**
     * @brief getActiveJointsMask return a vector of length NumberOfDOFs.
     * If an element is false the corresponding column of the task jacobian is set to 0.
     * @return a vector of bool
     */
    virtual std::vector<bool> getActiveJointsMask();

    /**
     * @brief setActiveJointsMask set a mask on the jacobian
     * @param active_joints_mask
     * @return true if success
     */
    virtual bool setActiveJointsMask(const std::vector<bool>& active_joints_mask);

    /**
     * @brief getDistalLink return "gaze" as controlled link
     * @return string with distal link name
     */
    std::string getDistalLink(){ return _distal_link;}

    /**
     * @brief getLambda
     * @return the lambda weight of the task
     */
    virtual void setLambda(double lambda);

    /**
     * @brief setBaseLink change the base link of the task
     * @param base_link the new base link
     * @return false if the base link does not exists
     */
    bool setBaseLink(const std::string& base_link);

private:
    std::string _distal_link;
    Cartesian::Ptr _cartesian_task;
    SubTask::Ptr   _subtask;

    Eigen::Affine3d _gaze_T_obj;
    Eigen::VectorXd _tmp_vector;

    XBot::ModelInterface& _robot;

    KDL::Frame _bl_T_gaze_kdl;
    KDL::Frame _gaze_goal;
    Eigen::Affine3d _tmpEigenM;
    Eigen::Affine3d _tmpEigenM2;



};

}

}

}

#endif

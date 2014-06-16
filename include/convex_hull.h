#ifndef _CONVEX_HULL_H_
#define _CONVEX_HULL_H_

#include <moveit/robot_model/robot_model.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <kdl/frames.hpp>
#include <drc_shared/cartesian_utils.h>

namespace wb_sot
{

struct foot_reference_frames
{
    KDL::Frame upper_left;
    KDL::Frame upper_right;
    KDL::Frame lower_left;
    KDL::Frame lower_right;

    static std::string _UPPER_LEFT;
    static std::string _LOWER_LEFT;
    static std::string _UPPER_RIGHT;
    static std::string _LOWER_RIGHT;
};

class convex_hull
{
public:
    convex_hull(){}
    /**
     * @brief convex_hull
     * @param robot_urdf
     * @param l_ankle_CoM pose from left ankle to CoM
     * @param r_ankle_CoM pose from right ankle to CoM
     */
    void init(const boost::shared_ptr<const urdf::ModelInterface> &robot_urdf,
                const KDL::Frame& l_ankle_CoM, const KDL::Frame& r_ankle_CoM);
    ~convex_hull();

    void updateCoMPosition(const KDL::Frame& l_ankle_CoM, const KDL::Frame& r_ankle_CoM)
    {
        _l_ankle_CoM = l_ankle_CoM;
        _r_ankle_CoM = r_ankle_CoM;
    }

    void computeConvexHull();

private:
    boost::shared_ptr<const urdf::ModelInterface> _robot_urdf;
    foot_reference_frames _l_foot; //from left ankle to reference frame in sole
    foot_reference_frames _r_foot; //from right ankle to reference frame in sole
    KDL::Frame _l_ankle_CoM; //From left ankle to CoM
    KDL::Frame _r_ankle_CoM; //From left ankle to CoM

    /**
     * @brief getRefFrames from ankle to sole
     * @param foot_name
     * @param sole_name
     * @param foot
     */
    void getRefFrames(const std::string& foot_name, const std::string &sole_name,
                      foot_reference_frames& foot);

    void URDFPose2KDLFrame(const urdf::Pose& pose, KDL::Frame& T);
};

}

#endif

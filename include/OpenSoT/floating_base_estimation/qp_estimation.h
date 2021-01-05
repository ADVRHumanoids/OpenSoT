#ifndef _OPENSOT_FLOATING_BASE_ESTIMATION_QP_ESTIMATION_
#define _OPENSOT_FLOATING_BASE_ESTIMATION_QP_ESTIMATION_

#include <OpenSoT/utils/FloatingBaseEstimation.h>
#include <OpenSoT/tasks/floating_base/Contact.h>
#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/constraints/GenericConstraint.h>
#include <OpenSoT/tasks/floating_base/IMU.h>


namespace OpenSoT{
namespace floating_base_estimation{
/**
     * @brief The qp_estimation class uses a QP to estimate the floating base pose and velocities from
     * contact information and IMU (optional).
     * The QP solved is a weighted sum of the measurements.
     */
    class qp_estimation: public OpenSoT::FloatingBaseEstimation
    {
    public:
        typedef boost::shared_ptr<qp_estimation> Ptr;

        qp_estimation(XBot::ModelInterface::Ptr model,
                      std::vector<std::string> contact_links,
                      const Eigen::MatrixXd& contact_matrix = Eigen::MatrixXd::Identity(6,6));
        ~qp_estimation();
        bool update(OpenSoT::FloatingBaseEstimation::Update update =
                        OpenSoT::FloatingBaseEstimation::Update::None);

        virtual bool setContactState(const std::string& contact_link, const bool state);

        virtual void log(XBot::MatLogger2::Ptr logger);

        AutoStack::Ptr getStack()
        {
            return _autostack;
        }

        solvers::iHQP::Ptr getSolver()
        {
            return _solver;
        }

    private:
        std::list<OpenSoT::tasks::Aggregated::TaskPtr> _contact_tasks;
        std::map<std::string, unsigned int> _map_tasks;
        tasks::Aggregated::Ptr _aggregated_tasks;
        solvers::iHQP::Ptr _solver;
        constraints::GenericConstraint::Ptr _fb_limits;
        AutoStack::Ptr _autostack;

    };

    class kinematic_estimation
    {
    public:
        typedef boost::shared_ptr<kinematic_estimation> Ptr;

        /**
         * @brief kinematic_estimation, at the moment only the pose of the floating base is computed
         * @param model
         * @param anchor_link
         * @param anchor_pose is the pose of the anchor in world frame
         */
        kinematic_estimation(XBot::ModelInterface::Ptr model, const std::string& anchor_link,
                             const Eigen::Affine3d& anchor_pose = Eigen::Affine3d::Identity());


        /**
         * @brief getAnchor
         * @return link which represents the anchor
         */
        const std::string& getAnchor();

        /**
         * @brief setAnchor permits to change the anchor link.
         * @param anchor_link
         * @return true if anchor_link exists
         */
        bool setAnchor(const std::string& anchor_link);

        /**
         * @brief getAnchorPose
         * @return the anchor pose in world frame
         */
        const Eigen::Affine3d& getAnchorPose();
        void getAnchorPose(Eigen::Affine3d& anchor_pose);
        /**
         * @brief setAnchorPose set the pose of the anchor in world frame
         * @param world_T_anchor
         */
        void setAnchorPose(const Eigen::Affine3d& world_T_anchor);

        /**
         * @brief update computes the new pose of the floating base in world considering the actual anchor link
         * @param update_model if false the model is not updated with the new computed floating base pose, otherwise is udpated
         */
        void update(const bool update_model = true);

        /**
         * @brief getFloatingBasePose
         * @return the last updated floating base pose in world frame
         */
        const Eigen::Affine3d& getFloatingBasePose();

        /**
         * @brief getFloatingBaseTwist
         * @return the last updated floating base twist in world frame
         */
        const Eigen::Vector6d& getFloatingBaseTwist();

    private:
        /**
         * @brief _anchor_link is a link of the robot which is assumed fixed wrt the world frame.
         */
        std::string _anchor_link;

        std::string _base_link;

        /**
         * @brief _world_T_anchor is the pose of the anchor in world frame
         */
        Eigen::Affine3d _world_T_anchor;

        XBot::ModelInterface::Ptr _model;

        Eigen::Affine3d _old_anchor_T_new_anchor;
        Eigen::Affine3d _world_T_new_anchor;
        Eigen::Affine3d _anchor_T_base_link;

        /**
         * @brief _world_T_base_link pose of base_link in world_frame
         */
        Eigen::Affine3d _world_T_base_link;

    };
}

}

#endif

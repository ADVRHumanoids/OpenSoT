#ifndef __OPENSOT_SE3_TASK_OC_H__
#define __OPENSOT_SE3_TASK_OC_H__

#include <OpenSoT/Task.h>
#include <OpenSoT/utils/Affine.h>
#include <xbot2_interface/xbotinterface2.h>
// #include 


namespace OpenSoT::oc{


class SE3Task : public OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>{

    public:
        typedef std::shared_ptr<SE3Task> Ptr;

        SE3Task(const std::string& id, const XBot::ModelInterface& robot, const AffineHelper& dx, const std::string& distal_frame);

        /*
        @param T: reference in world frame
        */
        void setReference(const Eigen::Affine3d& T)
        {
            _ref = T;
        }

        const Eigen::Affine3d& getReference() const
        {
            return _ref;
        }

        const std::string& getDistalFrame() const
        {
            return _distal_frame;
        }

        const Eigen::Vector6d& getError();


    private:
        const XBot::ModelInterface& _robot;
        AffineHelper _dx;
        AffineHelper _task;

        std::string _distal_frame;

        Eigen::Affine3d _ref;
        Eigen::Affine3d _d_T_w;

        Eigen::MatrixXd _J;
        Eigen::Matrix6d _Adj;

        Eigen::Vector6d _w;

        virtual void _update();

};
}





#endif
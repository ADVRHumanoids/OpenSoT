#ifndef __OPENSOT_TASKS_VELOCITY_PURE_ROLLING_H__
#define __OPENSOT_TASKS_VELOCITY_PURE_ROLLING_H__

#include <OpenSoT/Task.h>
#include <XBotInterface/ModelInterface.h>
#include <OpenSoT/SubTask.h>

namespace OpenSoT { namespace tasks { namespace velocity {
    
    class PureRolling : public Task<Eigen::MatrixXd, Eigen::VectorXd> {
        
    public:
        
        typedef boost::shared_ptr<PureRolling> Ptr;
        
        PureRolling(std::string wheel_link_name, 
                    double radius,
                    const XBot::ModelInterface& model);
        
        virtual void _update(const Eigen::VectorXd& x);
        
        virtual void _log(XBot::MatLogger::Ptr logger);
        
    private:
        
        const XBot::ModelInterface& _model;
        std::string _wheel_link_name;
        Eigen::Vector3d _world_contact_plane_normal;
        Eigen::Vector3d _wheel_axis;
        Eigen::Vector3d _wheel_contact_point;
        double _radius;
        
        Eigen::MatrixXd _Jc;
        Eigen::MatrixXd _S;
        
        Eigen::VectorXd _qdot;
        
        Eigen::Affine3d _world_T_wheel;
    Eigen::Matrix3d _world_R_wheel;
        
    };

    class PureRollingPosition : public Task<Eigen::MatrixXd, Eigen::VectorXd>
    {
    public:
        typedef boost::shared_ptr<PureRollingPosition> Ptr;

        PureRollingPosition(std::string wheel_link_name,
                    double radius,
                    const XBot::ModelInterface& model);

        virtual void _update(const Eigen::VectorXd& x);


    private:
        PureRolling::Ptr _pure_rolling;
        OpenSoT::SubTask::Ptr _subtask;
        std::list<unsigned int> _position_indices;

    };

    class PureRollingOrientation : public Task<Eigen::MatrixXd, Eigen::VectorXd>
    {
    public:
        typedef boost::shared_ptr<PureRollingOrientation> Ptr;

        PureRollingOrientation(std::string wheel_link_name,
                    double radius,
                    const XBot::ModelInterface& model);

        virtual void _update(const Eigen::VectorXd& x);


    private:
        PureRolling::Ptr _pure_rolling;
        OpenSoT::SubTask::Ptr _subtask;
        std::list<unsigned int> _orientation_indices;

    };
    
} } }





#endif

#ifndef __OPENSOT_TASKS_VELOCITY_ICR_H__
#define __OPENSOT_TASKS_VELOCITY_ICR_H__

#include <OpenSoT/Task.h>
#include <XBotInterface/ModelInterface.h>

namespace OpenSoT { namespace tasks { namespace velocity {
    
    class RigidRotation : public Task<Eigen::MatrixXd, Eigen::VectorXd> {
        
    public:
        
        typedef boost::shared_ptr<RigidRotation> Ptr;
        
        RigidRotation(std::string wheel_link_name, 
                      std::string waist_link_name,
                      const XBot::ModelInterface& model);
        
        void setReference(const Eigen::Vector6d& twist);
        
        virtual void _update(const Eigen::VectorXd& x);
        
        virtual void _log(XBot::MatLogger::Ptr logger);
        
    private:
        
        void setIcr();
        
        const XBot::ModelInterface& _model;
        std::string _wheel_link_name;
        std::string _waist_link_name;
        std::string _parent_parent_link; /* HACK */
        
        Eigen::Vector3d _world_contact_plane_normal;
        Eigen::Vector3d _waist_icr;
        double _thetadot;
        Eigen::Vector3d _wheel_spinning_axis;
        Eigen::Vector3d _wheel_forward_axis_ref;
        Eigen::Vector3d _wheel_forward_axis;
        Eigen::Vector3d _waist_forward_axis_ref;
        Eigen::Vector3d _waist_forward_axis;
        Eigen::Vector3d _world_omega_ref;
        Eigen::Vector6d _waist_ref_twist;
        
        Eigen::Vector3d _preferred_forward_axis;
        
        Eigen::MatrixXd _Jwheel;
        Eigen::MatrixXd _S;
        
        Eigen::VectorXd _qdot;
        
    };
    
} } }



#endif
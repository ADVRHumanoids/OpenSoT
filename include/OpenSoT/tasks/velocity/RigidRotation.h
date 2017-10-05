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
                      const XBot::ModelInterface& model,
                      double dt
                     );
        
        void setReference(double theta, const Eigen::Vector3d& cart_vref);
        void setReference(double theta, const Eigen::Vector3d& cart_vref, const Eigen::Vector2d& wheel_pos_ref);
        
        virtual void _update(const Eigen::VectorXd& x);
        
        virtual void _log(XBot::MatLogger::Ptr logger);
        
    private:
        
        double _dt;
        
        const XBot::ModelInterface& _model;
        std::string _wheel_link_name;
        std::string _waist_link_name;
        std::string _parent_parent_link; /* HACK */
        
        Eigen::Vector3d _cart_wheel_pos_ref;
        Eigen::Vector3d _world_contact_plane_normal;
        Eigen::Vector3d _cart_vref;
        Eigen::Matrix3d _world_R_cart;
        Eigen::Vector3d _wheel_spinning_axis;
        Eigen::Vector3d _wheel_forward_axis_ref;
        Eigen::Vector3d _wheel_forward_axis;
        Eigen::Vector3d _cart_forward_axis_ref;
        Eigen::Vector3d _cart_forward_axis;
        Eigen::Vector3d _cart_omega_ref;
        Eigen::Vector3d _cart_wheel_pos;
        Eigen::Vector3d _preferred_forward_axis;
        
        Eigen::Vector3d _wheel_vel_ref;
        Eigen::Vector3d _wheel_relative_vel_ref;
        Eigen::MatrixXd _Jwheel, _Jwaist, _Jrel;
        Eigen::MatrixXd _S;
        
        Eigen::VectorXd _qdot;
        
    };
    
} } }



#endif
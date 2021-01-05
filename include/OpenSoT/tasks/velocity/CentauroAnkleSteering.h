#ifndef __OPENSOT_CENTAURO_ANKLE_STEERING_H__
#define __OPENSOT_CENTAURO_ANKLE_STEERING_H__

#include <OpenSoT/tasks/velocity/Postural.h>

namespace OpenSoT { namespace tasks { namespace velocity {
    
    class HysteresisComparator
    {
      
    public:
        
        HysteresisComparator(double th_lo = -1., 
                             double th_hi =  1., 
                             bool init_lo = true);
        
        bool compare(double value);
        
        double getCurrentThreshold() const;
        
    private:
        
        const double _th_lo, _th_hi;
        double _th_curr;
        
    };

    
    class SimpleSteering 
    {
      
    public:
        
        SimpleSteering(XBot::ModelInterface::ConstPtr model, 
                       std::string wheel_name);
        
        double computeSteeringAngle(const Eigen::Vector3d& wheel_vel);
        
        double getDofIndex() const;
        
        void setOutwardNormal(const Eigen::Vector3d& n);
        
        const std::string& getWheelName() const { return _wheel_name; }
        
        void log(XBot::MatLogger2::Ptr logger);
        
    private:
        
        struct LogData
        {
            double theta, theta_ref, q;
            Eigen::Vector3d steering_axis, normal, forward, vdes;
        };
        
        static double wrap_angle(double q);
        static double sign(double x);
        
        LogData _log_data;
        
        HysteresisComparator _comp;
        int _steering_id;
        Eigen::Vector3d _local_steering_axis, _wheel_spinning_axis;
        Eigen::Matrix3d _local_R_world;
        XBot::ModelInterface::ConstPtr _model;
        XBot::Joint::ConstPtr _steering_joint;
        std::string _wheel_name;
        std::string _waist_name;
        std::string _wheel_parent_name;
        Eigen::VectorXd _q;
        
        Eigen::Vector3d _vdes;
        
        double _prev_qdes;
        
        
    };
    
    class CentauroAnkleSteering : public Task<Eigen::MatrixXd, Eigen::VectorXd>
    {
      
    public:
        
        static constexpr double DEFAULT_MAX_STEERING_SPEED = 3.0;
        
        CentauroAnkleSteering(std::string wheel_name, 
                              XBot::ModelInterface::ConstPtr model,
                              double dt,
                              double max_steering_speed = DEFAULT_MAX_STEERING_SPEED
                             );
        
        void setOutwardNormal(const Eigen::Vector3d& n);
        
        void _update(const Eigen::VectorXd& x) override;
        
        void _log(XBot::MatLogger2::Ptr logger) override;
        
        
    private:
        
        SimpleSteering _steering;
        
        XBot::ModelInterface::ConstPtr _model;
        XBot::Joint::ConstPtr _steering_joint;
        Eigen::VectorXd _q;
        int _steering_dof_idx;
        
        double _max_steering_dq;
    };
    

    
} } }








#endif 

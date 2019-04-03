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
        
        const std::string& getWheelName() const { return _wheel_name; }
        
        void log(XBot::MatLogger::Ptr logger);
        
    private:
        
        static double wrap_angle(double q);
        static double sign(double x);
        
        HysteresisComparator _comp;
        int _steering_id;
        Eigen::Vector3d _world_steering_axis, _wheel_spinning_axis;
        XBot::ModelInterface::ConstPtr _model;
        XBot::Joint::ConstPtr _steering_joint;
        std::string _wheel_name;
        std::string _waist_name;
        Eigen::VectorXd _q;
        
        Eigen::Vector3d _vdes;
        
        
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
        
        void _update(const Eigen::VectorXd& x) override;
        
        
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

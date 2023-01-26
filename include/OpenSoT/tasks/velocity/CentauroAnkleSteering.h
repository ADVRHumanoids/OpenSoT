#ifndef __OPENSOT_CENTAURO_ANKLE_STEERING_H__
#define __OPENSOT_CENTAURO_ANKLE_STEERING_H__

#include <OpenSoT/tasks/velocity/Postural.h>

namespace OpenSoT { namespace tasks { namespace velocity {
    
    class HysteresisComparator
    {
      
    public:
        
        HysteresisComparator(double th_lo,
                             double th_hi,
                             bool init_lo);

        HysteresisComparator& operator=(HysteresisComparator other)
        {
            _th_lo = other._th_lo;
            _th_hi = other._th_hi;
            _th_curr = other._th_curr;

            return *this;
        }

        static HysteresisComparator MakeHysteresisComparator(double th_lo = 0.0025,
                                                             double th_hi =  0.01,
                                                             bool init_lo = true);
        
        bool compare(double value);
        
        double getCurrentThreshold() const;
        
     private:

        double _th_lo, _th_hi;
        double _th_curr;

    };

    
    class SimpleSteering 
    {
      
    public:
        
        SimpleSteering(XBot::ModelInterface::ConstPtr model, 
                       std::string wheel_name,
                       std::vector<double> hyst_comp,
                       double dz_th);
        
        double computeSteeringAngle(const Eigen::Vector3d& wheel_vel);
        
        double getDofIndex() const;
        
        void setOutwardNormal(const Eigen::Vector3d& n);

        void setHysteresisComparisonThreshold(double th_lo, double th_hi);
        
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

        double _dz_th;
        
        double _prev_qdes;
        
        
    };
    
    class CentauroAnkleSteering : public Task<Eigen::MatrixXd, Eigen::VectorXd>
    {
      
    public:
        
        static constexpr double DEFAULT_MAX_STEERING_SPEED = 3.0;
        
        CentauroAnkleSteering(std::string wheel_name, 
                              XBot::ModelInterface::ConstPtr model,
                              double dt,
                              double dz_th,
                              double max_steering_speed = DEFAULT_MAX_STEERING_SPEED,
                              std::vector<double> hyst_comp = {}
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

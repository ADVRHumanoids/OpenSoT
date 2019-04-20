#include <OpenSoT/utils/Affine.h>
#include <OpenSoT/Constraint.h>

#include <XBotInterface/ModelInterface.h>

namespace OpenSoT { namespace constraints { namespace force {
    
    class StaticConstraint : public Constraint<Eigen::MatrixXd, Eigen::VectorXd> 
    {
        
    public:
        
        StaticConstraint(const XBot::ModelInterface& robot, 
                        std::vector<std::string> contact_links,
                        std::vector<AffineHelper> forces,
                        AffineHelper robot_torque);
        
        
    private:
        
        void update(const Eigen::VectorXd& x) override;
        
        const XBot::ModelInterface& _robot;
        std::vector<std::string> _contact_links;
        std::vector<AffineHelper> _forces;
        AffineHelper _robot_torque;
        
        Eigen::MatrixXd _J;
        Eigen::VectorXd _gcomp;
        AffineHelper _constr;
        
    };

    
} } }



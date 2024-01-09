#include <OpenSoT/utils/Affine.h>
#include <OpenSoT/Constraint.h>

#include <xbot2_interface/xbotinterface2.h>

namespace OpenSoT { namespace constraints { namespace force {
    
   /**
     * @brief The StaticConstraint class computes the static (only gravity is taken into account) distribution of
     * contact wrenches [6 x 1] given a list of links in contact:
     *
     *      \boldsymbol{\tau} + \mathbf{J}_c^T\mathbf{F} = \mathbf{g}
     *
     */
    class StaticConstraint : public Constraint<Eigen::MatrixXd, Eigen::VectorXd> 
    {
        
    public:
        /**
         * @brief StaticConstraint constructor
         * @param robot model
         * @param contact_links list of link frames in contact
         * @param forces variables
         * @param robot_torque variables
         */
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



#ifndef __QPPVM_FORCE_OPT_H__
#define __QPPVM_FORCE_OPT_H__

#include <OpenSoT/constraints/force/FrictionCone.h>
#include <OpenSoT/Task.h>
#include <OpenSoT/utils/Affine.h>
#include <OpenSoT/solvers/iHQP.h>
#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/SubTask.h>
#include <OpenSoT/constraints/GenericConstraint.h>
#include <OpenSoT/constraints/TaskToConstraint.h>
#include <OpenSoT/tasks/MinimizeVariable.h>
#include <OpenSoT/tasks/force/FloatingBase.h>
#include <OpenSoT/constraints/force/CoP.h>

namespace OpenSoT { namespace utils {
    
    /**
     * @brief This class computes the solution of a force distribution problem,
     * in order to actuate a full N+6 component torque vector on a floating-fixed_base
     * robot, by using ground reaction forces exchanged with the environment.
     * 
     */
    class ForceOptimization 
    {
      
    public:
        
        typedef std::shared_ptr<ForceOptimization> Ptr;
        
        constexpr static double DEFAULT_FRICTION_COEFF = 0.5;
        
        /**
         * @brief Constructor
         * 
         * @param model ModelInterface object that is kept updated with the robot state
         * @param contact_links List of contact links
         * @param optimize_torque False if point contacts are assumed
         */
        ForceOptimization(XBot::ModelInterface::Ptr model, 
                          std::vector<std::string> contact_links,
                          bool optimize_torque = true,
                          double friction_coeff = DEFAULT_FRICTION_COEFF
                          );
        
        /**
         * @brief Translate a fixed-base torque vector to an under-actuated
         * torque vector + contact forces.
         */
        bool compute(const Eigen::VectorXd& fixed_base_torque, 
                     std::vector<Eigen::Vector6d>& Fc,
                     Eigen::VectorXd& tau
                    );
        
        void setContactRotationMatrix(const std::string& contact_link,
                                      const Eigen::Matrix3d& w_R_c);
        
        void log(XBot::MatLogger2::Ptr logger);

        /**
         * @brief getObjective
         * @return the value of the cost function at first priority level
         */
        double getObjective();
        
        
    private:
        
        XBot::ModelInterface::Ptr _model;
        std::vector<std::string> _contact_links;
        std::vector< OpenSoT::AffineHelper > _wrenches;
        
        OpenSoT::constraints::force::FrictionCones::Ptr _friction_cone;
        OpenSoT::tasks::force::FloatingBase::Ptr _forza_giusta;
        OpenSoT::solvers::iHQP::Ptr _solver;
        OpenSoT::AutoStack::Ptr _autostack;
        
        Eigen::VectorXd _x_value;
        Eigen::MatrixXd _JC;
        Eigen::VectorXd _fc_i;
        Eigen::VectorXd _Fci;
        
    };
    
} }



#endif

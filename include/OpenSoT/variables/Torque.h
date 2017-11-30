#ifndef __OPENSOT_VARIABLES_TORQUE_H__
#define __OPENSOT_VARIABLES_TORQUE_H__

#include <OpenSoT/utils/Affine.h>
#include <XBotInterface/ModelInterface.h>
#include <XBotInterface/Utils.h>

namespace OpenSoT { namespace variables {
        
class Torque : public AffineHelper {
  
public:
    
    Torque(XBot::ModelInterface::Ptr model, 
           const AffineHelper& qddot_var,
           std::vector<std::string> contact_links = std::vector<std::string>(),
           std::vector<AffineHelper> force_vars = std::vector<AffineHelper>()
          );

    virtual void update();
    
    
private:
    
    XBot::ModelInterface::Ptr _model;
    
    int _num_contacts;
    std::vector<std::string> _contact_links;

    AffineHelper _qddot_var;
    std::vector<AffineHelper> _force_vars;
    
    Eigen::VectorXd _h;
    std::vector<Eigen::MatrixXd> _Jc;
    Eigen::MatrixXd _B, _S;
    
    Eigen::MatrixXd _C;
    Eigen::VectorXd _d;
    
    AffineHelper _tau;
    
    
};
        
        
} }






#endif

#ifndef __OPENSOT_DYNAMICS_CONSTRAINTS_H__
#define __OPENSOT_DYNAMICS_CONSTRAINTS_H__

#include <OpenSoT/Task.h>
#include <OpenSoT/utils/Affine.h>
#include <xbot2_interface/xbotinterface2.h>


namespace OpenSoT { namespace oc {
class DynamicsConstraint : public OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd> {
public:
    typedef std::shared_ptr<DynamicsConstraint> Ptr;

    DynamicsConstraint(XBot::ModelInterface& robot, const AffineHelper& dX, const AffineHelper& dU, const double dt);


private:
    XBot::ModelInterface& _robot;
    AffineHelper _dU;
    AffineHelper _dX;
    double _dt;

    Eigen::VectorXd _q;
    Eigen::VectorXd _qdot;
    Eigen::VectorXd _qddot;

    Eigen::MatrixXd _dtau_dq;
    Eigen::MatrixXd _dtau_dv;
    Eigen::MatrixXd _dtau_da;

    AffineHelper _dTAU;
    Eigen::MatrixXd _Fx;
    Eigen::MatrixXd _Fu;


    virtual void _update();
    
};


}
}


#endif
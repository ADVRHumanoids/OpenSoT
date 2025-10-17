#ifndef __OPENSOT_TORQUES_TASK_H__
#define __OPENSOT_TORQUES_TASK_H__

#include <OpenSoT/Task.h>
#include <OpenSoT/utils/Affine.h>
#include <xbot2_interface/xbotinterface2.h>


namespace OpenSoT { namespace oc {
class TorquesTask : public OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd> {
public:
    typedef std::shared_ptr<TorquesTask> Ptr;

    TorquesTask(XBot::ModelInterface& robot, const AffineHelper& dX, const AffineHelper& dU);


private:
    XBot::ModelInterface& _robot;
    AffineHelper _dU;
    AffineHelper _dX;

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
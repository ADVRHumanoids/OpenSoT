#ifndef __OPENSOT_SE3_DERIVATIVES_H__
#define __OPENSOT_SE3_DERIVATIVES_H__

#include <OpenSoT/Task.h>
#include <OpenSoT/utils/Affine.h>
#include <xbot2_interface/xbotinterface2.h>
#include <OpenSoT/utils/LieGroupsUtils.h>
// #include 

namespace OpenSoT { namespace oc {
class EulerSE3 : public OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd> {
public:
    typedef std::shared_ptr<EulerSE3> Ptr;

    /**
     * @brief EulerSE3 computes derivatives for SE3 integration:
     * 
     *      \mathbf{X}_{k+1} = \mathbf{X}_k Exp(dt\mahtbf{U}_k)
     * 
     * with derivatives:
     * 
     *      \delta \mathbf{X}_{k+1} = \frac{\partial \mathbf{X}_{k+1}}{\partial \mathbf{X}_{k}\delta\mathbf{X} + \frac{\partial \mathbf{X}_{k+1}}{\partial \mathbf{U}_{k}}\delta\mathbf{U}
     * 
     * @param robot model of the robot
     * @param dX element of the tangent space of SE3
     * @param dU element of the tangent space of SE3
     */
    EulerSE3(const XBot::ModelInterface& robot,
                               const AffineHelper& dX,
                               const AffineHelper& dU,
                               const AffineHelper& Xk,
                               const AffineHelper& Uk,
                               const AffineHelper& Xk_1,
                               const double dt);


private:
    const XBot::ModelInterface& _robot;
    AffineHelper _dU;
    AffineHelper _dX;
    AffineHelper _Xk;
    AffineHelper _Uk;
    AffineHelper _Xk_1;
    double _dt;

    AffineHelper _dXnext;

    Eigen::VectorXd _qdot;
    Eigen::Vector6d _xi;
    Eigen::Matrix3d _RbT;
    Eigen::Matrix3d _t_skew;
    Eigen::Matrix6d _Fx;
    Eigen::Matrix6d _Fu;



    virtual void _update();
    
};


}
}


#endif
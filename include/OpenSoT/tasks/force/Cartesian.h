#ifndef __OPENSOT_FORCE_TASK_CARTESIAN_H__
#define __OPENSOT_FORCE_TASK_CARTESIAN_H__

#include <OpenSoT/Task.h>
#include <OpenSoT/utils/Affine.h>
#include <xbot2_interface/xbotinterface2.h>
#include <xbot2_interface/common/utils.h>

namespace OpenSoT { namespace tasks { namespace force {
 
/**
   * @brief The Cartesian class implements a Cartesian Impedance Controller rsulting in a Catesian Wrench to track:
   *
   *    W = M(xddot_d - Jdotqdot) + K(x_d-x) + D(xdot_d - xdot) + F_d
   *
   */
  class Cartesian : public OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd> {
  
  public:
  
    typedef std::shared_ptr<Cartesian> Ptr;
    
      /**
     * @brief Cartesian
     * @param task_id
     * @param robot
     * @param distal_link controlled frame
     * @param base_link base_frame where motion, impedance and force quantities are expressed
     * @param wrench affine helper variable
     * NOTICE: at the moment, the term M(xddot_d - Jdotqdot) is NOT computed!
     */
    Cartesian(const std::string task_id, 
	      const XBot::ModelInterface& robot,
	      const std::string& distal_link,
	      const std::string& base_link,
	      const AffineHelper& wrench);
    
    /**
     * @brief resetReference reset internal velocities and force references
     */
    void resetReference();
    
    /**
     * @brief setReference for desired pose and velocities
     * @param pose_ref
     * @param vel_ref
     */
    void setReference(const Eigen::Affine3d& pose_ref);
    void setReference(const Eigen::Affine3d& pose_ref, Eigen::Vector6d& vel_ref);
    
    /**
     * @brief setForceReference
     * @param _force_ref
     */
    void setForceReference(const Eigen::Vector6d& _force_ref);
    
    /**
     * @brief getReference
     * @param pose_des
     * @param twist_des
     */
    void getReference(Eigen::Affine3d& pose_des);
    void getReference(Eigen::Affine3d& pose_des, Eigen::Vector6d& twist_des);
    
    /**
     * @brief setCartesianStiffness
     * @param Kp
     * @param Kd
     */
    void setCartesianStiffness(const Eigen::Matrix6d& Kp);
    void setCartesianDamping(const Eigen::Matrix6d& Kd);
    
    /**
     * @brief getCartesianStiffness
     * @param Kp
     * @param Kd
     */
    void getCartesianStiffness(Eigen::Matrix6d& Kp);
    void getCartesianDamping(Eigen::Matrix6d& Kd);
    
    /**
     * @brief getBaseLink
     * @return base frame
     */
    const std::string& getBaseLink() const { return _base_link;}

    /**
     * @brief getDistalLink
     * @return distal frame
     */
    const std::string& getDistalLink() const { return _distal_link;}
  
  private:
  
    static const std::string world_name;
    
    virtual void _update();
    virtual void _log(XBot::MatLogger2::Ptr logger);
    
    const XBot::ModelInterface& _robot;
    std::string _base_link, _distal_link;
    
    Eigen::Affine3d _pose_current, _pose_ref;
    Eigen::Vector6d _pose_error, _vel_error, _vel_ref, _vel_current;
    Eigen::Vector6d _virtual_force, _force_desired;
    Eigen::Vector3d _orientation_error;
    
    
    //Eigen::Vector6d _jodoqdot, _acc_ref;
    //Eigen::Matrix6d _CartesianInertia;
    
    Eigen::Matrix6d _Kp, _Kd;
    
    AffineHelper _wrench;
    AffineHelper _cartesian_task;
    Eigen::Matrix6d I;
    
    
  };
  
} 
} 
}

#endif

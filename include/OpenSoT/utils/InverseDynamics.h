#ifndef _OPENSOT_INVERSE_DYNAMICS_H_
#define _OPENSOT_INVERSE_DYNAMICS_H_

#include <OpenSoT/utils/Affine.h>
#include <XBotInterface/ModelInterface.h>
#include <OpenSoT/utils/AutoStack.h>

namespace OpenSoT {
namespace utils{

/**
 * @brief The InverseDynamics class is a utility to help the computation of wholebody inverse dynamics controllers with contacts
 */
class InverseDynamics{

public:
    typedef std::shared_ptr<InverseDynamics> Ptr;

    /**
     * @brief InverseDynamics constructor
     * @param links_in_contact a vector of string each one representing a contact
     * @param model
     */
    InverseDynamics(const std::vector<std::string> links_in_contact,
                    XBot::ModelInterface& model);

    /**
     * @brief getJointsAccelerationAffine return the Affine related to the joint accelerations
     * @return _qddot
     */
    const AffineHelper& getJointsAccelerationAffine() const;

    /**
     * @brief getContactsWrenchAffine return a vector of Affines each related to a contact wrench
     * @return _contacts_wrench
     */
    const std::vector<AffineHelper>& getContactsWrenchAffine() const;

    /**
     * @brief getSerializer return a pointer to the object which contains all the variables of the Inverse Dynamics
     * @return
     */
    const std::shared_ptr<OpenSoT::OptvarHelper> getSerializer() const;

    /**
     * @brief computedTorque given the solution from the solver which contains all the optimization variables, returns the torque
     * to send to the robot
     * @param x optimized variables
     * @param tau computed torques
     * @param qddot computed accelerations from solution x
     * @param contact_wrench computed wrenches from solution x
     * @return true if the model is fixed base, false if the base wrench is not 0 (for floating base models)
     */
    bool computedTorque(const Eigen::VectorXd& x, Eigen::VectorXd& tau,
                        Eigen::VectorXd& qddot, std::vector<Eigen::Vector6d>& contact_wrench);
    
    

    /**
     * @brief log internal variables: wrenches, tau and qddot
     * @param logger
     */
    void log(XBot::MatLogger2::Ptr& logger);

private:
    AffineHelper    _qddot;
    std::vector<AffineHelper> _contacts_wrench;
    std::vector<std::string> _links_in_contact;
    XBot::ModelInterface& _model;
    std::shared_ptr<OpenSoT::OptvarHelper> _serializer;

    Eigen::VectorXd _qddot_val;
    std::vector<Eigen::Vector6d> _contacts_wrench_val;
    std::vector<Eigen::MatrixXd> _Jc;
    Eigen::VectorXd _tau_val;

};

}
}

#endif

#ifndef __CONSTRAINT_ACCELERATION_TORQUE_LIMITS_H__
#define __CONSTRAINT_ACCELERATION_TORQUE_LIMITS_H__

#include <OpenSoT/Constraint.h>
#include <OpenSoT/utils/Affine.h>
#include <xbot2_interface/xbotinterface2.h>
#include <OpenSoT/constraints/GenericConstraint.h>

namespace OpenSoT { namespace constraints { namespace acceleration {
/**
 * @brief The TorqueLimits class implements torque limits in inverse dynamics:
 *
 *      -tau_lims - h <= Mqddot - Jc'Fc <= tau_lims + h
 *
 *  NOTE: torque limits contains as well limits on floating base torque (that should be zeros)
 */
class TorqueLimits : public Constraint<Eigen::MatrixXd, Eigen::VectorXd> {
public:
    typedef std::shared_ptr<TorqueLimits> Ptr;

    /**
     * @brief TorqueLimits
     * @param robot
     * @param qddot
     * @param wrenches
     * @param contact_links
     * @param torque_limits
     */
    TorqueLimits(const XBot::ModelInterface& robot,
                 const AffineHelper& qddot,
                 const std::vector<AffineHelper>& wrenches,
                 const std::vector<std::string>& contact_links,
                 const Eigen::VectorXd& torque_limits);

    void update(const Eigen::VectorXd& x);

    /**
     * @brief setTorqueLimits to set new torque limits
     * @param torque_limits
     */
    void setTorqueLimits(const Eigen::VectorXd& torque_limits);

    /**
     * @brief enableContact for task computation
     * @param contact_link
     * @return false if the contact is not in the contacts vector
     */
    bool enableContact(const std::string& contact_link);

    /**
     * @brief disableContact for task computation
     * @param contact_link
     * @return false if the contact is not in the contacts vector
     */
    bool disableContact(const std::string& contact_link);

    /**
     * @brief getEnabledContacts
     * @return vector of booleans indicating active contact (in the same order as in contact_links)
     */
    const std::vector<bool>& getEnabledContacts() const;

private:
    const XBot::ModelInterface& _robot;

    AffineHelper _qddot;
    std::vector<AffineHelper> _wrenches;
    std::vector<std::string> _contact_links;
    AffineHelper _dyn_constraint;

    std::vector<bool> _enabled_contacts;

    Eigen::VectorXd _torque_limits;

    Eigen::VectorXd _h;
    Eigen::MatrixXd _B, _Jtmp;
};
}
}
}

#endif

#include <OpenSoT/utils/InverseDynamics.h>
#include <memory>

using namespace OpenSoT::utils;

InverseDynamics::InverseDynamics(const std::vector<std::string> links_in_contact,
                                 XBot::ModelInterface& model):
    _links_in_contact(links_in_contact),
    _model(model)
{
    /// CREATION OF THE OPTIMIZATION VARIABLES
    OpenSoT::OptvarHelper::VariableVector variable_name_dims;
    variable_name_dims.emplace_back("qddot", _model.getJointNum());

    for(unsigned int i = 0; i < _links_in_contact.size(); ++i)
        variable_name_dims.emplace_back(_links_in_contact[i], 6);

    _serializer = std::make_shared<OpenSoT::OptvarHelper>(variable_name_dims);

    _qddot = _serializer->getVariable("qddot");
    _qddot_val.setZero(_model.getJointNum());
    _tau_val = _qddot_val;

    Eigen::MatrixXd J;
    Eigen::Vector6d w; w.setZero();
    for(unsigned int i = 0; i < _links_in_contact.size(); ++i){
        _contacts_wrench.push_back(_serializer->getVariable(_links_in_contact[i]));

        _model.getJacobian(_links_in_contact[i], J);
        _Jc.push_back(J);

        _contacts_wrench_val.push_back(w);
    }
}

bool InverseDynamics::computedTorque(const Eigen::VectorXd& x, Eigen::VectorXd& tau,
                                     Eigen::VectorXd& qddot, std::vector<Eigen::Vector6d>& contact_wrench)
{
    if(x.size() != _serializer->getSize())
    {
        XBot::Logger::error("x.size() != _serializer->getSize()\n");
        return false;
    }

    _qddot.getValue(x, _qddot_val);
    _model.setJointAcceleration(_qddot_val);
    _model.update();
    _model.computeInverseDynamics(_tau_val);

    for(unsigned int i = 0; i < _links_in_contact.size(); ++i){
        _contacts_wrench[i].getValue(x, _contacts_wrench_val[i]);

        _model.getJacobian(_links_in_contact[i], _Jc[i]);

        _tau_val -= _Jc[i].transpose()*_contacts_wrench_val[i];
    }

    tau = _tau_val;
    qddot = _qddot_val;
    contact_wrench = _contacts_wrench_val;

    if(_model.isFloatingBase())
    {
        for(unsigned int i = 0; i < 6; ++i)
        {
            if(fabs(tau[i]) > 10e-3){
                XBot::Logger::error("Floating Base Wrench is not 0!");
                return false;}
        }
    }

    return true;
}


const OpenSoT::AffineHelper& InverseDynamics::getJointsAccelerationAffine() const
{
    return _qddot;
}

const std::vector<OpenSoT::AffineHelper>& InverseDynamics::getContactsWrenchAffine() const
{
    return _contacts_wrench;
}

const std::shared_ptr<OpenSoT::OptvarHelper> InverseDynamics::getSerializer() const
{
    return _serializer;
}

void InverseDynamics::log(XBot::MatLogger2::Ptr& logger)
{
    logger->add("qddot", _qddot_val);

    for(unsigned int i = 0; i < _contacts_wrench_val.size(); ++i)
        logger->add(_links_in_contact[i]+"_wrench", _contacts_wrench_val[i]);

    logger->add("tau", _tau_val);
}

#include <OpenSoT/utils/InverseDynamics.h>
#include <memory>

using namespace OpenSoT::utils;

InverseDynamics::InverseDynamics(const std::vector<std::string> links_in_contact,
                                 XBot::ModelInterface& model, const CONTACT_MODEL& contact_model):
    _links_in_contact(links_in_contact),
    _model(model)
{
    /// CREATION OF THE OPTIMIZATION VARIABLES
    OpenSoT::OptvarHelper::VariableVector variable_name_dims;
    variable_name_dims.emplace_back("qddot", _model.getNv());

    if(contact_model == CONTACT_MODEL::SURFACE_CONTACT)
    {
        for(unsigned int i = 0; i < _links_in_contact.size(); ++i)
            variable_name_dims.emplace_back(_links_in_contact[i], 6); //here we create a wrench variable
    }
    else if(contact_model == CONTACT_MODEL::POINT_CONTACT)
    {
        for(unsigned int i = 0; i < _links_in_contact.size(); ++i)
            variable_name_dims.emplace_back(_links_in_contact[i], 3); //here we create a force variable
    }
    else
        throw std::runtime_error("Unsupported  contact model");

    _serializer = std::make_shared<OpenSoT::OptvarHelper>(variable_name_dims);

    _qddot = _serializer->getVariable("qddot");
    _qddot_val.setZero(_model.getNv());
    _tau_val = _qddot_val;

    Eigen::MatrixXd J;
    for(unsigned int i = 0; i < _links_in_contact.size(); ++i){
        if(contact_model == CONTACT_MODEL::SURFACE_CONTACT) //in this case we can directly use the serializer
            _contacts_wrench.push_back(_serializer->getVariable(_links_in_contact[i]));
        else if(contact_model == CONTACT_MODEL::POINT_CONTACT)//we create a wrench variable from the force variable
        {
            Eigen::MatrixXd M(6,3);
            M.setZero();
            M.block(0,0,3,3) = Eigen::Matrix3d::Identity();
            Eigen::VectorXd q(6);
            q.setZero();

            OpenSoT::AffineHelper wrench = M*_serializer->getVariable(_links_in_contact[i]) + q;
            _contacts_wrench.push_back(wrench);
        }

        _model.getJacobian(_links_in_contact[i], J);
        _Jc.push_back(J);

        _contacts_wrench_val.push_back(Eigen::Vector6d::Zero());
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

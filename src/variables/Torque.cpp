#include <OpenSoT/variables/Torque.h>


OpenSoT::variables::Torque::Torque(XBot::ModelInterface::Ptr model, 
                                   const OpenSoT::AffineHelper& qddot_var, 
                                   std::vector< std::string > contact_links, 
                                   std::vector< OpenSoT::AffineHelper > force_vars): 
    OpenSoT::AffineHelper(qddot_var.getInputSize(), model->getActuatedJointNum()),
    _model(model),
    _num_contacts(contact_links.size()),
    _contact_links(contact_links),
    _qddot_var(qddot_var),
    _force_vars(force_vars)
{
    if( _qddot_var.getOutputSize() != model->getJointNum() ){
        throw std::runtime_error("_qddot_var.getOutputSize() != model->getJointNum()");
    }
    _S.setZero(model->getActuatedJointNum(), model->getJointNum());
    _S.rightCols(model->getActuatedJointNum()) = Eigen::MatrixXd::Identity(model->getActuatedJointNum(), model->getActuatedJointNum());
    update();
}

void OpenSoT::variables::Torque::update()
{
    
    setZero();
    
    // previous line is equivalent to the following two lines
    // _C.setZero(getSize(), getOptvarHelper().getSize());
    // _d.setZero(getSize());
    
    _model->getInertiaMatrix(_B);
    
     self() = self() + _S*_B*_qddot_var;
    
    // previous line is equivalent to the following two lines
    // _C += _S * _B * _qddot_var.getC();
    // _d += _S * _B * _qddot_var.getd();
    
    
    for(int i = 0; i < _num_contacts; i++){
        
        _model->getJacobian(_contact_links[i], _Jc[i]);

        self() =  self() + (-_S)*_Jc[i]*_force_vars.at(i);
        
        // previous line is equivalent to the following two lines
        // _C -= _S * _Jc[i] * _force_vars.at(i).getC();
        // _d -= _S * _Jc[i] * _force_vars.at(i).getd();
    }
    
    _model->computeNonlinearTerm(_h);

    self() = self() + _S*_h;
    
    // previous line is equivalent to
    // _d += _S * _h;
    
    // if alternative version was used, call setters
    // setC(_C);
    // setd(_d);
    
}
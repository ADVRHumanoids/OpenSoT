#include <OpenSoT/solvers/OSQPBackEnd.h>

using namespace OpenSoT::solvers;

OSQPBackEnd::OSQPBackEnd(const int number_of_variables,
            const int number_of_constraints):
    BackEnd(number_of_variables, number_of_constraints),
    _Apiled(number_of_variables),_lApiled(1), _uApiled(1)
{
    _settings.reset(new OSQPSettings());
    _data.reset(new OSQPData());

    I.setIdentity(number_of_variables, number_of_variables);

    osqp_set_default_settings(_settings.get());
}

bool OSQPBackEnd::solve()
{
    return true;
}

boost::any OSQPBackEnd::getOptions()
{

}

void OSQPBackEnd::setOptions(const boost::any &options)
{

}

bool OSQPBackEnd::initProblem(const Eigen::MatrixXd &H, const Eigen::VectorXd &g,
                                 const Eigen::MatrixXd &A,
                                 const Eigen::VectorXd &lA, const Eigen::VectorXd &uA,
                                 const Eigen::VectorXd &l, const Eigen::VectorXd &u)
{
    _H = H; _g = g; _A = A; _lA = lA; _uA = uA; _l = l; _u = u;

    _Apiled.pile(_A);
    _Apiled.pile(I);

    _lApiled.pile(_lA);
    _lApiled.pile(_l);

    _uApiled.pile(_uA);
    _uApiled.pile(_u);

    if(!(_lApiled.rows() == _uApiled.rows())){
        XBot::Logger::error("lApiled size: %i \n", _lApiled.rows());
        XBot::Logger::error("uApiled size: %i \n", _uApiled.rows());
        assert(_lApiled.rows() == _uApiled.rows());
        return false;}
    if(!(_Apiled.rows() == _lApiled.rows())){
        XBot::Logger::error("Apiled size: %i \n", _Apiled.rows());
        XBot::Logger::error("lApiled size: %i \n", _lApiled.rows());
        assert(_lApiled.rows() == _Apiled.rows());
        return false;}

    _Asp = _Apiled.generate_and_get().sparseView();
    _Psp = _H.sparseView();

    std::cout<<"_Asp: \n"<<_Asp<<std::endl;

    _Asp.makeCompressed();
    _Acsc.reset(csc_matrix(_Asp.rows(), _Asp.cols(), _Asp.nonZeros(), _Asp.valuePtr(), _Asp.innerIndexPtr(), _Asp.outerIndexPtr()));

    _Psp.makeCompressed();
    _Pcsc.reset(csc_matrix(_Psp.rows(), _Psp.cols(), _Psp.nonZeros(), _Psp.valuePtr(), _Psp.innerIndexPtr(), _Psp.outerIndexPtr()));


//    std::cout<<"_Asp.innerIndexPtr():{ ";
//    for (int i=0; i< sizeof(_Asp.innerIndexPtr())/sizeof(_Asp.innerIndexPtr()[0]); i++)
//            std::cout <<_Asp.innerIndexPtr()[i]<<" ";
//    std::cout<<"}"<<std::endl;
//    std::cout<<"_Asp.outerIndexPtr():{ ";
//    for (int i=0; i< sizeof(_Asp.outerIndexPtr())/sizeof(_Asp.outerIndexPtr()[0]); i++)
//            std::cout <<_Asp.outerIndexPtr()[i]<<" ";
//    std::cout<<"}"<<std::endl;



//    std::cout<<"_Acsc->m: "<<_Acsc->m<<std::endl;
//    std::cout<<"_Acsc->n: "<<_Acsc->n<<std::endl;
//    std::cout<<"_Acsc->nzmax: "<<_Acsc->nzmax<<std::endl;
//    std::cout<<"_Acsc->x:{ ";
//    for (int i=0; i< sizeof(_Acsc->x)/sizeof(_Acsc->x[0]); i++)
//            std::cout <<_Acsc->x[i]<<" ";
//    std::cout<<"}"<<std::endl;
//    std::cout<<"_Acsc->i:{ ";
//    for (int i=0; i< sizeof(_Acsc->i)/sizeof(_Acsc->i[0]); i++)
//            std::cout <<_Acsc->i[i]<<" ";
//    std::cout<<"}"<<std::endl;
//    std::cout<<"_Acsc->p:{ ";
//    for (int i=0; i< sizeof(_Acsc->p)/sizeof(_Acsc->p[0]); i++)
//            std::cout <<_Acsc->p[i]<<" ";
//    std::cout<<"}"<<std::endl;

    print_csc_matrix(_Acsc.get(), "_Acsc");
    print_csc_matrix(_Pcsc.get(), "_Pcsc");

    toData();

    _workspace.reset(osqp_setup(_data.get(), _settings.get()));

    bool exitflag = osqp_solve(_workspace.get());

    _solution = Eigen::Map<Eigen::VectorXd>(_workspace->solution->x, _solution.size());

    if(exitflag != 0)
        return false;
    return true;
}


void OSQPBackEnd::toData()
{
    _data->n = _H.rows(); //number of variables
    _data->m = _Apiled.rows(); //number of constraints + bounds

    _data->P = _Pcsc.get(); //Hessian
    _data->A = _Acsc.get(); //Constraints

    _data->q = _g.data(); //Gradient
    _data->l = _lApiled.generate_and_get().data(); //Constraints lower
    _data->u = _uApiled.generate_and_get().data(); //Constraints upper

}

OSQPBackEnd::~OSQPBackEnd()
{

}

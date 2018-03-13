#include <OpenSoT/solvers/OSQPBackEnd.h>
#include <osqp/glob_opts.h>

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
    _settings.get()->verbose = 0;
}

void OSQPBackEnd::setCSCMatrix(csc* a, Eigen::SparseMatrix<double>& A)
{
    a->m = A.rows();
    a->n = A.cols();
    a->nzmax = A.nonZeros();
    a->x = A.valuePtr();
    a->i = A.innerIndexPtr();
    a->p = A.outerIndexPtr();
}

bool OSQPBackEnd::updateTask(const Eigen::MatrixXd &H, const Eigen::VectorXd &g)
{
    if(!(_g.rows() == _H.rows())){
        XBot::Logger::error("g size: %i \n", _g.rows());
        XBot::Logger::error("H size: %i \n", _H.rows());
        assert(_g.rows() == _H.rows());
        return false;}
    if(!(_H.cols() == H.cols())){
        XBot::Logger::error("H cols: %i \n", H.cols());
        XBot::Logger::error("should be: %i \n", _H.cols());
        return false;}

    if(_H.rows() == H.rows())
    {
        _H = H.triangularView<Eigen::Upper>();
        _g = g;

        return true;
    }
    else
        return false;
}

bool OSQPBackEnd::solve()
{
    if(_A.rows() > 0)
    {
        _Apiled.set(_A);
        _lApiled.set(_lA);
        _uApiled.set(_uA);

        if(_l.rows() > 0)
        {
            _Apiled.pile(I);
            _lApiled.pile(_l);
            _uApiled.pile(_u);
        }
    }
    else if(_l.rows() > 0)
    {
        _Apiled.set(I);
        _lApiled.set(_l);
        _uApiled.set(_u);
    }

    _Asp = _Apiled.generate_and_get().sparseView();
    _Psp = _H.sparseView();

    _Asp.makeCompressed();
    _Psp.makeCompressed();

    setCSCMatrix(_Acsc.get(), _Asp);
    setCSCMatrix(_Pcsc.get(), _Psp);

    osqp_update_lin_cost(_workspace.get(), _g.data());
    osqp_update_bounds(_workspace.get(), _lApiled.generate_and_get().data(), _uApiled.generate_and_get().data());
    _workspace.get()->data->A = _Acsc.get();
    _workspace.get()->data->P = _Pcsc.get();


    ///QUESTO è DA RIMUOVERE!!!
    _workspace.reset(osqp_setup(_data.get(), _settings.get()));



    osqp_update_warm_start(_workspace.get(), 1);
    osqp_warm_start(_workspace.get(), _solution.data(), _workspace->solution->y); //We try with both primal/dual initial guess

    bool exitflag = osqp_solve(_workspace.get());

    if(exitflag != 0)
    {
        osqp_warm_start_x(_workspace.get(), _solution.data()); //We try only with primal initial guess

        exitflag = osqp_solve(_workspace.get());

        if(exitflag != 0)
        {
            osqp_update_warm_start(_workspace.get(), 0); //We try without warm start
            exitflag = osqp_solve(_workspace.get());
            if(exitflag != 0)
                return false;
            _solution = Eigen::Map<Eigen::VectorXd>(_workspace->solution->x, _solution.size());
            return true;
        }

        _solution = Eigen::Map<Eigen::VectorXd>(_workspace->solution->x, _solution.size());
        return true;
    }
    _solution = Eigen::Map<Eigen::VectorXd>(_workspace->solution->x, _solution.size());
    return true;
}

boost::any OSQPBackEnd::getOptions()
{
    return _settings;
}

void OSQPBackEnd::setOptions(const boost::any &options)
{
    _settings.reset(new OSQPSettings(boost::any_cast<OSQPSettings>(options)));
    _workspace->settings = _settings.get();
}

bool OSQPBackEnd::initProblem(const Eigen::MatrixXd &H, const Eigen::VectorXd &g,
                                 const Eigen::MatrixXd &A,
                                 const Eigen::VectorXd &lA, const Eigen::VectorXd &uA,
                                 const Eigen::VectorXd &l, const Eigen::VectorXd &u)
{
#ifdef DLONG
    XBot::Logger::error("DLONG option in OSQP should be set to OFF in CMakeLists!");
    return false;
#endif

    _H = H.triangularView<Eigen::Upper>(); _g = g; _A = A; _lA = lA; _uA = uA; _l = l; _u = u;

    if(_A.rows() > 0)
    {
        _Apiled.pile(_A);

        _lApiled.pile(_lA);

        _uApiled.pile(_uA);

    }

    if(_l.rows() > 0)
    {
        _Apiled.pile(I);
        _lApiled.pile(_l);
        _uApiled.pile(_u);
    }

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
    _Psp = _H.sparseView(); //_H.sparseView();

    _Asp.makeCompressed();
    _Acsc.reset(csc_matrix(_Asp.rows(), _Asp.cols(), _Asp.nonZeros(), _Asp.valuePtr(), _Asp.innerIndexPtr(), _Asp.outerIndexPtr()));

    _Psp.makeCompressed();
    _Pcsc.reset(csc_matrix(_Psp.rows(), _Psp.cols(), _Psp.nonZeros(),
                           _Psp.valuePtr(),
                           _Psp.innerIndexPtr(), _Psp.outerIndexPtr()));

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

//void OSQPBackEnd::print_csc_matrix_raw(csc* a, const std::string& name)
//{
//    XBot::Logger::info("%s->m: %i\n",name.c_str(), a->m);
//    XBot::Logger::info("%s->n: %i\n",name.c_str(), a->n);
//    XBot::Logger::info("%s->nzmax: %i\n",name.c_str(), a->nzmax);
//    XBot::Logger::info("%s->nz: %i\n",name.c_str(), a->nz);

//    XBot::Logger::info("%s->x:{",name.c_str());
//    for (int i=0; i< sizeof(a->x)/sizeof(a->x[0]); i++)
//        XBot::Logger::info("%f ",a->x[i]);
//    XBot::Logger::info("}\n");
//    XBot::Logger::info("%s->i:{",name.c_str());
//    for (int i=0; i< sizeof(a->i)/sizeof(a->i[0]); i++)
//        XBot::Logger::info("%f ",a->i[i]);
//    XBot::Logger::info("}\n");
//    XBot::Logger::info("%s->p:{",name.c_str());
//    for (int i=0; i< sizeof(a->p)/sizeof(a->p[0]); i++)
//        XBot::Logger::info("%f ",a->p[i]);
//    XBot::Logger::info("}\n");
//}

OSQPBackEnd::~OSQPBackEnd()
{

}

#include <OpenSoT/solvers/OSQPBackEnd.h>
#include <osqp/glob_opts.h>
#include <exception>
using namespace OpenSoT::solvers;

OSQPBackEnd::OSQPBackEnd(const int number_of_variables,
                         const int number_of_constraints):
    BackEnd(number_of_variables, number_of_constraints)
{
    
    #ifdef DLONG
        throw std::runtime_error("DLONG option in OSQP should be set to OFF in CMakeLists!");
    #endif
    
    _eye.setIdentity(number_of_variables, number_of_variables);
    _P_values.resize(getNumVariables()*(getNumVariables() + 1)/2);
    
    _settings.reset(new OSQPSettings());
    _settings->verbose = 0;
    
    osqp_set_default_settings(_settings.get());
    
    _data.reset(new OSQPData());
    _Pcsc.reset(new csc());
    _Acsc.reset(new csc());
    
    _lb_piled.setConstant(getNumVariables() + getNumConstraints(), -1.0);
    _ub_piled.setConstant(getNumVariables() + getNumConstraints(),  1.0);
 
    __generate_data_struct();
    
    
}

void OSQPBackEnd::__generate_data_struct()
{
    /* Set appropriate sparsity pattern to P (upper triangular) */
    
    Eigen::MatrixXd sp_pattern, ones;
    sp_pattern.setZero(getNumVariables(), getNumVariables());
    ones.setOnes(getNumVariables(), getNumVariables());
    
    sp_pattern.triangularView<Eigen::Upper>() = ones;
    
    _Psparse = sp_pattern.sparseView();
    
    setCSCMatrix(_Pcsc.get(), _Psparse);
    
    
    
    /* Set appropriate sparsity pattern to A (dense + diagonal) */
    ones.setOnes(getNumConstraints(), getNumVariables());
    sp_pattern.setZero(getNumConstraints() + getNumVariables(), getNumVariables());
    sp_pattern.topRows(getNumConstraints()) = ones;
    sp_pattern.bottomRows(getNumVariables()) = Eigen::MatrixXd::Identity(getNumVariables(), getNumVariables());
    
    _Asparse_upper = ones.sparseView();
    _Asparse = sp_pattern.sparseView();
    _Asparse_rowmaj = sp_pattern.sparseView();
    
    setCSCMatrix(_Acsc.get(), _Asparse);
    
    
    /* Fill data */
    
    _data->n = getNumVariables();
    _data->m = getNumConstraints() + getNumVariables();
    _data->l = _lb_piled.data();
    _data->u = _ub_piled.data();
    _data->q = _g.data();
    _data->A = _Acsc.get();
    _data->P = _Pcsc.get();
    
    
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
    bool success = BackEnd::updateTask(H, g);
    
    if(!success)
    {
        return false;
    }
    
    
    int idx = 0;
    for(int c = 0; c < getNumVariables(); c++)
    {
        _P_values.segment(idx, c+1) = H.col(c).head(c+1);
        idx += c+1;
    }
    
    
    return true;
}




bool OSQPBackEnd::updateConstraints(const Eigen::Ref<const Eigen::MatrixXd>& A, 
                                const Eigen::Ref<const Eigen::VectorXd>& lA, 
                                const Eigen::Ref<const Eigen::VectorXd>& uA)
{
    bool success = BackEnd::updateConstraints(A, lA, uA);
    
    if(!success)
    {
        return false;
    }
        
    /* Update values in A upper part (constraints) */
    memcpy(_Asparse_upper.valuePtr(), A.data(), A.size()*sizeof(double));
    _Asparse_rowmaj.topRows(getNumConstraints()) = _Asparse_upper;
    _Asparse = _Asparse_rowmaj;
    
    /* Update constraints bounds */
    _lb_piled.head(getNumConstraints()) = lA;
    _ub_piled.head(getNumConstraints()) = uA;
    
    return true;
}

bool OSQPBackEnd::updateBounds(const Eigen::VectorXd& l, const Eigen::VectorXd& u)
{
    bool success =  OpenSoT::solvers::BackEnd::updateBounds(l, u);
    
    if(!success)
    {
        return false;
    }
    
    _lb_piled.tail(getNumVariables()) = l;
    _ub_piled.tail(getNumVariables()) = u;
    
    return true;
}


bool OSQPBackEnd::solve()
{
    
    
    osqp_update_lin_cost(_workspace.get(), _g.data());
    osqp_update_bounds(_workspace.get(), _lb_piled.data(), _ub_piled.data());
    osqp_update_A(_workspace.get(), _Asparse.valuePtr(), nullptr, _Asparse.nonZeros());
    osqp_update_P(_workspace.get(), _P_values.data(), nullptr, _P_values.size());
    
    
    bool exitflag = osqp_solve(_workspace.get());
    
    _solution = Eigen::Map<Eigen::VectorXd>(_workspace->solution->x, _solution.size());
    
    return exitflag;
    
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
    bool success = true;
    success = updateTask(H, g) && success;
    success = updateConstraints(A, lA, uA) && success;
    success = updateBounds(l, u) && success;
    
    _workspace.reset( osqp_setup(_data.get(), _settings.get()) );
    
    success = solve() && success;
    
    return success;
}


void OSQPBackEnd::toData()
{
    _data->n = _H.rows(); //number of variables
    _data->m = _A.rows() + _data->n; //number of constraints + bounds

    _data->P = _Pcsc.get(); //Hessian
    _data->A = _Acsc.get(); //Constraints

    _data->q = _g.data(); //Gradient
    _data->l = _lb_piled.data(); //Constraints lower
    _data->u = _ub_piled.data(); //Constraints lower

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

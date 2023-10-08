#include <OpenSoT/solvers/qpSWIFTBackEnd.h>

using namespace OpenSoT::solvers;

#define BASE_REGULARISATION 2.22E-13 //previous 1E-12


/* Define factories for dynamic loading */
extern "C" BackEnd * create_instance(const int number_of_variables,
                               const int number_of_constraints,
                               OpenSoT::HessianType hessian_type, const double eps_regularisation)
{
    return new qpSWIFTBackEnd(number_of_variables, number_of_constraints, eps_regularisation);
}

extern "C" void destroy_instance( BackEnd * instance )
{
    delete instance;
}

/**
 * @brief qpSWIFTBackEnd::qpSWIFTBackEnd
 * @param number_of_variables
 * @param number_of_constraints
 * @param eps_regularisation
 * @note the solver can not handle constraint matrices with rows of zeros
 */
qpSWIFTBackEnd::qpSWIFTBackEnd(const int number_of_variables,
                               const int number_of_constraints,
                               const double eps_regularisation):
    BackEnd(number_of_variables, number_of_constraints),
    _AA(number_of_variables),
    _b(1),
    _G(number_of_variables),
    _h(1),
    _eps_regularisation(eps_regularisation*BASE_REGULARISATION) //TO HAVE COMPATIBILITY WITH THE QPOASES ONE!
{
    _I.resize(number_of_variables, number_of_variables);
    _I.setIdentity();
}

qpSWIFTBackEnd::~qpSWIFTBackEnd()
{

}

void qpSWIFTBackEnd::createDataStructure(const Eigen::MatrixXd &A, const Eigen::VectorXd &lA, const Eigen::VectorXd &uA,
                                         const Eigen::VectorXd &l, const Eigen::VectorXd &u)
{
    _equality_constraint_indices.clear();
    _equality_bounds_indices.clear();
    _inequality_constraint_indices.clear();
    _inequality_bounds_indices.clear();

    //1) Creates equality and inequality indices
    for(unsigned int i = 0; i < lA.size(); ++i)
    {
        if(std::fabs(lA[i]-uA[i]) <= std::numeric_limits<double>::epsilon())
            _equality_constraint_indices.push_back(i);
        else
            _inequality_constraint_indices.push_back(i);
    }

    for(unsigned int i = 0; i < l.size(); ++i)
    {
        if(std::fabs(l[i]-u[i]) <= std::numeric_limits<double>::epsilon())
            _equality_bounds_indices.push_back(i);
        else
            _inequality_bounds_indices.push_back(i);
    }

    //2) Creates equality and inequality matrices
    for(const auto& i : _equality_constraint_indices)
    {
        _AA.pile(A.row(i));
        _b.pile(lA.segment(i, 1));
    }
    for(const auto& i : _equality_bounds_indices)
    {
        _AA.pile(_I.row(i));
        _b.pile(_l.segment(i, 1));
    }

    for(const auto& i : _inequality_constraint_indices)
    {
        _G.pile(A.row(i));
        _h.pile(uA.segment(i, 1));

        _G.pile(-A.row(i));
        _h.pile(-lA.segment(i, 1));
    }
    for(const auto& i : _inequality_bounds_indices)
    {
        _G.pile(_I.row(i));
        _h.pile(u.segment(i, 1));

        _G.pile(-_I.row(i));
        _h.pile(-l.segment(i, 1));

    }
}

bool qpSWIFTBackEnd::initProblem(const Eigen::MatrixXd &H,
                                 const Eigen::VectorXd &g, const Eigen::MatrixXd &A,
                                 const Eigen::VectorXd &lA, const Eigen::VectorXd &uA,
                                 const Eigen::VectorXd &l, const Eigen::VectorXd &u)
{
    _A = A;
    _lA = lA;
    _uA = uA;
    _l = l;
    _u = u;
    _g = g;
    _H = H;

    for(unsigned int i = 0; i < H.cols(); ++i)
        _H(i, i) += _eps_regularisation;

    createDataStructure(_A, _lA, _uA, _l, _u);

    _qp.reset(QP_SETUP_dense(_H.cols(), _h.rows(), _b.rows(),
                             _H.data(),
                             _AA.generate_and_get().data(),
                             _G.generate_and_get().data(),
                             _g.data(),
                             _h.generate_and_get().data(),
                             _b.generate_and_get().data(),
                             NULL, COLUMN_MAJOR_ORDERING));

    if(_qp)
        return solve();
    return false;
}

bool qpSWIFTBackEnd::solve()
{

    _AA.reset();
    _b.reset();
    _h.reset();
    _G.reset();

    createDataStructure(_A, _lA, _uA, _l, _u);



    _qp.reset(QP_SETUP_dense(_H.cols(), _h.rows(), _b.rows(),
                             _H.data(),
                             _AA.generate_and_get().data(),
                             _G.generate_and_get().data(),
                             _g.data(),
                             _h.generate_and_get().data(),
                             _b.generate_and_get().data(),
                             NULL, COLUMN_MAJOR_ORDERING));

    if(_user_options)
        _qp->options = _user_options.get();

    qp_int exit_code = QP_SOLVE(_qp.get());
    if(exit_code == QP_MAXIT)
        return false;
    if(exit_code == QP_FATAL)
        return false;
    if(exit_code == QP_KKTFAIL)
        return false;

    for(unsigned int i = 0; i < this->_number_of_variables; ++i)
        _solution[i] = _qp->x[i];

    //QP_CLEANUP_dense(_qp.get());

    return true;
}

boost::any qpSWIFTBackEnd::getOptions()
{
    return _qp->options;
}

void qpSWIFTBackEnd::setOptions(const boost::any& options)
{
    _user_options = std::make_shared<settings>();
    settings* options_ptr = boost::any_cast<settings*>(options);

    _user_options->maxit = options_ptr->maxit;
    _user_options->reltol = options_ptr->reltol;
    _user_options->abstol = options_ptr->abstol;
    _user_options->sigma = options_ptr->sigma;
    _user_options->verbose = options_ptr->verbose;
}

bool qpSWIFTBackEnd::updateTask(const Eigen::MatrixXd& H, const Eigen::VectorXd& g)
{
    if(_H.rows() != H.rows() || _H.cols() != H.cols())
        return false;
    if(g.size() != _g.size())
        return false;

    _g = g;
    _H = H;

    for(unsigned int i = 0; i < H.cols(); ++i)
        _H(i, i) += _eps_regularisation;

    return true;
}

bool qpSWIFTBackEnd::updateConstraints(const Eigen::Ref<const Eigen::MatrixXd>& A,
                            const Eigen::Ref<const Eigen::VectorXd>& lA,
                            const Eigen::Ref<const Eigen::VectorXd>& uA)
{
    if(_A.rows() != A.rows() || _A.cols() != A.cols())
        return false;
    if(lA.size() != _lA.size())
        return false;
    if(uA.size() != _uA.size())
        return false;

    _A = A;
    _lA = lA;
    _uA = uA;

    return true;
}

bool qpSWIFTBackEnd::updateBounds(const Eigen::VectorXd& l, const Eigen::VectorXd& u)
{
    if(l.size() != _l.size())
        return false;
    if(u.size() != _u.size())
        return false;

    _l = l;
    _u = u;

    return true;
}

double qpSWIFTBackEnd::getObjective()
{
    return _qp->stats->fval;
}

#include <OpenSoT/solvers/proxQPBackEnd.h>

using namespace OpenSoT::solvers;
using namespace proxsuite;
using namespace proxsuite::proxqp;

#define BASE_REGULARISATION 2.22E-13 //previous 1E-12

/* Define factories for dynamic loading */
extern "C" BackEnd * create_instance(const int number_of_variables,
                               const int number_of_constraints,
                               OpenSoT::HessianType hessian_type, const double eps_regularisation)
{
    return new proxQPBackEnd(number_of_variables, number_of_constraints, eps_regularisation);
}

extern "C" void destroy_instance( BackEnd * instance )
{
    delete instance;
}

proxQPBackEnd::proxQPBackEnd(const int number_of_variables,
                             const int number_of_constraints,
                             const double eps_regularisation):
    BackEnd(number_of_variables, number_of_constraints),
    _AA(number_of_variables),
    _b(1),
    _G(number_of_variables),
    _ll(1),
    _uu(1),
    _eps_regularisation(eps_regularisation*BASE_REGULARISATION) //TO HAVE COMPATIBILITY WITH THE QPOASES ONE!
{
    _I.resize(number_of_variables, number_of_variables);
    _I.setIdentity();
}

proxQPBackEnd::~proxQPBackEnd()
{

}

bool proxQPBackEnd::initProblem(const Eigen::MatrixXd &H,
                                 const Eigen::VectorXd &g, const Eigen::MatrixXd &A,
                                 const Eigen::VectorXd &lA, const Eigen::VectorXd &uA,
                                 const Eigen::VectorXd &l, const Eigen::VectorXd &u)
{
    //1) Creates equality and inequality indices
    ///TODO: what if we have equality bounds???
    for(unsigned int i = 0; i < lA.size(); ++i)
    {
        if(std::fabs(lA[i]-uA[i]) <= std::numeric_limits<double>::epsilon())
            _equality_indices.push_back(i);
        else
            _inequality_indices.push_back(i);
    }

    //2) Creates equality and inequality matrices
    _A = A;
    _lA = lA;
    _uA = uA;
    _l = l;
    _u = u;

    for(const auto& i : _equality_indices)
    {
        _AA.pile(_A.row(i));
        _b.pile(_lA.segment(i, 1));
    }

    for(const auto& i : _inequality_indices)
    {
        _G.pile(_A.row(i));
        _uu.pile(_uA.segment(i, 1));
        _ll.pile(_lA.segment(i, 1));
    }
    _G.pile(_I);
    _uu.pile(_u);
    _ll.pile(_l);

    //3) popolate qp structure
    _g = g;
    _H = H;

    for(unsigned int i = 0; i < H.cols(); ++i)
        _H(i, i) += _eps_regularisation;


    //4) init
    _QP = std::make_shared<dense::QP<double>>(H.rows(), _AA.rows(), _G.rows());
    _QP->init(_H, _g, _AA.generate_and_get(), _b.generate_and_get(), _G.generate_and_get(), _uu.generate_and_get(), _ll.generate_and_get());
    _QP->solve();
    _QP->settings.initial_guess =
        InitialGuessStatus::WARM_START_WITH_PREVIOUS_RESULT;

    return true;
}

bool proxQPBackEnd::solve()
{
    _AA.reset();
    _b.reset();
    _ll.reset();
    _uu.reset();
    _G.reset();

    for(const auto& i : _equality_indices)
    {
        _AA.pile(_A.row(i));
        _b.pile(_lA.segment(i, 1));
    }

    for(const auto& i : _inequality_indices)
    {
        _G.pile(_A.row(i));
        _uu.pile(_uA.segment(i, 1));
        _ll.pile(_lA.segment(i, 1));
    }
    _G.pile(_I);
    _uu.pile(_u);
    _ll.pile(_l);

    _QP->update(_H, _g, _AA.generate_and_get(), _b.generate_and_get(), _G.generate_and_get(), _uu.generate_and_get(), _ll.generate_and_get());
    _QP->solve();

    _solution = _QP->results.x;

    return true;
}

bool proxQPBackEnd::updateTask(const Eigen::MatrixXd& H, const Eigen::VectorXd& g)
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

bool proxQPBackEnd::updateConstraints(const Eigen::Ref<const Eigen::MatrixXd>& A,
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

bool proxQPBackEnd::updateBounds(const Eigen::VectorXd& l, const Eigen::VectorXd& u)
{
    if(l.size() != _l.size())
        return false;
    if(u.size() != _u.size())
        return false;

    _l = l;
    _u = u;

    return true;
}

void proxQPBackEnd::setOptions(const boost::any& options)
{
    ///TODO
}

boost::any proxQPBackEnd::getOptions()
{
    return NULL;
}

double proxQPBackEnd::getObjective()
{
    ///TODO
    return 0.;
}





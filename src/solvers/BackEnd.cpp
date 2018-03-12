#include <OpenSoT/solvers/BackEnd.h>

using namespace OpenSoT::solvers;

BackEnd::BackEnd(const int number_of_variables, const int number_of_constraints)
{
    _solution.setZero(number_of_variables);

    _H.setZero(number_of_variables,number_of_variables);
    _g.setZero(number_of_variables);
    _A.setZero(number_of_constraints,number_of_variables);
    _lA.setZero(number_of_constraints);
    _uA.setZero(number_of_constraints);
    _l.setZero(number_of_variables);
    _u.setZero(number_of_variables);
}

bool BackEnd::updateConstraints(const Eigen::Ref<const Eigen::MatrixXd>& A,
                                const Eigen::Ref<const Eigen::VectorXd> &lA,
                                const Eigen::Ref<const Eigen::VectorXd> &uA)
{
    if(!(A.cols() == _H.cols())){
        XBot::Logger::error("A cols: %i \n", A.cols());
        XBot::Logger::error("should be: %i \n", _H.cols());
        return false;}
    if(!(lA.rows() == A.rows())){
        XBot::Logger::error("lA size: %i \n", lA.rows());
        XBot::Logger::error("A rows: %i \n", A.rows());
        return false;}
    if(!(lA.rows() == uA.rows())){
        XBot::Logger::error("lA size: %i \n", lA.rows());
        XBot::Logger::error("uA size: %i \n", uA.rows());
        return false;}

    if(A.rows() == _A.rows())
    {
        _A = A;
        _lA = lA;
        _uA = uA;
        return true;
    }
    else
        return false;
}

bool BackEnd::updateTask(const Eigen::MatrixXd &H, const Eigen::VectorXd &g)
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
        _H = H;
        _g = g;

        return true;
    }
    else
        return false;
}

bool BackEnd::updateBounds(const Eigen::VectorXd &l, const Eigen::VectorXd &u)
{
    if(!(l.rows() == _l.rows())){
        XBot::Logger::error("l size: %i \n", l.rows());
        XBot::Logger::error("should be: %i \n", _l.rows());
        return false;}
    if(!(u.rows() == _u.rows())){
        XBot::Logger::error("u size: %i \n", u.rows());
        XBot::Logger::error("should be: %i \n", _u.rows());
        return false;}
    if(!(l.rows() == u.rows())){
        XBot::Logger::error("l size: %i \n", l.rows());
        XBot::Logger::error("u size: %i \n", u.rows());
        return false;}

    _l = l;
    _u = u;

    return true;
}

bool BackEnd::updateProblem(const Eigen::MatrixXd &H, const Eigen::VectorXd &g,
                            const Eigen::MatrixXd &A, const Eigen::VectorXd &lA, const Eigen::VectorXd &uA,
                            const Eigen::VectorXd &l, const Eigen::VectorXd &u)
{
    bool success = true;
    success = success && updateBounds(l, u);
    success = success && updateConstraints(A, lA, uA);
    success = success && updateTask(H, g);
    return success;
}

BackEnd::~BackEnd()
{

}

void BackEnd::log(XBot::MatLogger::Ptr logger, int i)
{
    logger->add("H_"+std::to_string(i), _H);
    logger->add("g_"+std::to_string(i), _g);
    if(_A.rows() > 0 && _A.cols() > 0)
        logger->add("A_"+std::to_string(i), _A);
    if(_lA.size() > 0)
        logger->add("lA_"+std::to_string(i), _lA);
    if(_uA.size() > 0)
        logger->add("uA_"+std::to_string(i), _uA);
    if(_l.size() > 0)
        logger->add("l_"+std::to_string(i), _l);
    if(_u.size() > 0)
        logger->add("u_"+std::to_string(i), _u);
    if(_solution.size() > 0)
        logger->add("solution_"+std::to_string(i), _solution);

    _log(logger, i);
}

void BackEnd::printProblemInformation(const int problem_number, const std::string& problem_id,
                                      const std::string& constraints_id, const std::string& bounds_id)
{
    std::cout<<std::endl;
    if(problem_number == -1)
        XBot::Logger::info("PROBLEM ID: %s \n", problem_id);
    else
        XBot::Logger::info("PROBLEM %i ID: %s \n", problem_number, problem_id);

    XBot::Logger::info("CONSTRAINTS ID: %s \n", constraints_id);
    XBot::Logger::info("    # OF CONSTRAINTS: %i \n", _A.rows());

    XBot::Logger::info("BOUNDS ID: %s \n", bounds_id);
    XBot::Logger::info("    # OF BOUNDS: %i \n", _l.size());

    XBot::Logger::info("# OF VARIABLES: %i \n", _H.rows());

    _printProblemInformation();

    XBot::Logger::info("\n");
}

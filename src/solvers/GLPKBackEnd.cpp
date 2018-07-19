#include <OpenSoT/solvers/GLPKBackEnd.h>
#include <XBotInterface/SoLib.h>
#include <boost/make_shared.hpp>


using namespace OpenSoT::solvers;

/* Define factories for dynamic loading */
extern "C" BackEnd * create_instance(const int number_of_variables, const int number_of_constraints,
                               OpenSoT::HessianType hessian_type, const double eps_regularisation)
{
    return new GLPKBackEnd(number_of_variables, number_of_constraints, eps_regularisation);
}

extern "C" void destroy_instance( BackEnd * instance )
{
    delete instance;
}

GLPKBackEnd::GLPKBackEnd(const int number_of_variables, const int number_of_constraints, const double eps_regularisation):
    BackEnd(number_of_variables, number_of_constraints),
    _rows((number_of_constraints*number_of_variables)+1),
    _cols(_rows.size()),
    _a(_rows.size())
{
    _mip = glp_create_prob();
    glp_set_prob_name(_mip, "mip_problem");

    //at this stage the BE creates a LP with no extra variables
    glp_add_rows(_mip, number_of_constraints);
    glp_add_cols(_mip, number_of_variables);
}

GLPKBackEnd::~GLPKBackEnd()
{
    glp_delete_prob(_mip);
}

bool GLPKBackEnd::solve()
{
    createsVectorsFromConstraintsMatrix();
    roundBounds();

    //SETTING BOUNDS & COST FUNCTION
    for(unsigned int i = 0; i < _l.rows(); ++i){
        glp_set_col_bnds(_mip, i+1, checkConstrType(_u[i], _l[i]), _l[i], _u[i]);
        glp_set_obj_coef(_mip, i+1, _g[i]);}
    //SETTING CONSTRAINTS
    for(unsigned int i = 0; i < _A.rows(); ++i)
        glp_set_row_bnds(_mip, i+1, checkConstrType(_uA[i], _lA[i]), _lA[i], _uA[i]);
    //SETTING CONSTRAINT MATRIX
    glp_load_matrix(_mip, _A.rows()*_A.cols(), _rows.data(), _cols.data(), _a.data());

    _param.presolve = GLP_ON;
    if(!(glp_intopt(_mip, &_param) == 0))
    {
        XBot::Logger::error("GLPK return false in solve!");
        return false;
    }

    for(unsigned int i = 0; i < _solution.size(); ++i)
        _solution[i] = glp_mip_col_val(_mip, i+1);
    return true;
}

bool GLPKBackEnd::initProblem(const Eigen::MatrixXd &H, const Eigen::VectorXd &g,
                         const Eigen::MatrixXd &A, const Eigen::VectorXd &lA, const Eigen::VectorXd &uA,
                         const Eigen::VectorXd &l, const Eigen::VectorXd &u)
{
    _H = H; _g = g; _A = A; _lA = lA; _uA = uA; _l = l; _u = u;

    createsVectorsFromConstraintsMatrix();
    roundBounds();


    if(!(_l.rows() == _u.rows())){
        XBot::Logger::error("l size: %i \n", _l.rows());
        XBot::Logger::error("u size: %i \n", _u.rows());
        assert(_l.rows() == _u.rows());
        return false;}
    if(!(_lA.rows() == _A.rows())){
        XBot::Logger::error("lA size: %i \n", _lA.rows());
        XBot::Logger::error("A rows: %i \n", _A.rows());
        assert(_lA.rows() == _A.rows());
        return false;}
    if(!(_lA.rows() == _uA.rows())){
        XBot::Logger::error("lA size: %i \n", _lA.rows());
        XBot::Logger::error("uA size: %i \n", _uA.rows());
        assert(_lA.rows() == _uA.rows());
        return false;}

    //SETTING BOUNDS & COST FUNCTION
    for(unsigned int i = 0; i < _l.rows(); ++i){
        glp_set_col_bnds(_mip, i+1, checkConstrType(_u[i], _l[i]), _l[i], _u[i]);
        glp_set_obj_coef(_mip, i+1, _g[i]);}
    //SETTING CONSTRAINTS
    for(unsigned int i = 0; i < _A.rows(); ++i)
        glp_set_row_bnds(_mip, i+1, checkConstrType(_uA[i], _lA[i]), _lA[i], _uA[i]);
    //SETTING CONSTRAINT MATRIX
    glp_load_matrix(_mip, _A.rows()*_A.cols(), _rows.data(), _cols.data(), _a.data());


    glp_init_iocp(&_param);
    _param.presolve = GLP_ON;
    _param.msg_lev = GLP_MSG_ALL;

    if(!(glp_intopt(_mip, &_param) == 0))
    {
        XBot::Logger::error("GLPK return false in initProblem!");
        return false;
    }

    for(unsigned int i = 0; i < _solution.size(); ++i)
        _solution[i] = glp_mip_col_val(_mip, i+1);
    return true;
}

//IN EIGEN COLUMN MAJOIR!
void GLPKBackEnd::createsVectorsFromConstraintsMatrix()
{
    int idx = 0;
    for(unsigned int i = 0; i < _A.cols(); ++i)
    {
        for(unsigned int j = 0; j < _A.rows(); ++j)
        {
            _cols[idx+1] = i+1;
            _rows[idx+1] = j+1;
            idx++;
        }
    }

    _a.setZero(_a.size());
    _a.tail(_A.rows()*_A.cols()) = Eigen::Map<Eigen::VectorXd>(_A.data(), _A.rows()*_A.cols());
}

int GLPKBackEnd::checkConstrType(const double u, const double l)
{
    if(fabs(u-l) <= 2.2e-16)
        return GLP_FX;

    if(l <= -1e20 && u >= 1e20)
    return GLP_FR;

    if(l <= -1e20 && u < 1e20)
        return GLP_UP;

    if(l > -1e20 && u >= 1e20)
        return GLP_LO;

    return GLP_DB;
}


Eigen::VectorXd GLPKBackEnd::getGLPKUpperBounds()
{
    Eigen::VectorXd u(_u.size());
    for(unsigned int i = 0; i < u.size(); ++i)
        u[i] = glp_get_col_ub(_mip, i+1);
    return u;
}

Eigen::VectorXd GLPKBackEnd::getGLPKLowerBounds()
{
    Eigen::VectorXd l(_l.size());
    for(unsigned int i = 0; i < l.size(); ++i)
        l[i] = glp_get_col_lb(_mip, i+1);
    return l;
}

Eigen::VectorXd GLPKBackEnd::getGLPKUpperConstraints()
{
    Eigen::VectorXd uA(_uA.size());
    for(unsigned int i = 0; i < uA.size(); ++i)
        uA[i] = glp_get_row_ub(_mip, i+1);
    return uA;
}

Eigen::VectorXd GLPKBackEnd::getGLPKLowerConstraints()
{
    Eigen::VectorXd lA(_lA.size());
    for(unsigned int i = 0; i < lA.size(); ++i)
        lA[i] = glp_get_row_lb(_mip, i+1);
    return lA;
}

Eigen::MatrixXd GLPKBackEnd::getGLPKConstraintMatrix()
{
    Eigen::MatrixXd A(_A.rows(),_A.cols());

    for(unsigned int i = 0; i < _A.rows(); ++i)
    {
        double val[_A.cols()];
        int id[_A.cols()];

        glp_get_mat_row(_mip, i+1, id, val);

        Eigen::VectorXd row(_A.cols());
        for(unsigned int j = 0; j < _A.cols(); ++j)
            row[id[j+1]-1] = val[j+1];

        A.row(i) = row;

    }

    return A;
}

void GLPKBackEnd::setOptions(const boost::any& options)
{
    _opt = boost::any_cast<GLPKBackEndOptions>(options);

    if(_opt.param)
        _param = *(_opt.param);

    if(!(_opt.var_id_kind_.empty()))
        _var_id_kind = _opt.var_id_kind_;

    for(unsigned int i = 0; i < _var_id_kind.size(); ++i)
        glp_set_col_kind(_mip, _var_id_kind[i].first+1, _var_id_kind[i].second);


}

void GLPKBackEnd::roundBounds()
{
   if(!_opt.var_id_kind_.empty())
   {
       for(unsigned int i = 0; i < _var_id_kind.size(); ++i)
       {
           if(_var_id_kind[i].second != GLP_CV)
           {
               int id = _var_id_kind[i].first;
               if(_opt.ROUND_BOUNDS == 1)
               {
                   _l[id] = ceil(_l[id]);
                   _u[id] = ceil(_u[id]);
               }
               else if(_opt.ROUND_BOUNDS == -1)
               {
                   _l[id] = floor(_l[id]);
                   _u[id] = floor(_u[id]);
               }
           }
       }
   }
}


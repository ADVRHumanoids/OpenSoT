#ifndef _WB_SOT_SOLVERS_GLPK_BE_H_
#define _WB_SOT_SOLVERS_GLPK_BE_H_

#include <OpenSoT/solvers/BackEnd.h>
#include <glpk.h>

namespace OpenSoT{
namespace solvers{

class GLPKBackEnd:  public BackEnd{
public:
    typedef int index; //start from 0!
    typedef int kind; //GLP_CV, GLP_IV, GLP_BV
    typedef std::pair<index, kind> var_id_kind;

    struct GLPKBackEndOptions
    {
        /**
         * @brief ROUND_BOUNDS
         *  0: bounds are not rounded
         *  1: bounds are rounded to nearest sup integer
         * -1: bounds are rounded to nearest inf integer
         */
        int ROUND_BOUNDS = 0;

        std::vector<var_id_kind> var_id_kind_;
        std::shared_ptr<glp_iocp> param;

    };

    GLPKBackEnd(const int number_of_variables,
            const int number_of_constraints,
            const double eps_regularisation = 0.0);

    ~GLPKBackEnd();

    /**
     * H NOT USED!
     * g Linear cost function: g'x
     * A Linear Constraints Ax
     * lA/uA constraints
     * l/u bounds
     **/
    bool initProblem(const Eigen::MatrixXd &H, const Eigen::VectorXd &g,
                             const Eigen::MatrixXd &A,
                             const Eigen::VectorXd &lA, const Eigen::VectorXd &uA,
                             const Eigen::VectorXd &l, const Eigen::VectorXd &u);

    bool solve();
    void setOptions(const boost::any& options);
    boost::any getOptions(){return _opt;}
    double getObjective();

    Eigen::VectorXd getGLPKUpperBounds();
    Eigen::VectorXd getGLPKLowerBounds();
    Eigen::VectorXd getGLPKUpperConstraints();
    Eigen::VectorXd getGLPKLowerConstraints();
    Eigen::MatrixXd getGLPKConstraintMatrix();

    /**
     * @brief writeLP call glp_write_lp to have text file of the problem
     * @return check glpk.h
     */
    int writeLP();

private:
    glp_prob* _mip;
    glp_iocp _param;
    glp_smcp _param_simplex;

    Eigen::VectorXi _rows;
    Eigen::VectorXi _cols;
    Eigen::VectorXd _a;

    void createsVectorsFromConstraintsMatrix();

    int checkConstrType(const double u, const double l);

    void roundBounds();

    GLPKBackEndOptions _opt;

    std::vector<var_id_kind> _var_id_kind;

    /**
     * @brief printErrorOutput prints outputs error meaning (taken directly from the documentation of GLPK)
     * @param out the error code
     */
    void printErrorOutput(const int out);

};

}
}

#endif

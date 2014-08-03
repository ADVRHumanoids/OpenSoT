#ifndef _WB_SOT_SOLVERS_QP_OASES_H_
#define _WB_SOT_SOLVERS_QP_OASES_H_

#include <qpOASES.hpp>
#include <wb_sot/Solver.h>
#include <vector>

namespace wb_sot{
    namespace solvers{

    template <class Matrix_type, class Vector_type>
    class QPOases: public Solver<Matrix_type, Vector_type>
    {
    public:
        QPOases():
            _bounds(0),
            _constraints(0)
        {}

        ~QPOases(){}

        void solve(Vector_type& solution){}
        Vector_type solve()
        {
            Vector_type solution;
            solve(solution);
            return solution;
        }


        unsigned int getNumberOfBounds(){return _bounds.size();}
        unsigned int getNumberOfConstraints(){return _constraints.size();}

    private:
        vector<qpOASES::Bounds> _bounds;
        vector<qpOASES::Constraints> _constraints;
    };

    }
}

#endif

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
            _constraints(0),
            _options(0),
            _qpProblems(0),
            _initial_guess(false)
        {}

        ~QPOases(){}

        void solve(Vector_type& solution)
        {

        }

        Vector_type solve()
        {
            Vector_type solution;
            solve(solution);
            return solution;
        }


        unsigned int getNumberOfBounds(){return _bounds.size();}
        unsigned int getNumberOfConstraints(){return _constraints.size();}
        unsigned int getNumberOfStacks(){return _qpProblems.size();}

    protected:
        vector<qpOASES::Bounds> _bounds;
        vector<qpOASES::Constraints> _constraints;
        vector<qpOASES::Options> _options;
        vector<qpOASES::QProblem> _qpProblems;
        bool _initial_guess;
    };

    }
}

#endif

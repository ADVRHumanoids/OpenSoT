#ifndef _WB_SOT_SOLVERS_BE_FACTORY_H_
#define _WB_SOT_SOLVERS_BE_FACTORY_H_

#include <OpenSoT/solvers/BackEnd.h>
#include <boost/make_shared.hpp>
#include <type_traits>

namespace OpenSoT{
    namespace solvers{
        enum class solver_back_ends{
            qpOASES,
            OSQP,
            GLPK,
            eiQuadProg,
            ODYS,
            qpSWIFT,
            proxQP
        };

        template < typename C, C beginVal, C endVal>
        class Iterator {
          typedef typename std::underlying_type<C>::type val_t;
          int val;
        public:
          Iterator(const C & f) : val(static_cast<val_t>(f)) {}
          Iterator() : val(static_cast<val_t>(beginVal)) {}
          Iterator operator++() {
            ++val;
            return *this;
          }
          C operator*() { return static_cast<C>(val); }
          Iterator begin() { return *this; } //default ctor is good
          Iterator end() {
              static const Iterator endIter=++Iterator(endVal); // cache it
              return endIter;
          }
          bool operator!=(const Iterator& i) { return val != i.val; }
        };

        typedef Iterator<solver_back_ends, solver_back_ends::qpOASES, solver_back_ends::proxQP> solver_back_ends_iterator;

        /**
         * @brief BackEndFactory creates an instance of an OpenSoT BackEnd
         * @param be_solver the type of solver
         * @param number_of_variables of the problem
         * @param number_of_constraints of the problem
         * @param hessian_type of the problem
         * @param eps_regularisation of the problem
         * @return a BackEnd pointer
         */
        BackEnd::Ptr BackEndFactory(const solver_back_ends be_solver, const int number_of_variables,
                               const int number_of_constraints,
                               OpenSoT::HessianType hessian_type,
                               const double eps_regularisation);

        /**
         * @brief whichBackEnd return a string whith the used BackEnd
         * @param be_solver the input enum
         * @return string
         */
        std::string whichBackEnd(const solver_back_ends be_solver);
    }
}

#endif

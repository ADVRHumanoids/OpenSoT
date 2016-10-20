#ifndef _LEGACY_WB_SOT_SOLVERS_QP_OASES_H_
#define _LEGACY_WB_SOT_SOLVERS_QP_OASES_H_

#include <OpenSoT/solvers/QPOases.h>
#include <yarp/sig/Vector.h>
#include <idynutils/cartesian_utils.h>


using namespace yarp::sig;


namespace OpenSoT{
namespace legacy{
    namespace solvers{

    /**
     * @brief The QPOases_sot class implement a solver that accept a Stack of Tasks with Bounds and Constraints
     */
    class QPOases_sot: public OpenSoT::solvers::QPOases_sot
    {
    public:
        bool solve(yarp::sig::Vector& solution)
        {
            Eigen::VectorXd solution__;
            bool a = OpenSoT::solvers::QPOases_sot::solve(solution__);
            if(a)
                solution = cartesian_utils::fromEigentoYarp(solution__);
            return a;
        }


    };
    }

    }
}

#endif

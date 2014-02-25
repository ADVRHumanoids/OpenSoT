#include "task_solver.h"
#include <yarp/math/Math.h>

using namespace yarp::math;


/// dT is in [s]
bool task_solver::computeControlHQP(const yarp::sig::Matrix &J1,
                                    const yarp::sig::Vector &e1,
                                    const yarp::sig::Matrix &J2,
                                    const yarp::sig::Vector &eq,
                                    const yarp::sig::Vector &qMax,
                                    const yarp::sig::Vector &qMin,
                                    const yarp::sig::Vector &q,
                                    const double &MAX_JOINT_VELOCITY,
                                    const double &dT,
                                    yarp::sig::Vector &dq_ref)
{
    int nj = dq_ref.size();
    static bool initial_guess = false;

    static yarp::sig::Vector dq1(nj, 0.0);
    static yarp::sig::Vector y1(nj, 0.0);
    static yarp::sig::Vector dq2(nj, 0.0);
    static yarp::sig::Vector y2(nj, 0.0);

    static qpOASES::Bounds bounds1;
    static qpOASES::Bounds bounds2;
    static qpOASES::Constraints constraints2;

    /**
      We solve a single QP where the priority between
      different tasks is set by using a weight matrix Q

      min         (Ax - b)'Q(Ax - b)
      subj to     l <=   x <=  u

      QPOASES::Quadratic_program solves by default a quadratic problem in the form
      min         x'Hx + x'g
      subj to  Alb <= Ax <= Aub
                 l <=  x <= u
     **/

    int njTask1 = J1.rows(); // size for task 1

    yarp::sig::Matrix H1 = J1.transposed()*J1; // size of problem is bigger than the size of task because we need the extra slack variables
    yarp::sig::Vector g1 = -1.0*J1.transposed()*e1;

    yarp::sig::Matrix H2 = J2.transposed();
    yarp::sig::Vector g2 = -1.0*J2.transposed()*eq;

    yarp::sig::Vector u1(nj); yarp::sig::Vector l1(nj); //Joint Limits constraints;
    u1 = (qMax - q)*dT; //Suppose we want to reach joint limits in 1 sec
    l1 = (qMin - q)*dT;

    yarp::sig::Vector u2(nj, MAX_JOINT_VELOCITY*dT); //Max velocity
    yarp::sig::Vector u(nj); yarp::sig::Vector l(nj);
    for(unsigned int i = 0; i < nj; ++i){
        u[i] = std::min(u1[i], u2[i]);
        l[i] = std::max(l1[i], -u2[i]);
    }

    USING_NAMESPACE_QPOASES

    /** Setting up QProblem object. **/
    Options qpOasesOptionsqp1;
    qpOasesOptionsqp1.printLevel = PL_HIGH;
    qpOasesOptionsqp1.setToReliable();
    QProblemB qp1( nj, HST_SEMIDEF);
    qp1.setOptions( qpOasesOptionsqp1 );

    Options qpOasesOptionsqp2;
    qpOasesOptionsqp2.printLevel = PL_HIGH;
    qpOasesOptionsqp2.setToReliable();
    QProblem qp2( nj, njTask1, HST_POSDEF);
    qp2.setOptions( qpOasesOptionsqp2 );


    /** Solve first QP. **/
    /**
      Qui viene usato initial_guess che viene messo a false all'inizio del metodo...
      forse deve essere una variabile della classe.
    **/
    int nWSR = 2^32;
    if(initial_guess==true)
        qp1.init( H1.data(),g1.data(), l.data(), u.data(),nWSR,0, dq1.data(), y1.data(), &bounds1);
    else
        qp1.init( H1.data(),g1.data(), l.data(), u.data(), nWSR,0);

    if(dq1.size() != qp1.getNV()) {
        dq1.resize(qp1.getNV());
        initial_guess = false;
    }
    if(y1.size() != qp1.getNV()) {
        y1.resize(qp1.getNV());
        initial_guess = false;
    }

    int success1 = qp1.getPrimalSolution( dq1.data() );
    qp1.getDualSolution(y1.data());
    qp1.getBounds(bounds1);

    if(success1== RET_QP_NOT_SOLVED ||
      (success1 != RET_QP_SOLVED && success1 != SUCCESSFUL_RETURN))
    {
        std::cout<<"ERROR OPTIMIZING FIRST TASK! ERROR #"<<success1<<std::endl;
        initial_guess = false;
    }
    else
    {
        /** Solve second QP. **/
        yarp::sig::Vector b2 = J1*dq1;
        yarp::sig::Vector b2u = b2;
        yarp::sig::Vector b2l = b2;

        nWSR = 2^32;

        /**
          Qui viene usato initial_guess che viene messo a false all'inizio del metodo...
          forse deve essere una variabile della classe.
        **/
        if(initial_guess == true)
            qp2.init( H2.data(),g2.data(), J1.data(), l.data(), u.data(),
                      b2l.data(), b2u.data(), nWSR, 0, dq2.data(), y2.data(),
                      &bounds2, & constraints2);
        else
            qp2.init( H2.data(),g2.data(), J1.data(), l.data(), u.data(),
                      b2l.data(), b2u.data(), nWSR, 0);
        if(dq2.size() != qp2.getNV()) {
            dq2.resize(qp2.getNV());
            initial_guess = false;
        }
        if(y2.size() != qp2.getNV() + qp2.getNC()) {
            y2.resize(qp2.getNV() + qp2.getNC());
            initial_guess = false;
        }

        int success2 = qp2.getPrimalSolution( dq2.data() );
        qp2.getDualSolution(y2.data());
        qp2.getBounds(bounds2);
        qp2.getConstraints(constraints2);

        if(success2 == RET_QP_NOT_SOLVED ||
          (success2 != RET_QP_SOLVED && success2 != SUCCESSFUL_RETURN))
        {
            std::cout<<"ERROR OPTIMIZING POSTURE TASK! ERROR #"<<success2<<std::endl;
            initial_guess = false;
        }
        else
        {
            dq_ref = dq2;
            return true;
        }
    }
    return false;
}

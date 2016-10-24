#ifndef _LEGACY_SUBTASK_H__
#define _LEGACY_SUBTASK_H__

 #include <OpenSoT/SubTask.h>
 #include <yarp/sig/all.h>
 #include <yarp/math/Math.h>
 #include <yarp/math/SVD.h>
#include <idynutils/cartesian_utils.h>


 namespace OpenSoT {
 namespace legacy{

    /**
     * @brief SubTask represents a task which is obtained as a sub task
     * in the form \f$T(A_s,b_s)\f$,
     * where \f$A_s\f$ is a reduced task error jacobian
     * and \f$b_s\f$ its corresponding task error.
     * Updating a SubTask calls the update method for the corresponding father
     * task which it reduces, and recreates the reduced A,b and Weight matrices.
     * In the same way, the constraints of the SubTask are those of the father Task,
     * as well as the weight matrix W (the \f$W_\text{subtask}\f$ is a submatrix of \f$W\f$)
     * On the other side, the \f$\lambda\f$ for the SubTask is unique to the SubTask.
    */
    class SubTask : public OpenSoT::SubTask {

    public:
        virtual void setWeight(const yarp::sig::Matrix& W)
        {
            OpenSoT::SubTask::setWeight(
                        cartesian_utils::toEigen(W));
        }


    };
}

 }

#endif

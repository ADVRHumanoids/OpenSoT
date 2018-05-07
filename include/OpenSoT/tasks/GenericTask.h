#ifndef __OPENSOT_GENERIC_TASK__
#define __OPENSOT_GENERIC_TASK__

#include <OpenSoT/Task.h>
#include <OpenSoT/utils/Affine.h>
#include <XBotInterface/ModelInterface.h>

namespace OpenSoT { namespace tasks  {

/**
 * @brief The GenericTask class implements a task where the A and b matrix can be updated outside the task itself,
 * the task has the generic form:
 *
 * ||Ax - b||
 */
class GenericTask: public Task<Eigen::MatrixXd, Eigen::VectorXd> {
  public:
    typedef boost::shared_ptr<GenericTask> Ptr;

    /**
     * @brief GenericTask constructor
     * @param task_id name of the task
     * @param A matrix
     * @param b vector
     */
    GenericTask(const std::string& task_id,
                const Eigen::MatrixXd& A, const Eigen::VectorXd& b);

    /**
     * @brief GenericTask constructor
     * @param task_id name of the task
     * @param A matrix
     * @param b vector
     * @param var variable
     */
    GenericTask(const std::string& task_id,
                const Eigen::MatrixXd& A, const Eigen::VectorXd& b,
                const AffineHelper& var);


    ~GenericTask();

    virtual void _update(const Eigen::VectorXd &x);

    /**
     * @brief setA update the A matrix of the task
     * @param A matrix
     * @return false if A.rows() != b.size()
     */
    bool setA(const Eigen::MatrixXd& A);

    /**
     * @brief setb update the b matrix of the task
     * @param b vector
     * @return false if b.size() != A.rows()
     */
    bool setb(const Eigen::VectorXd& b);

    /**
     * @brief setAb update A and b of the task
     * @param A matrix
     * @param b vector
     * @return false if A.rows() != b.size()
     * NOTE: this is NOT RT-SAFE!
     */
    bool setAb(const Eigen::MatrixXd& A, const Eigen::VectorXd& b);

private:
    AffineHelper _var;
    AffineHelper _task;

    Eigen::MatrixXd __A;
    Eigen::VectorXd __b;


};

}}

#endif

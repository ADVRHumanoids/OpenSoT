#ifndef __OPENSOT_GENERIC_LP_TASK__
#define __OPENSOT_GENERIC_LP_TASK__

#include <OpenSoT/Task.h>
#include <OpenSoT/utils/Affine.h>
#include <OpenSoT/tasks/GenericTask.h>
#include <XBotInterface/ModelInterface.h>

namespace OpenSoT { namespace tasks  {

/**
 * @brief The GenericLPTask class implements a task where the A and b matrix are zeros and c can be updated outside the task itself,
 * the task has the generic form:
 *
 * c'x
 */
class GenericLPTask: public Task<Eigen::MatrixXd, Eigen::VectorXd> {
  public:
    typedef boost::shared_ptr<GenericLPTask> Ptr;

    /**
     * @brief GenericLPTask constructor
     * @param task_id name of the task
     * @param c vector
     */
    GenericLPTask(const std::string& task_id, const Eigen::VectorXd& c);

    /**
     * @brief GenericLPTask constructor
     * @param task_id name of the task
     * @param c vector
     * @param var variable
     */
    GenericLPTask(const std::string& task_id, const Eigen::VectorXd& c,
                  const AffineHelper& var);


    ~GenericLPTask();

    virtual void _update(const Eigen::VectorXd &x);

    /**
     * @brief setc update the c of the task
     * @param c vector
     * @return false if c.size() != b.size()
     */
    bool setc(const Eigen::VectorXd& c);

private:
    GenericTask::Ptr _task;


};

}}

#endif

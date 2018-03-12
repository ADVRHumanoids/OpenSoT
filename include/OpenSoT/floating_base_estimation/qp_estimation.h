#ifndef _OPENSOT_FLOATING_BASE_ESTIMATION_QP_ESTIMATION_
#define _OPENSOT_FLOATING_BASE_ESTIMATION_QP_ESTIMATION_

#include <OpenSoT/utils/FloatingBaseEstimation.h>
#include <OpenSoT/tasks/floating_base/Contact.h>
#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/constraints/GenericConstraint.h>
#include <OpenSoT/tasks/floating_base/IMU.h>


namespace OpenSoT{
namespace floating_base_estimation{
/**
     * @brief The qp_estimation class uses a QP to estimate the floating base pose and velocities from
     * contact information and IMU (optional).
     * The QP solved is a weighted sum of the measurements.
     */
    class qp_estimation: public OpenSoT::FloatingBaseEstimation
    {
    public:
        typedef boost::shared_ptr<qp_estimation> Ptr;

        qp_estimation(XBot::ModelInterface::Ptr model, XBot::ImuSensor::ConstPtr imu,
                      std::vector<std::string> contact_links,
                      const Eigen::MatrixXd& contact_matrix = Eigen::MatrixXd::Identity(6,6));
        ~qp_estimation();
        bool update(double dT);

        virtual bool setContactState(const std::string& contact_link, const bool state);

        virtual void log(XBot::MatLogger::Ptr logger);

    private:
        std::list<OpenSoT::tasks::Aggregated::TaskPtr> _contact_tasks;
        OpenSoT::tasks::floating_base::IMU::Ptr _imu_task;
        std::map<std::string, unsigned int> _map_tasks;
        tasks::Aggregated::Ptr _aggregated_tasks;
        solvers::iHQP::Ptr _solver;
        constraints::GenericConstraint::Ptr _fb_limits;
        AutoStack::Ptr _autostack;

    };
}

}

#endif

#include <OpenSoT/utils/FloatingBaseEstimation.h>
#include <OpenSoT/tasks/floating_base/Contact.h>
#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/solvers/QPOases.h>
#include <OpenSoT/constraints/GenericConstraint.h>



namespace OpenSoT{
namespace floating_base_estimation{
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
        std::map<std::string, unsigned int> _map_tasks;
        tasks::Aggregated::Ptr _aggregated_tasks;
        solvers::QPOases_sot::Ptr _solver;
        constraints::GenericConstraint::Ptr _fb_limits;
        AutoStack::Ptr _autostack;

    };
}

}

#ifndef __OPENSOT_ACCELERATION_TASK_DYNFEAS_H__
#define __OPENSOT_ACCELERATION_TASK_DYNFEAS_H__

#include <OpenSoT/Task.h>
#include <OpenSoT/utils/Affine.h>
#include <XBotInterface/ModelInterface.h>

namespace OpenSoT {
    namespace tasks {
        namespace acceleration {
        /**
         * @brief The DynamicFeasibility class models floating-base underactuation:
         *
         *  Mqddot + h = Jc'Wc
         *
         * where M is the [6 x n+6] floating base inertia matrix, qddot are the [n+6] joint accelerations,
         * h are the [6] vector of non linear terms, Jc' are the stacked contacts [6 x k*6] (full wrench) and Wc are
         * the [k*6] contact wrenches.
         *
         */
        class DynamicFeasibility : public Task<Eigen::MatrixXd, Eigen::VectorXd> {

        public:

            typedef boost::shared_ptr<DynamicFeasibility> Ptr;

            /**
             * @brief DynamicFeasibility constructor
             * @param task_id id
             * @param robot model reference
             * @param qddot variables
             * @param wrenches variables
             * @param contact_links vector of string of contact links
             */
            DynamicFeasibility(const std::string task_id,
                               const XBot::ModelInterface& robot,
                               const AffineHelper& qddot,
                               const std::vector<AffineHelper>& wrenches,
                               const std::vector<std::string>& contact_links);



            /**
             * @brief enableContact for task computation
             * @param contact_link
             * @return false if the contact is not in the contacts vector
             */
            bool enableContact(const std::string& contact_link);

            /**
             * @brief disableContact for task computation
             * @param contact_link
             * @return false if the contact is not in the contacts vector
             */
            bool disableContact(const std::string& contact_link);

            /**
             * @brief getEnabledContacts
             * @return vector of booleans indicating active contact (in the same order as in contact_links)
             */
            const std::vector<bool>& getEnabledContacts() const;

            Eigen::VectorXd checkTask(const Eigen::VectorXd& x);

        private:
                virtual void _update(const Eigen::VectorXd& x);

                const XBot::ModelInterface& _robot;
                AffineHelper _qddot;
                std::vector<AffineHelper> _wrenches;
                std::vector<std::string> _contact_links;
                AffineHelper _dyn_constraint;

                std::vector<bool> _enabled_contacts;

                Eigen::VectorXd _h, _hu;
                Eigen::MatrixXd _B, _Bu, _Jtmp;
                Eigen::Matrix6d _Jf;
        };

        }
    }
}

#endif

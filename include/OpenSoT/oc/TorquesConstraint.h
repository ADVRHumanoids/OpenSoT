#ifndef __OPENSOT_TORQUES_CONSTRAINT_H__
#define __OPENSOT_TORQUES_CONSTRAINT_H__

#include <OpenSoT/Constraint.h>
#include <OpenSoT/oc/TorquesTask.h>
#include <OpenSoT/utils/Affine.h>
#include <xbot2_interface/xbotinterface2.h>

namespace OpenSoT
{
    namespace oc
    {

        class DynamicsConstraint : public Constraint<Eigen::MatrixXd, Eigen::VectorXd>
        {
        public:
            typedef std::shared_ptr<DynamicsConstraint> Ptr;

        private:
            XBot::ModelInterface& _robot;
            AffineHelper _dU;
            AffineHelper _dX;

            Eigen::VectorXd _torquelim;
            TorquesTask _task;
            double _boundScaling;

            void _update();
        
        public:

            DynamicsConstraint(XBot::ModelInterface &robot,
                               const AffineHelper &dX,
                               const AffineHelper &dU);

            Eigen::VectorXd getTorqueLimit();
            void setTorqueLimit(Eigen::VectorXd tau);
        
        };
    }
}

#endif

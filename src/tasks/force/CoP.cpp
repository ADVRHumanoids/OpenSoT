#include <OpenSoT/tasks/force/CoP.h>
#include <idynutils/cartesian_utils.h>

using namespace yarp::math;

namespace OpenSoT {
   namespace tasks {
       namespace force {

        CoP::CoP(const yarp::sig::Vector &x, iDynUtils &robot, double ft_to_sole_height, const std::string &ft_in_contact):
            Task("CoP", x.size()), _robot(robot),
            _d(ft_to_sole_height), _ft_in_contact(ft_in_contact)
        {
            _W.resize(2,2);
            _W.eye();
            _lambda = 0.0;

            _hessianType = HST_SEMIDEF;

            _b = yarp::sig::Vector(6,0.0);

            moveit::core::LinkModel* ft_link = robot.moveit_robot_model->getLinkModel(_ft_in_contact);

            _ft_index = robot.iDyn3_model.getFTSensorIndex(ft_link->getParentJointModel()->getName());
            if(_ft_index == -1)
                throw "Passed ft_frame is not in model!";

            _wrench_in_sensor_frame.resize(6, 0.0);
            _robot.iDyn3_model.getSensorMeasurement(_ft_index, _wrench_in_sensor_frame);

            _desiredCoP = cartesian_utils::computeFootZMP(_wrench_in_sensor_frame.subVector(0,2),
                                            _wrench_in_sensor_frame.subVector(3,5), _d, 0.0);

            _update();
        }

        CoP::~CoP()
        {

        }

        void CoP::computeA()
        {
            yarp::sig::Matrix world_T_ft = _robot.iDyn3_model.getPosition(
                        _robot.iDyn3_model.getLinkIndex(_ft_in_contact));

            //Transform CoP desired from sensor to world
            yarp::sig::Matrix CoP_d_in_world(4,4); CoP_d_in_world.eye();
            CoP_d_in_world(0,3) = _desiredCoP(0);
            CoP_d_in_world(1,3) = _desiredCoP(1);
            CoP_d_in_world = world_T_ft * CoP_d_in_world;

            _A.resize(2,6); _A.zero();
            _A(0,0) = _d;  _A(0,2) = -CoP_d_in_world(0,3); _A(0,4) = -1;
            _A(1,1) = -_d; _A(0,2) = -CoP_d_in_world(1,3); _A(0,3) = -1;
        }

        yarp::sig::Vector CoP::getReference() const
        {
            return _desiredCoP;
        }

        void CoP::setReference(const yarp::sig::Vector &desiredCoP)
        {
            _desiredCoP = desiredCoP;
        }

        std::string CoP::getBaseLink()
        {
            return BASE_LINK;
        }

        void CoP::_update(const yarp::sig::Vector &x)
        {
            computeA();
        }

       }
   }
}

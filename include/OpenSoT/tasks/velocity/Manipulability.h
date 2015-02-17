#ifndef __TASKS_VELOCITY_MANIPULABILITY_H__
#define __TASKS_VELOCITY_MANIPULABILITY_H__

#include <OpenSoT/Task.h>
#include <idynutils/idynutils.h>
#include <idynutils/cartesian_utils.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <OpenSoT/tasks/velocity/CoM.h>

using namespace yarp::math;


namespace OpenSoT {
    namespace tasks {
        namespace velocity {
            /**
             * @brief The Manipulability class implements a task that tries to maximize the Manipulability
             * index computed as (Robotics: Modelling, Planning and Control, pag. 126):
             *
             *              w = sqrt(det(J*W*J'))
             *
             * The gradient of w is then computed and projected using the gardient projection method.
             */
            class Manipulability : public Task < yarp::sig::Matrix, yarp::sig::Vector > {
            public:
                typedef boost::shared_ptr<Manipulability> Ptr;

                Manipulability(const yarp::sig::Vector& x, const iDynUtils& robot_model, const Cartesian::Ptr CartesianTask);
                Manipulability(const yarp::sig::Vector& x, const iDynUtils& robot_model, const CoM::Ptr CartesianTask);

                ~Manipulability();

                void _update(const yarp::sig::Vector& x);

                /**
                 * @brief ComputeManipulabilityIndex
                 * @return the manipulability index at the actual configuration q (ast from latest update(q))
                 */
                double ComputeManipulabilityIndex();

                /**
                 * @brief setW set a Weight matrix for the manipulability index
                 * @param W weight matrix
                 */
                void setW(const yarp::sig::Matrix& W){
                    _manipulabilityIndexGradientWorker.setW(W);
                }

                /**
                 * @brief getW get a Weight matrix for the manipulability index
                 */
                yarp::sig::Matrix getW(){
                    return _manipulabilityIndexGradientWorker.getW();
                }

            protected:

                yarp::sig::Vector _x;

                /**
                 * @brief The ComputeManipulabilityIndexGradient class implements a worker class to computes
                 * the effort for a certain configuration.
                 */
                class ComputeManipulabilityIndexGradient: public cartesian_utils::CostFunction{
                public:
                    iDynUtils _robot;
                    const iDynUtils& _model;
                    yarp::sig::Matrix _W;
                    yarp::sig::Vector _zeros;
                    OpenSoT::Task<yarp::sig::Matrix, yarp::sig::Vector>::TaskPtr _CartesianTask;

                    ComputeManipulabilityIndexGradient(const yarp::sig::Vector& q, const iDynUtils& robot_model,
                                                       const Cartesian::Ptr CartesianTask) :
                        _robot(robot_model.getRobotName(),
                               robot_model.getRobotURDFPath(),
                               robot_model.getRobotSRDFPath()),
                        _model(robot_model),
                        _W(q.size(),q.size()),
                        _zeros(q.size(), 0.0)
                    {
                        _W.eye();

                        _robot.updateiDyn3Model(_model.iDyn3_model.getAng(),
                                                _model.iDyn3_model.getDAng(),
                                                _model.iDyn3_model.getD2Ang(), true);
                        _robot.switchAnchor(_model.getAnchor());
                        _robot.setAnchor_T_World(_model.getAnchor_T_World());

                        _CartesianTask = Cartesian::Ptr(new Cartesian(CartesianTask->getTaskID(), q,
                                                _robot, CartesianTask->getDistalLink(), CartesianTask->getBaseLink()));
                    }

                    ComputeManipulabilityIndexGradient(const yarp::sig::Vector& q, const iDynUtils& robot_model,
                                                       const CoM::Ptr CartesianTask) :
                        _robot(robot_model.getRobotName(),
                               robot_model.getRobotURDFPath(),
                               robot_model.getRobotSRDFPath()),
                        _model(robot_model),
                        _W(q.size(),q.size()),
                        _zeros(q.size(), 0.0)
                    {
                        _W.eye();

                        _robot.updateiDyn3Model(_model.iDyn3_model.getAng(),
                                                _model.iDyn3_model.getDAng(),
                                                _model.iDyn3_model.getD2Ang(), true);
                        _robot.switchAnchor(_model.getAnchor());
                        _robot.setAnchor_T_World(_model.getAnchor_T_World());

                        _CartesianTask = CoM::Ptr(new OpenSoT::tasks::velocity::CoM(q, _robot));
                    }

                    double compute(const yarp::sig::Vector &q)
                    {
                        if(_robot.getAnchor() != _model.getAnchor())
                            _robot.switchAnchor(_model.getAnchor());

                        _robot.updateiDyn3Model(q, true);

                        if(_model.getAnchor_T_World() != _robot.getAnchor_T_World())
                        {
                            assert("if q and anchor are the same, anchor_t_world should be the same!");

                            _robot.setAnchor_T_World(_model.getAnchor_T_World());
                        }

                        _CartesianTask->update(q);

                        return computeManipulabilityIndex();
                    }

                    void setW(const yarp::sig::Matrix& W) { _W = W; }

                    yarp::sig::Matrix& getW() {return _W;}

                    double computeManipulabilityIndex()
                    {
                        yarp::sig::Matrix J = _CartesianTask->getA();
                        return sqrt(det(J*_W*J.transposed()));
                    }
                };

                ComputeManipulabilityIndexGradient _manipulabilityIndexGradientWorker;
            };
        }
    }
}

#endif

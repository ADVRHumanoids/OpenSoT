/*
 * Copyright (C) 2014 Walkman
 * Author: Alessio Rocchi, Enrico Mingo
 * email:  alessio.rocchi@iit.it, enrico.mingo@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU Lesser General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#ifndef __PREVIEWER_H__
#define __PREVIEWER_H__

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>
#include <idynutils/idynutils.h>
#include <idynutils/cartesian_utils.h>
#include <iCub/iDynTree/yarp_kdl.h>
#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <OpenSoT/tasks/velocity/CoM.h>
#include <utility>

/**
 * @example example_previwer.cpp
 * The Previwer class allows to check the SoT is able to perform a trajectory
 * using a defined stack without (self-) collisions and with limited tracking error.
 */

namespace OpenSoT {
    /**
     * @brief The Previewer class creates a kinematic simulator of a robot
     *        that uses the OpenSoT IK as a solver, and a trajectory generator
     *
     * You can see an example in @ref example_previewer.cpp
     */
    template <class TrajectoryGenerator>
    class Previewer
    {
        public:
            typedef boost::shared_ptr<TrajectoryGenerator> TrajGenPtr;

            struct Results
            {
                enum Reason { COLLISION_EVENT, ERROR_UNBOUNDED };
                static std::string reasonToString(Reason reason) {
                    switch(reason)
                    {
                        case COLLISION_EVENT:
                            return "Collision Occured";
                        case ERROR_UNBOUNDED:
                            return "Unbounded Cartesian Error";
                        default:
                            return "Unknown error";
                    }
                }

                struct TrajectoryLogEntry   { double t; yarp::sig::Vector q;
                                              TrajectoryLogEntry(double t, const yarp::sig::Vector& q) :
                                                t(t), q(q) {} };
                struct FailuresLogEntry     { double t; Reason reason;
                                              OpenSoT::Task<yarp::sig::Matrix, yarp::sig::Vector>::TaskPtr task;
                                              FailuresLogEntry(double t, Reason reason) :
                                                t(t), reason(reason) {}
                                              FailuresLogEntry(double t, Reason reason,
                                                               OpenSoT::Task<yarp::sig::Matrix, yarp::sig::Vector>::TaskPtr task) :
                                                t(t), reason(reason), task(task) {} };

                std::vector <TrajectoryLogEntry> trajectory;
                std::vector <FailuresLogEntry> failures;

                void logFailure(double time, Reason reason)
                {
                    failures.push_back(
                        FailuresLogEntry(time, reason));
                }

                void logFailure(double time, Reason reason,
                                OpenSoT::Task<yarp::sig::Matrix, yarp::sig::Vector>::TaskPtr task)
                {
                    failures.push_back(
                        FailuresLogEntry(time, reason, task));
                }

                void logTrajectory(double time, const yarp::sig::Vector& q)
                {
                    trajectory.push_back(
                        TrajectoryLogEntry(time, q));
                }
            };

            class TrajBinding
            {
            public:
                double convergenceTolerance;
                double maximumAllowedError;
                OpenSoT::Task<yarp::sig::Matrix, yarp::sig::Vector>::TaskPtr task;
                TrajGenPtr trajectoryGenerator;

                TrajBinding(TrajGenPtr trajectoryGenerator,
                            OpenSoT::Task<yarp::sig::Matrix, yarp::sig::Vector>::TaskPtr task,
                            double maximumAllowedError = 1e-1,
                            double convergenceTolerance = 1e-4) :
                    convergenceTolerance(convergenceTolerance),
                    maximumAllowedError(maximumAllowedError),
                    task(task),
                    trajectoryGenerator(trajectoryGenerator)
                {}
            };

            typedef std::list<TrajBinding> TrajectoryBindings;
            typedef boost::shared_ptr<OpenSoT::Previewer<TrajectoryGenerator>> Ptr;
            typedef boost::accumulators::accumulator_set<double,
                                                        boost::accumulators::stats<boost::accumulators::tag::rolling_mean>
                                                        > Accumulator;
            struct ErrorLog {
                Accumulator accumulator;
                double previousMean;
                double lastError;

                ErrorLog(int window_size = 100) :
                    accumulator(boost::accumulators::tag::rolling_mean::window_size = window_size),
                    previousMean(0.0), lastError(0.0) {}

                void update(double error)
                {
                    lastError = error;
                    previousMean = this->getRollingMean();
                    accumulator(lastError);
                }

                double getRollingMean()
                {
                    return boost::accumulators::extract::rolling_mean(accumulator);
                }
            };

            typedef std::map<OpenSoT::Task<yarp::sig::Matrix, yarp::sig::Vector>::TaskPtr,
                             ErrorLog> ErrorMap;
            typedef std::map<OpenSoT::Task<yarp::sig::Matrix, yarp::sig::Vector>::TaskPtr,
                             KDL::Frame>  CartesianNodes;

        private:
            ErrorMap cartesianErrors;

            // nodes are last "far" configurations. i.e., whenever a position moves more than a threshold,
            // it gets saved in a node. For each task, and for the q vector, we then keep an history of one value.
            CartesianNodes cartesianNodes;
            yarp::sig::Vector qNode;

            yarp::sig::Vector q;
            yarp::sig::Vector dq;

        protected:
            OpenSoT::AutoStack::Ptr autostack;
            TrajectoryBindings bindings;
            double dT;
            iDynUtils& model;
            OpenSoT::Solver<yarp::sig::Matrix, yarp::sig::Vector>::SolverPtr solver;
            double t;



            bool isCartesian(OpenSoT::Task<yarp::sig::Matrix, yarp::sig::Vector>::TaskPtr task)
            {
                return (bool)boost::dynamic_pointer_cast<OpenSoT::tasks::velocity::Cartesian>(task);
            }

            OpenSoT::tasks::velocity::Cartesian::Ptr asCartesian(OpenSoT::Task<yarp::sig::Matrix, yarp::sig::Vector>::TaskPtr task)
            {
                return boost::dynamic_pointer_cast<OpenSoT::tasks::velocity::Cartesian>(task);
            }

            bool isCoM(OpenSoT::Task<yarp::sig::Matrix, yarp::sig::Vector>::TaskPtr task)
            {
                return (bool)boost::dynamic_pointer_cast<OpenSoT::tasks::velocity::CoM>(task);
            }

            OpenSoT::tasks::velocity::CoM::Ptr asCoM(OpenSoT::Task<yarp::sig::Matrix, yarp::sig::Vector>::TaskPtr task)
            {
                return boost::dynamic_pointer_cast<OpenSoT::tasks::velocity::CoM>(task);
            }

            /**
             * @brief cartesianErrorIsBounded checks whether cartesian errors for all tasks are lower than a threshold
             * @return true if cartesian error for all tasks is bounded
             */
            bool cartesianErrorIsBounded()
            {
                bool bounded = true;
                for(typename TrajectoryBindings::iterator b = bindings.begin();
                    b != bindings.end();
                    ++b)
                {
                    if(cartesianErrors[b->task].lastError > b->maximumAllowedError)
                        bounded = false;
                }

                return bounded;
            }

            /**
             * @brief cartesianErrorDecreasing checks whether the moving average of the cartesian
             *        errors norm is decreasing
             * @param threshold the minimum decrease we detect. If we converge slower than this, it will not get detected
             * @return true if error is decreasing on all tasks
             */
            bool cartesianErrorDecreasing(double threshold = 1e-9)
            {
                bool allTasksDecreasing = true;
                for(typename TrajectoryBindings::iterator b = bindings.begin();
                    b != bindings.end();
                    ++b)
                {
                    if(cartesianErrors[b->task].previousMean - cartesianErrors[b->task].getRollingMean() < threshold)
                        allTasksDecreasing = false;
                }
                return allTasksDecreasing;
            }

            /**
             * @brief cartesianErrorConverged checks whether cartesian error is smaller
             *        than the convergence tolerance
             * @return true if cartesian error converged on all tasks
             */
            bool cartesianErrorConverged()
            {
                bool allTasksConverged = true;
                for(typename TrajectoryBindings::iterator b = bindings.begin();
                    b != bindings.end();
                    ++b)
                {
                    if(cartesianErrors[b->task].lastError > b->convergenceTolerance)
                        allTasksConverged = false;
                }
                return allTasksConverged;
            }

            bool cartesianErrorOK()
            {
                return (cartesianErrorConverged() && !cartesianErrorDecreasing());
            }

            /**
             * @brief cartesianPoseChanged checks whether the cartesian pose for a certain task changed too much
             * @param task the task for which to check the cartesian pose or position
             * @param threshold the threshold in norm
             * @return true if the cartesian pose or position changed more than threshold, in norm
             */
            bool cartesianPoseChanged(OpenSoT::Task<yarp::sig::Matrix, yarp::sig::Vector>::TaskPtr task,
                                      double threshold=1e-3)
            {
                double error(std::numeric_limits<double>::infinity());

                KDL::Frame f;
                yarp::sig::Matrix yf, yfOld;
                yarp::sig::Vector yv, yvOld;

                yarp::sig::Vector positionError(3, 0.0);
                yarp::sig::Vector orientationError(3, 0.0);

                if(isCartesian(task))
                {
                    yf = asCartesian(task)->getActualPose();
                }
                else if(isCoM(task))
                {
                    yv = asCoM(task)->getActualPosition();
                }

                if(isCartesian(task))
                {
                    assert(YarptoKDL(yf,f) && "error converting from yarp frame to kdl::Frame");
                }
                else if(isCoM(task))
                {
                    f.p.x(yv(0));
                    f.p.y(yv(1));
                    f.p.z(yv(2));
                }

                if(cartesianNodes.count(task) == 0)
                {
                    if(isCartesian(task))
                    {
                        cartesianNodes[task] = f;
                        return true;
                    }
                    else if(isCoM(task))
                    {
                        cartesianNodes[task] = f;
                        return true;
                    }
                    else return false;
                } else {

                    yfOld.resize(4,4);
                    assert(KDLtoYarp_position(cartesianNodes[task], yfOld) &&
                          "Error transforming a kdl::Frame into  a yarp 4x4 matrix");
                    assert(KDLtoYarp(cartesianNodes[task].p, yvOld) &&
                           "Error transforming a kdl::Vector into a yarp vector");

                    if(isCartesian(task))
                    {
                        cartesian_utils::computeCartesianError(yf, yfOld,
                                                               positionError, orientationError);
                        double Ko = asCartesian(task)->getOrientationErrorGain();
                        error = yarp::math::norm(
                                    yarp::math::cat(
                                        positionError,
                                        -Ko*orientationError));
                    } else if(isCoM(task))
                    {
                        using namespace yarp::math;
                        error = norm(yv - yvOld);
                    } else return false;

                    if(error > threshold)
                        cartesianNodes[task] = f;

                }

                return error > threshold;
            }

            /**
             * @brief checkConsistency checks whether the bindings are consistent
             *        Bindings are consistent if every binding of a trajectory is performed
             *        either with a Cartesian or CoM task.
             * @return true if all bindings are consistent
             */
            bool checkConsistency()
            {
                for(typename TrajectoryBindings::iterator b = bindings.begin();
                    b != bindings.end();
                    ++b)
                    if(!isCartesian(b->task) && !isCoM(b->task)) return false;
                return true;
            }

            /**
             * @brief shouldCheckSelfCollision checks whether cartesian poses changed or
             * joint position changed too much; in which case, we force a collision check
             * @return true if we need to call a self-collision check
             */
            bool shouldCheckSelfCollision()
            {
                bool cartesianPosesChanged = false;
                bool qChanged = false;

                for(typename TrajectoryBindings::iterator b = bindings.begin();
                    b != bindings.end();
                    ++b)
                    if(cartesianPoseChanged(b->task)) cartesianPosesChanged = true;

                qChanged = jointSpaceConfigurationChanged();

                return cartesianPosesChanged || qChanged;
            }

            /**
             * @brief jointSpaceConfigurationConverged checks whether the solver has converged
             * @param threshold comparable with the norm of the joint position change dq
             * @return true if the robot is stable (converged also to its null-space goals)
             */
            bool jointSpaceConfigurationConverged(double threshold = 1e-9)
            {
                return yarp::math::norm(dq) < threshold;
            }

            /**
             * @brief jointSpaceConfigurationOK checks whether the solver has converged
             * @return true if the robot is stable (converged also to its null-space goals)
             */
            bool jointSpaceConfigurationOK()
            {
                return jointSpaceConfigurationConverged();
            }


            /**
             * @brief jointSpaceConfigurationChanged checks whether joint space configuration changed
             *        with respect to last check (done by calling this function)
             * @param threshold comparable with the norm of joint space movements
             * @return true if the joint space configuration changed in norm more than threshold
             */
            bool jointSpaceConfigurationChanged(double threshold = 1e-5)
            {
                using namespace yarp::math;
                double error = norm(q - qNode);

                if(error > threshold)
                {
                    qNode = q;
                    return true;
                }

                return false;
            }

            /**
             * @brief reset clears the history of cartesian errors,
             * cartesian nodes, joint space node, resets q, dq, t
             */
            void reset()
            {
                cartesianErrors.clear();
                cartesianNodes.clear();

                t = 0.0;

                q = model.iDyn3_model.getAng();
                dq.resize(q.size()); dq.zero();
                qNode.resize(q.size()); qNode.zero();
            }

            /**
             * @brief trajectoriesCompleted checks whether we simulated forward in time enough
             *        to ecompass the duratino of all trajectories.
             * @param safetyMargin a percentage of the longest trajectory duration we wish to wait for.
             * @return true if t has surpassed the longest trajectory duration by safetyMargin (in percentage)
             */
            bool trajectoriesCompleted(double safetyMargin = 1.5)
            {
                bool trajCompleted = true;

                for(typename TrajectoryBindings::iterator b = bindings.begin();
                    b != bindings.end();
                    ++b)
                {
                    if(b->trajectoryGenerator->Duration() < t*safetyMargin)
                        trajCompleted = false;

                }

                return trajCompleted;
            }

            void updateErrorsStatistics(double windowSizeInSecs = .1)
            {
                for(typename TrajectoryBindings::iterator b = bindings.begin();
                    b != bindings.end();
                    ++b)
                {
                    if(cartesianErrors.count(b->task) == 0)
                        cartesianErrors[b->task] =
                            ErrorLog(std::ceil(windowSizeInSecs/dT));

                    cartesianErrors[b->task].update(yarp::math::norm(b->task->getb()));
                }
            }

            bool updateReferences()
            {
                for(typename TrajectoryBindings::iterator b = bindings.begin();
                    b != bindings.end();
                    ++b)
                {
                    KDL::Frame f = b->trajectoryGenerator->Pos(t);
                    KDL::Twist tw = b->trajectoryGenerator->Vel(t);
                    yarp::sig::Matrix yf = KDLtoYarp_position(f);
                    yarp::sig::Vector yt(6, 0.0);  KDLtoYarp(tw, yt);
                    yarp::sig::Vector yv(3,0.0);
                    assert(KDLtoYarp(f.p, yv) &&
                           "Error converting kdl::Vector in yarp::sig::Vector");

                    if(isCartesian(b->task))
                        asCartesian(b->task)->setReference(yf,yt*dT);
                    else if(isCoM(b->task))
                        asCoM(b->task)->setReference(yv, yt*dT);
                    else return false;
                }
                return true;
            }


        public:
            /**
             * @brief Previewer constructs a new previewer
             * @param dT the integration time
             * @param idyn the model from which to copy the robot state
             * @param autostack the stack
             * @param bindings a list of bindings between trajectories and tasks in the stack
             */
            Previewer(double dT,
                     iDynUtils& idyn,
                     OpenSoT::AutoStack::Ptr autostack,
                     TrajectoryBindings bindings) :
                autostack(autostack),
                bindings(bindings),
                dT(dT),
                model(idyn),
                solver(new OpenSoT::solvers::QPOases_sot(autostack->getStack(),
                                                         autostack->getBounds()))
            {
                if(!checkConsistency()) throw new std::runtime_error("Uncoherent bindings");
                this->reset(idyn);
            }

            /**
             * @brief Previewer constructs a new previewer
             * @param dT the integration time
             * @param idyn the model from which to copy the robot state
             * @param autostack the stack
             * @param bindings a list of bindings between trajectories and tasks in the stack
             * @param solver a solver to use to solve the stack. Must be initialized on the autostack
             */
            Previewer(double dT,
                     iDynUtils& idyn,
                     OpenSoT::AutoStack::Ptr autostack,
                     TrajectoryBindings bindings,
                     OpenSoT::Solver<yarp::sig::Matrix, yarp::sig::Vector>::SolverPtr solver) :
                autostack(autostack),
                bindings(bindings),
                dT(dT),
                model(idyn),
                solver(solver)
            {
                if(!checkConsistency()) throw new std::runtime_error("Uncoherent bindings");
                this->reset(idyn);
            }

            /**
             * @brief reset resets the internal model with the status of the specified one, and resets internal time.
             * @param idyn the model representing the current status of the robot
             */
            void reset(iDynUtils& idyn)
            {

                model.updateiDyn3Model( idyn.iDyn3_model.getAng(),
                                        idyn.iDyn3_model.getDAng(),
                                        idyn.iDyn3_model.getD2Ang(), true);
                model.switchAnchor(idyn.getAnchor());
                model.iDyn3_model.setFloatingBaseLink(idyn.iDyn3_model.getFloatingBaseLink());
                model.setAnchor_T_World(idyn.getAnchor_T_World());

                this->reset();
            }

            /**
             * @brief check simulates forward for time seconds. If time is infinity, it will simulate forward
             *              till the task converges in task space and joint space
             * @param time the time to simulate, in seconds
             * @param max_retries maximum number of times to retries solve() if errors occur
             * @return true if during the whole trajectory the tracking was always lower than the specified threshold,
             *              and no self-collisions were detected
             */
            bool check(const double time = std::numeric_limits<double>::infinity(),
                       const unsigned int max_retries = 3,
                       Results* results = NULL)
            {
                this->reset();

                bool finished = false;
                unsigned int retries = 0;
                bool check_ok = true;

                // logging initial trajectory node
                if(results != NULL) {
                    results->logTrajectory(t, q);
                }

                while(!finished) {
                    if(time == std::numeric_limits<double>::infinity())
                        finished =  trajectoriesCompleted() ||
                                    (cartesianErrorOK() && jointSpaceConfigurationOK());
                    else
                        finished = (t >= time);

                    if(retries >= max_retries) {
                        finished = true;
                        check_ok = false;
                        std::cerr << "Error with solver: could not solve" << std::endl;
                    }

                    model.updateiDyn3Model(q, true);

                    // update the stack to get new error statistics
                    autostack->update(q);

                    updateErrorsStatistics();

                    updateReferences();

                    // update the stack to update the references
                    autostack->update(q);

                    if(shouldCheckSelfCollision()) {
                        if(!model.checkSelfCollision())
                        {
                            check_ok = false;
                            if(results != NULL)
                                results->logFailure(t, Results::Results::COLLISION_EVENT);
                        }
                    }

                    if(!cartesianErrorIsBounded())
                    {
                        check_ok = false;
                        if(results != NULL)
                            results->logFailure(t, Results::ERROR_UNBOUNDED);
                    }

                    if(solver->solve(dq))
                    {
                        retries = 0;

                        q += dq;
                        t += dT;

                        if(results != NULL) {
                            results->logTrajectory(t, q);
                        }

                    } else ++retries;
                }

                return check_ok;
            }
    };
}

#endif

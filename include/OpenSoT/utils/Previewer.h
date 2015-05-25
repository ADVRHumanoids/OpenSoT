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
#include <yarp/math/Math.h>
#include <map>
#include <utility>

/**
 * @example example_previewer.cpp
 * The Previwer class allows to check the SoT is able to perform a trajectory
 * using a defined stack without (self-) collisions and with limited tracking error.
 */

namespace OpenSoT {
    /**
     * @brief The PreviewerCallBack class implements an interface
     * for callbacks used by Previewer's check()
     */
    class PreviewerCallBack
    {
    public:
        PreviewerCallBack() {}
        virtual void callback(const yarp::sig::Vector& q) = 0;
    };

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
            friend class testPreviewer;
            typedef boost::shared_ptr<TrajectoryGenerator> TrajGenPtr;

            /**
             * @brief The TrajBinding class represent an entry in a list of bindings needed for cartesian control.
             *        In every TrajBinding instance we specify a trajectory generator and a task.
             *        For every previewer simulation step, we compute a pose from the trajectory generator
             *        and assign it as a reference pose for the task which is bound to that trajectory generator.
             *        For every binding we also define error bounds that are checked at every simulation step
             *        and a convergence threshold which gets checked if the previewer is asked to simulate to infinity
             *        for early termination of the preview
             */
            class TrajBinding
            {
            public:
                enum ConvergencePolicy {
                    // converges when cartesian error wrt goal goes below a certain threshold
                    CONVERGE_ON_CARTESIAN_ERROR_SMALL,
                    // converges when cartesian error wrt goal goes below a certain threshold AND error is not decreasing
                    CONVERGE_ON_CARTESIAN_ERROR_SMALL_AND_NOT_DECREASING,
                    // converges when cartesian error wrt goal goes below a certain threshold
                    CONVERGE_ON_CARTESIAN_ERROR_AND_NULL_STABLE };

                ConvergencePolicy convergencePolicy;
                double convergenceTolerance;
                double maximumAllowedError;
                OpenSoT::Task<yarp::sig::Matrix, yarp::sig::Vector>::TaskPtr task;
                TrajGenPtr trajectoryGenerator;

                /**
                 * @brief TrajBinding binds a trajectory generator to a task
                 * @param trajectoryGenerator a pointer to a trajectory generator
                 * @param task a pointer to a task (either Cartesian or CoM)
                 * @param maximumAllowedError maximum error, in norm (R^6 or R^3), during tracking
                 *          Notice that the norm of the R^6 error gets checked, so the
                 *          orientationErrorGain of the Task is used when we use the Cartesian task
                 * @param convergenceTolerance maximum error, in norm (R^3), to the goal.
                 *          Notice that convergenceTolerance gets checked separately
                 *          from positionErrorNorm and orientationErrorNorm (if orientationErrorNorm exists)
                 */
                TrajBinding(TrajGenPtr trajectoryGenerator,
                            OpenSoT::Task<yarp::sig::Matrix, yarp::sig::Vector>::TaskPtr task,
                            double maximumAllowedError = 5e-2,
                            double convergenceTolerance = 5e-4,
                            ConvergencePolicy convergencePolicy = CONVERGE_ON_CARTESIAN_ERROR_SMALL) :
                    convergenceTolerance(convergenceTolerance),
                    maximumAllowedError(maximumAllowedError),
                    task(task),
                    trajectoryGenerator(trajectoryGenerator),
                    convergencePolicy(convergencePolicy)
                {}
            };

            typedef std::list<TrajBinding> TrajectoryBindings;
            typedef boost::shared_ptr<OpenSoT::Previewer<TrajectoryGenerator>> Ptr;
            typedef boost::accumulators::accumulator_set<double,
                                                        boost::accumulators::stats<boost::accumulators::tag::rolling_mean>
                                                        > Accumulator;

            /**
             * @brief The CartesianError struct is an utility struct to store cartesian errors
             */
            struct CartesianError { yarp::sig::Vector positionError; yarp::sig::Vector orientationError; double Ko;
                                    CartesianError() :
                                        positionError(3,0.0), orientationError(3,0.0), Ko(1.0) {}
                                    double getPositionErrorNorm() {
                                        return yarp::math::norm(positionError); }
                                    double getOrientationErrorNorm() {
                                        return yarp::math::norm(orientationError); }
                                    double getNorm() {
                                        using namespace yarp::math;
                                        return std::sqrt(std::pow(getPositionErrorNorm(),2) +
                                                         std::pow(Ko*getOrientationErrorNorm(),2)); }
                                  };
            /**
             * @brief The ErrorLog struct is an utility struct that logs at every update step
             *        the distance to the goal as a CartesianError (i.e., desired pose at the
             *        final time of the trajectory) and the norm of the current error between
             *        actual pose and desired pose at the current time for a single task for
             *        which a trajectory binding exists
             */
            struct ErrorLog {
                /**
                 * @brief accumulator an accumulator that stores error values at every instant and computes
                 *        the moving average of the error norms over a window set by window_size
                 */
                Accumulator accumulator;
                /**
                 * @brief previousMean moving average of the error norms for each task
                 */
                double previousMean;

                /**
                 * @brief lastError previous error in norm
                 */
                double lastError;

                /**
                 * @brief lastCartesianError previous cartesian error
                 */
                CartesianError lastCartesianError;

                /**
                 * @brief errorToGoal cartesian error to goal
                 */
                CartesianError errorToGoal;

                ErrorLog(int window_size = 100) :
                    accumulator(boost::accumulators::tag::rolling_mean::window_size = window_size),
                    previousMean(0.0), lastError(0.0) {}

                void update(double error)
                {
                    lastError = error;
                    previousMean = this->getRollingMean();
                    accumulator(lastError);
                }

                void update(CartesianError error, CartesianError toGoal)
                {
                    this->update(error.getNorm());
                    errorToGoal = toGoal;
                    lastCartesianError = error;
                }

                double getRollingMean()
                {
                    return boost::accumulators::extract::rolling_mean(accumulator);
                }
            };

            typedef std::map<OpenSoT::Task<yarp::sig::Matrix, yarp::sig::Vector>*,
                             ErrorLog> ErrorMap;
            typedef std::map<OpenSoT::Task<yarp::sig::Matrix, yarp::sig::Vector>*,
                             KDL::Frame>  CartesianNodes;

            /**
             * @brief The Results struct is used to store information about the preview.
             *        We store the trajectory and all detected failures
             */
            struct Results
            {
            private:
                /**
                 * @brief dT integration time of the Solver, used by operator+()
                 */
                double dT;

            public:

                /**
                 * @brief num_failures number of failures along the whole preview
                 */
                int num_failures;

                Results() : dT(0.0), num_failures(0) {}

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

                struct LogEntry
                {
                    // importing CartesianError type
                    typedef OpenSoT::Previewer<TrajectoryGenerator>::CartesianError CartesianError;

                    /**
                     * @brief q joint posiitons
                     */
                    yarp::sig::Vector q;

                    /**
                     * @brief failures a vector of failures
                     */
                    std::list<Reason> failures;
                    /**
                     * @brief errors is a map holding Cartesian error information
                     *        about every task for which we set a trajectory binding
                     */
                    std::map<OpenSoT::Task<yarp::sig::Matrix, yarp::sig::Vector>*,
                             CartesianError> errors;
                };

                /**
                 * @brief LogMap maps times to logs
                 */
                typedef std::map<double, LogEntry> LogMap;

                /**
                 * @brief log results log. Contains trajectory,
                 * cartesian errors, and trajectory failures
                 * (unbounded errors or self-collision)
                 */
                LogMap log;

                void logFailure(double time, Reason reason)
                {
                    if(log.count(time) == 0) log[time] = LogEntry();
                    log[time].failures.push_back(reason);
                    ++num_failures;
                }

                void logTrajectory(double time, const yarp::sig::Vector& q)
                {
                    if(log.count(time) == 0) log[time] = LogEntry();
                    log[time].q = q;
                }

                void logErrors(double time, ErrorMap errors)
                {
                    if(log.count(time) == 0) log[time] = LogEntry();
                    for(typename ErrorMap::iterator it = errors.begin();
                        it != errors.end(); ++it) {
                        log[time].errors[it->first] = it->second.lastCartesianError;
                    }
                }

                const Results operator+(const Results& res) const
                {
                    Results final;

                    final.log = this->log;
                    final.num_failures = this->num_failures+res.num_failures;
                    final.dT = this->dT;

                    double finalTime = 0.0;
                    if(this->log.size() > 0)
                        finalTime = this->log.rbegin()->first + this->dT;
                    // copying all trajectory information
                    for(typename LogMap::const_iterator it =
                        res.log.begin(); it != res.log.end(); ++it)
                        final.log[finalTime+it->first] = it->second;

                    return final;
                }
            };

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
                    if(cartesianErrors[b->task.get()].lastError > b->maximumAllowedError)
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
            bool cartesianErrorDecreasing(const TrajBinding& b, double threshold = 1e-9)
            {
                if(cartesianErrors[b.task.get()].previousMean - cartesianErrors[b.task.get()].getRollingMean() < threshold)
                    return false;
                return true;
            }

            /**
             * @brief cartesianErrorConverged checks whether cartesian error is smaller
             *        than the convergence tolerance.
             *        We check position error and orientation error separately
             *        against the threshold, so that we detect convergence when
             *        positionErrorNorm <= threshold && orientationErrorNorm <= threshold
             * @return true if cartesian error converged on all tasks
             */
            bool cartesianErrorConverged(const TrajBinding& b)
            {
                if(cartesianErrors[b.task.get()].errorToGoal.getPositionErrorNorm() > b.convergenceTolerance ||
                   cartesianErrors[b.task.get()].errorToGoal.getOrientationErrorNorm() > b.convergenceTolerance)
                    return false;
                return true;
            }

            /**
             * @brief cartesianPoseChanged checks whether the cartesian pose for a certain task changed too much
             * @param task the task for which to check the cartesian pose or position
             * @param threshold the threshold in norm
             * @return true if the cartesian pose or position changed more than threshold, in norm.
             * Notice it will also return true if it was never called, i.e. the first time it gets called.
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
                    bool res = YarptoKDL(yf,f);
                    assert(res && "error converting from yarp frame to kdl::Frame");
                }
                else if(isCoM(task))
                {
                    yv = asCoM(task)->getActualPosition();
                    f.p.x(yv(0));
                    f.p.y(yv(1));
                    f.p.z(yv(2));
                }

                if(cartesianNodes.count(task.get()) == 0)
                {
                    cartesianNodes[task.get()] = f;
                    return true;
                } else {

                    yfOld.resize(4,4);
                    bool res = KDLtoYarp_position(cartesianNodes[task.get()], yfOld);
                    assert(res && "Error transforming a kdl::Frame into  a yarp 4x4 matrix");
                    res = KDLtoYarp(cartesianNodes[task.get()].p, yvOld);
                    assert(res && "Error transforming a kdl::Vector into a yarp vector");

                    if(isCartesian(task))
                    {
                        using namespace yarp::math;
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
                    };

                    if(error > threshold)
                        cartesianNodes[task.get()] = f;

                }

                return (error > threshold);
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
             * @brief converged checks whether all tasks succesfully converged according on convergence policy
             * @return true if all tasks converged according to their convergence policy and cartesian/joint space errors
             */
            bool converged()
            {
                bool allTasksConverged = true;
                for(typename TrajectoryBindings::iterator b = bindings.begin();
                    b != bindings.end();
                    ++b)
                {
                    if(b->convergencePolicy == TrajBinding::CONVERGE_ON_CARTESIAN_ERROR_SMALL)
                    {
                        if(!trajectoryCompleted(*b) || !cartesianErrorConverged(*b))
                            allTasksConverged = false;
                    }
                    else if(b->convergencePolicy == TrajBinding::CONVERGE_ON_CARTESIAN_ERROR_SMALL_AND_NOT_DECREASING)
                    {
                         if(!trajectoryCompleted(*b) || cartesianErrorDecreasing(*b) || !(cartesianErrorConverged(*b)))
                             allTasksConverged = false;
                    }
                    else if(b->convergencePolicy == TrajBinding::CONVERGE_ON_CARTESIAN_ERROR_AND_NULL_STABLE)
                    {
                         if(!trajectoryCompleted(*b) || !((cartesianErrorConverged(*b) && !cartesianErrorDecreasing(*b)) && jointSpaceConfigurationConverged()))
                             allTasksConverged = false;
                    }
                }
                return allTasksConverged;
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
             * @brief jointSpaceConfigurationConverged checks whether the solver has converged
             * @param threshold comparable with the norm of the joint position change dq
             * @return true if the robot is stable (converged also to its null-space goals)
             */
            bool jointSpaceConfigurationConverged(double threshold = 1e-9)
            {
                return yarp::math::norm(dq) < threshold;
            }

            /**
             * @brief resetPreviewer clears the history of cartesian errors,
             * cartesian nodes, joint space node, resets q, dq, t
             */
            void resetPreviewer()
            {
                cartesianErrors.clear();
                cartesianNodes.clear();

                t = 0.0;

                q = model.iDyn3_model.getAng();
                dq.resize(q.size()); dq.zero();
                qNode.resize(q.size()); qNode.zero();
            }

            /**
             * @brief shouldCheckSelfCollision checks whether cartesian poses changed or
             * joint position changed too much; in which case, we force a collision check
             * @return true if we need to call a self-collision check
             */
            bool shouldCheckSelfCollision(double threshold = 2e-2)
            {
                bool cartesianPosesChanged = false;
                bool qChanged = false;

                for(typename TrajectoryBindings::iterator b = bindings.begin();
                    b != bindings.end();
                    ++b)
                {
                    if(cartesianPoseChanged(b->task,threshold))
                    {
                        cartesianPosesChanged = true;
                        std::cout << b->task->getTaskID()
                                  << ".DeltaX > " << threshold << " - ";

                    }
                }

                qChanged = jointSpaceConfigurationChanged(threshold);
                if(qChanged)
                    std::cout << "DeltaQ > " << threshold << " - ";
                return cartesianPosesChanged || qChanged;
            }

            /**
             * @brief trajectoryCompleted returns true when the trajectory has been played to the end
             * @return true if current simulation time is equal or greated than trajectory duration
             */
            bool trajectoryCompleted(const TrajBinding& b)
            {
                if(t >= b.trajectoryGenerator->Duration())
                    return true;
                return false;
            }

            /**
             * @brief trajectoriesExpired checks whether we simulated forward in time enough
             *        to ecompass the duratino of all trajectories.
             * @param safetyMargin a percentage of the longest trajectory duration we wish to wait for.
             * @return true if t has surpassed the longest trajectory duration by safetyMargin (in percentage)
             */
            bool trajectoriesExpired(double safetyMargin = 1.5)
            {
                bool trajCompleted = true;

                for(typename TrajectoryBindings::iterator b = bindings.begin();
                    b != bindings.end();
                    ++b)
                {
                    if(t < b->trajectoryGenerator->Duration()*safetyMargin)
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
                    if(cartesianErrors.count(b->task.get()) == 0)
                    {
                        cartesianErrors[b->task.get()] =
                            ErrorLog(std::ceil(windowSizeInSecs/dT));
                    }

                    KDL::Frame goal = b->trajectoryGenerator->Pos(
                        b->trajectoryGenerator->Duration());

                    double previousTime = t;
                    if(previousTime > 0) previousTime -= dT;

                    KDL::Frame desired = b->trajectoryGenerator->Pos(previousTime);

                    if(isCartesian(b->task))
                    {
                        CartesianError toGoal;
                        CartesianError toDesired;
                        yarp::sig::Vector positionError(3,0.0), orientationError(3,0.0);

                        /* updating CartesianError to goal */
                        cartesian_utils::computeCartesianError(asCartesian(b->task)->getActualPose(), KDLtoYarp_position(goal),
                                                               positionError,
                                                               orientationError);

                        toGoal.positionError = positionError;
                        toGoal.orientationError = orientationError;
                        toGoal.Ko = asCartesian(b->task)->getOrientationErrorGain();

                        /* updating CartesianError to desired pose at previous time step */
                        cartesian_utils::computeCartesianError(asCartesian(b->task)->getActualPose(), KDLtoYarp_position(desired),
                                                               positionError,
                                                               orientationError);

                        toDesired.positionError = positionError;
                        toDesired.orientationError = orientationError;
                        toDesired.Ko = asCartesian(b->task)->getOrientationErrorGain();

                        if(std::fabs(toDesired.getNorm() - yarp::math::norm(asCartesian(b->task)->getError())) > 1e-12) {
                            std::cout << std::endl << toDesired.getNorm() << " vs " << yarp::math::norm(asCartesian(b->task)->getError()) << std::endl;
                            std::cout << std::endl << KDLtoYarp_position(desired).toString()
                                      << std::endl << " vs "
                                      << std::endl << asCartesian(b->task)->getReference().toString()
                                      << "@t" << previousTime << std::endl;
                        }
                        assert(std::fabs(toDesired.getNorm() - yarp::math::norm(asCartesian(b->task)->getError())) < 1e-12 &&
                               "Error: discrepancy between CartesianError and error computed by Cartesian->getError()");

                        cartesianErrors[b->task.get()].update(toDesired, toGoal);
                    }

                    else if(isCoM(b->task))
                    {
                        CartesianError toGoal;
                        CartesianError toDesired;

                        toGoal.positionError = asCoM(b->task)->getActualPosition() - KDLtoYarp(goal.p);
                        toDesired.positionError = (asCoM(b->task)->getActualPosition() - KDLtoYarp(desired.p));

                        assert(std::fabs(toDesired.getNorm() - yarp::math::norm(asCoM(b->task)->getError())) < 1e-12 &&
                               "Error: discrepancy between CartesianError and error computed by CoM->getError()");

                        cartesianErrors[b->task.get()].update(toDesired, toGoal);
                    }
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
                    bool res = KDLtoYarp(f.p, yv);
                    assert(res && "Error converting kdl::Vector in yarp::sig::Vector");

                    {
                    using namespace yarp::math;
                        if(isCartesian(b->task))
                            asCartesian(b->task)->setReference(yf,yt*dT);
                        else if(isCoM(b->task))
                            asCoM(b->task)->setReference(yv, yt*dT);
                    }
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
                this->resetPreviewer();
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
                this->resetPreviewer();
            }

            /**
             * @brief resetModel resets the internal model with the status of the specified one, and resets internal time.
             * @param idyn the model representing the current status of the robot
             */
            void resetModel(iDynUtils& idyn)
            {

                model.updateiDyn3Model( idyn.iDyn3_model.getAng(),
                                        idyn.iDyn3_model.getDAng(),
                                        idyn.iDyn3_model.getD2Ang(), true);
                model.switchAnchor(idyn.getAnchor());
                model.iDyn3_model.setFloatingBaseLink(idyn.iDyn3_model.getFloatingBaseLink());
                model.setAnchor_T_World(idyn.getAnchor_T_World());

                this->resetPreviewer();
            }

            /**
             * @brief check simulates forward for time seconds. If time is infinity, it will simulate forward
             *              till the task converges in task space and joint space
             * @param time the time to simulate, in seconds. Notice that if the time is smaller than the trajectory time,
             *        the previewer will return false, since we didn't complete the trajectory.
             *        You can still check the results to see if any fault occured
             * @param max_retries maximum number of times to retries solve() if errors occur
             * @return true if during the whole trajectory the tracking was always lower than the specified threshold,
             *              and no self-collisions were detected.
             *              If the previewer is asked to simulate for a time lower than the longest trajectory duration,
             *              false will be returned. To get finer informations you should check the results structure
             */
            bool check(const double time = std::numeric_limits<double>::infinity(),
                       const unsigned int max_retries = 3,
                       Results* results = NULL,
                       PreviewerCallBack* call_back = NULL)
            {
                this->resetPreviewer();

                bool finished = false;
                unsigned int retries = 0;
                bool check_ok = true;

                // updating references for correct initial error display
                updateReferences();
                /**
                  Update the tasks to refresh the reference
                  */
                for(typename TrajectoryBindings::iterator b = bindings.begin();
                    b != bindings.end();
                    ++b)
                {
                    b->task->update(q);
                }

                while(!finished) {
                    if(time == std::numeric_limits<double>::infinity())
                    {
                        bool trajectoriesCheck = trajectoriesExpired();
                        bool convergenceCheck = converged();
                        if(trajectoriesCheck)
                            std::cout << "Trajectories are expired. Stopping." << std::endl;
                        if(convergenceCheck)
                            std::cout << "IK Converged. Stopping." << std::endl;
                        finished =  trajectoriesCheck ||
                                    convergenceCheck;
                    }
                    else
                        finished = (t >= time);

                    model.updateiDyn3Model(q, true);

                    // update the stack to get new error statistics
                    autostack->update(q);

                    updateErrorsStatistics();

                    if(call_back != NULL)
                        call_back->callback(q);

                    updateReferences();
                    /**
                      Update the tasks to refresh the reference
                      */
                    for(typename TrajectoryBindings::iterator b = bindings.begin();
                        b != bindings.end();
                        ++b)
                    {
                        b->task->update(q);
                    }

                    if(shouldCheckSelfCollision()) {
                        std::cout << " collision check ";
                        if(model.checkSelfCollision())
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

                    do {
                        if(solver->solve(dq))
                        {
                            retries = 0;

                            q += dq;
                            std::cout << std::endl << "t:" << t << " ";
                            std::cout.flush();

                            if(results != NULL) {
                                results->logErrors(t, cartesianErrors);
                                results->logTrajectory(t, q);
                            }

                            t += dT;

                        } else ++retries;
                      // do not cycle if retries == 0 (success),
                      // but also cycle for a maximum of max_retries times
                    } while(retries > 0 && retries <= max_retries);

                    if(retries >= max_retries) {
                        finished = true;
                        check_ok = false;
                        std::cerr << "Error with solver: could not solve" << std::endl;
                        continue;
                    }
                }

                std::cout << std::endl;
                std::cout.flush();

                if(!converged()) check_ok = false;

                return check_ok;
            }
    };
}

#endif

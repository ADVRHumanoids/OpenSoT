#include "JointLimitsPSAP.h"
#include <OpenSoT/constraints/acceleration/VelocityLimits.h>
#include <OpenSoT/constraints/GenericConstraint.h>
#include <OpenSoT/tasks/acceleration/Postural.h>
#include <OpenSoT/utils/AutoStack.h>
#include <xbot2_interface/xbotinterface2.h>
#include <xbot2_interface/logger.h>
#include <cmath>
#include <OpenSoT/solvers/iHQP.h>
#include <matlogger2/matlogger2.h>

std::string _path_to_cfg = OPENSOT_EXAMPLE_PATH "configs/coman/configs/config_coman_RBDL.yaml";

#define P 100.



class ExampleJointLimitsPSAP
{
public:
    ExampleJointLimitsPSAP()
    {
        /**
         * @brief Create and get a model Pointer from config
         */
        _model_ptr = XBot::ModelInterface::getModel(_path_to_cfg);

        /**
         * @brief Create a logger
         */
        logger = XBot::MatLogger2::MakeLogger("/tmp/exampleJointLimitsPSAP_acceleration");
        logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);

    }

    /**
     * @brief getGoodInitialPosition a starting configuration
     * @param _model
     * @return q
     */
    Eigen::VectorXd getGoodInitialPosition(XBot::ModelInterface::Ptr _model) {
        Eigen::VectorXd _q(_model->getJointNum());
        _q.setZero(_q.size());

        _q[_model->getDofIndex("RHipSag")] = -25.0*M_PI/180.0;
        _q[_model->getDofIndex("RKneeSag")] = 50.0*M_PI/180.0;
        _q[_model->getDofIndex("RAnkSag")] = -25.0*M_PI/180.0;

        _q[_model->getDofIndex("LHipSag")] = -25.0*M_PI/180.0;
        _q[_model->getDofIndex("LKneeSag")] = 50.0*M_PI/180.0;
        _q[_model->getDofIndex("LAnkSag")] = -25.0*M_PI/180.0;

        _q[_model->getDofIndex("LShSag")] =  20.0*M_PI/180.0;
        _q[_model->getDofIndex("LShLat")] = 20.0*M_PI/180.0;
        _q[_model->getDofIndex("LShYaw")] = -15.0*M_PI/180.0;
        _q[_model->getDofIndex("LElbj")] = -80.0*M_PI/180.0;

        _q[_model->getDofIndex("RShSag")] =  20.0*M_PI/180.0;
        _q[_model->getDofIndex("RShLat")] = -20.0*M_PI/180.0;
        _q[_model->getDofIndex("RShYaw")] = 15.0*M_PI/180.0;
        _q[_model->getDofIndex("RElbj")] = -80.0*M_PI/180.0;

        return _q;
    }

    /**
     * @brief createStack set up a simple stack using the autostack
     */
    void createStack()
    {
        /**
         * @brief Initialize zero joint velocity in the model
         */
        qdot.setZero(_model_ptr->getJointNum());
        _model_ptr->setJointVelocity(qdot);

        /**
         * @brief Initialize robot position with q from setGoodInitialPosition()
         */
        q = getGoodInitialPosition(_model_ptr);
        _model_ptr->setJointPosition(q);

        /**
         * @brief Updates model with q and qdot set
         */
        _model_ptr->update();

        /**
         * @brief Creates the variable qddot
         */
        qddot = OpenSoT::AffineHelper::Identity(_model_ptr->getJointNum());

        /**
         * @brief Set joint acceleration limits
         */
        acc_lims.setOnes(_model_ptr->getJointNum());
        acc_lims *= 20.;
        
        /**
         * @brief Loop time
         */
        dT = 0.001;

        /**
         * @brief Set joint velocity limits
         */
        qdotMax = M_PI;
        Eigen::VectorXd vel_lims;
        vel_lims.setOnes(_model_ptr->getJointNum());
        vel_lims *= qdotMax;

        /**
         * @brief Get joint position limits from model
         */
        _model_ptr->getJointLimits(qmin, qmax);

        /**
         * @brief Creates JointLimitsPSAP constraint using limits and qddot variable
         */
        jointLimits = std::make_shared<OpenSoT::constraints::acceleration::JointLimitsPSAP>(
                    *_model_ptr, qddot, qmax, qmin, vel_lims, acc_lims, dT);
        jointLimits->setPStepAheadPredictor(P);

        /**
         * @brief Creates postural task using qddot variable and set position error gain lambda
         * @note velocity gain lambda2 is automatically set to 2*std::sqrt(lambda)
         */
        postural = std::make_shared<OpenSoT::tasks::acceleration::Postural>(*_model_ptr, qddot);
        postural->setLambda(5000.);

        /**
         * @brief Creates a stack composed by the postural task constrained by jointLimits constraint
         */
        autostack = std::make_shared<OpenSoT::AutoStack>(postural);
        autostack<<jointLimits;

        /**
         * @brief Creates solver iHQP
         */
        solver = std::make_shared<OpenSoT::solvers::iHQP>(autostack->getStack(), autostack->getBounds(), 1e6);

    }

    virtual ~ExampleJointLimitsPSAP() {

    }

    /**
     * @brief checkConstraints
     * @param qddot
     * @param EPS
     * @return 0 if passed, 1 otherwise
     */
    int checkConstraints(const Eigen::VectorXd& qddot, double EPS)
    {

        for(unsigned int i = 0; i < qddot.size(); ++i)
        {
            double x = qddot[i];
            double x_min = -acc_lims[i];
            double x_max = acc_lims[i];

            if( !((x-x_min >= -EPS) && (x-x_max <= EPS)) ){
                std::cout<<"joint acc violated @i:"<<i<<" "<<x_min<<" <= "<<x<<" <= "<<x_max<<std::endl;
                return 1;}

            x_min = -qdotMax;
            x_max = qdotMax;
            x = qdot[i];
            if( !((x-x_min >= -EPS) && (x-x_max <= EPS)) ){
                std::cout<<"joint vel violated @i:"<<i<<" "<<x_min<<" <= "<<x<<" <= "<<x_max<<std::endl;
                return 1;}

            if( !((q[i]-qmin[i] >= -EPS) && (q[i]-qmax[i] <= EPS)) ){
                std::cout<<"joint limits @i:"<<i<<" "<<qmin[i]<<" <= "<<q[i]<<" <= "<<qmax[i]<<std::endl;
                return 1;}
        }
        return 0;
    }

    /**
     * @brief run
     * @return 0 if passed, 1 otherwise
     */
    int run_example()
    {
        this->createStack();

        /**
         * @brief Creates a reference in joint space and set to postural task
         */
        Eigen::VectorXd qref(this->q.size());
        qref.setZero();
        this->postural->setReference(qref);

        /**
         * @brief Control loop over 3 seconds to reach the qref
         */
        double T = 3;
        for(unsigned int  i = 0; i < T/this->dT; ++i)
        {
            /**
             * @brief 1. Model pointer is updated with latest position and velocities
             * @note Joint states may be measured or integrated from solution
             */
            this->_model_ptr->setJointVelocity(this->qdot);
            this->_model_ptr->setJointPosition(this->q);
            this->_model_ptr->update();

            /**
             * @brief 2. Autostack is updated
             * @note Acceleration based IK uses joint position and velocities retrieved from the internal model,
             * for this reason is not needed to pass anything to the update of the autostack
             * @note Internally all the stacks are updated using the updated model
             */
            this->autostack->update(Eigen::VectorXd(0));

            /**
             * @brief 3. Call solve from solver and computes next desired joint acceleration
             */
            Eigen::VectorXd qddot;
            if(!this->solver->solve(qddot))
            {
                std::cout<<"OpenSoT unable to solve!"<<std::endl;
                return 1;
            }

            /**
             * @brief Integrates solution to compute next joint position and velocities assuming constant acceleration
             */
            this->q += this->qdot*this->dT + 0.5*qddot*this->dT*this->dT;
            this->qdot += qddot*this->dT;

            /**
             * @brief Call check constraints to check if joint positions, velocities, and accelerations are kept.
             * @note In this example we are using a naive approach with weak constraint feasibility
             */
            if(this->checkConstraints(qddot, 3e-2) == 1)
                return 1;
        }

        /**
         * @brief Control loop over 10 seconds to track sinusoidal references breaking joint position limits
         */
        T = 10;
        Eigen::VectorXd q0 = this->q;
        for(unsigned int  i = 0; i < T/this->dT; ++i)
        {
            /**
             * @brief Computes sinusoidal reference and set to postural
             */
            qref.setOnes(qref.size());
            qref *= 3.* std::sin(i*this->dT);
            qref += q0;
            this->postural->setReference(qref);

            /**
             * @brief 1. Model pointer is updated with latest position and velocities
             * @note Joint states may be measured or integrated from solution
             */
            this->_model_ptr->setJointVelocity(this->qdot);
            this->_model_ptr->setJointPosition(this->q);
            this->_model_ptr->update();

            /**
             * @brief 2. Autostack is updated
             * @note Acceleration based IK uses joint position and velocities retrieved from the internal model,
             * for this reason is not needed to pass anything to the update of the autostack
             * @note Internally all the stacks are updated using the updated model
             */
            this->autostack->update(Eigen::VectorXd(0));
            this->autostack->log(this->logger);

            /**
             * @brief 3. Call solve from solver and computes next desired joint acceleration
             */
            Eigen::VectorXd qddot;
            if(!this->solver->solve(qddot))
            {
                std::cout<<"OpenSoT unable to solve!"<<std::endl;
                return 1;
            }

            /**
             * @brief Integrates solution to compute next joint position and velocities assuming constant acceleration
             */
            this->q += this->qdot*this->dT + 0.5*qddot*this->dT*this->dT;
            this->qdot += qddot*this->dT;

            /**
             * @brief Add and save variables in the logger
             */
            this->logger->add("qddot", qddot);
            this->logger->add("qdot", qdot);
            this->logger->add("q", q);
            this->logger->add("qmax", qmax);
            this->logger->add("qmin", qmin);
            this->logger->add("qdotmax", this->qdotMax);
            this->logger->add("qdotmin", -this->qdotMax);
            this->logger->add("qddotmax", this->acc_lims);
            this->logger->add("qddotmin", -this->acc_lims);
            this->logger->add("qref", qref);

            /**
             * @brief Call check constraints to check if joint positions, velocities, and accelerations are kept.
             * @note In this example we are using a naive approach with weak constraint feasibility
             */
            if(this->checkConstraints(qddot, 3e-2) == 1)
                return 1;
        }
        return 0;
    }

    const Eigen::VectorXd& getq() const {return q;}


private:
    XBot::ModelInterface::Ptr _model_ptr;
    OpenSoT::constraints::acceleration::JointLimitsPSAP::Ptr jointLimits;
    OpenSoT::tasks::acceleration::Postural::Ptr postural;
    OpenSoT::AffineHelper qddot;
    OpenSoT::AutoStack::Ptr autostack;
    double dT, qdotMax;
    Eigen::VectorXd q, qdot, qmin, qmax, acc_lims;
    XBot::MatLogger2::Ptr logger;
    OpenSoT::solvers::iHQP::Ptr solver;
};




int main(int argc, char **argv) {
  ExampleJointLimitsPSAP example;
  int result = example.run_example();
  if(result == 1)
      std::cout<<"There were errors in example.run_example()"<<std::endl;
  else
  {
      std::cout<<"Final configuration is: \n"<<example.getq().transpose()<<std::endl;
      std::cout<<"Full data can be found at /tmp/exampleJointLimitsPSAP_acceleration_<data>.mat"<<std::endl;
  }
  return result;
}

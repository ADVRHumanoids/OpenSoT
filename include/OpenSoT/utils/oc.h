#ifndef __OPENSOT_OC_H__
#define __OPENSOT_OC_H__

#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <xbot2_interface/xbotinterface2.h>
#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/utils/Affine.h>
#include <OpenSoT/tasks/Aggregated.h>
#include <OpenSoT/tasks/GenericTask.h>


namespace OpenSoT {

class ocp{
    public:
        typedef std::shared_ptr<ocp> Ptr;

        struct Stage{
            typedef std::shared_ptr<Stage> Ptr;

            Stage()
            {

            }

            bool isFinalStage()
            {
                if(dynamics_derivative)
                    return false;
                return true;
            }

            void update(const Eigen::VectorXd& x0, const Eigen::VectorXd& u0)
            {
                Eigen::VectorXd w0;
                if(u)
                {
                    w0.resize(x0.size() + u0.size());
                    w0.head(x0.size()) = x0;
                    w0.tail(u0.size()) = u0;
                }
                else
                {
                    w0 = x0;
                }


                //1 update model
                model->setJointPosition(q->getValue(w0));
                model->setJointVelocity(v->getValue(w0));
                model->update();

                //2 update and evaluate state variables
                x->update();
                x->getValue(w0);

                //3 update and evaluate control variables (may depends on model)
                if(u)
                {
                    u->update();
                    u->getValue(w0);
                };

                //4 update and evaluate variables
                for(unsigned int i = 0; i < variables.size(); ++i)
                {
                    variables[i]->update();
                    variables[i]->getValue(w0);
                }

                //3 update dynamics_derivative
                if(dynamics_derivative)
                    dynamics_derivative->update();

                //4 update stack
                if(stack)
                    stack->update();

            }

            std::shared_ptr<XBot::ModelInterface> model;
            std::vector<std::shared_ptr<AffineHelper>> variables;
            tasks::Aggregated::TaskPtr dynamics_derivative;
            std::shared_ptr<AffineHelper> x, u, q, v;
            AutoStack::Ptr stack;
        };

        typedef std::vector<Stage::Ptr> horizon;


        ocp();

        void addStage(Stage::Ptr stage);

        void update(const std::vector<Eigen::VectorXd>& x0, const std::vector<Eigen::VectorXd>& u0);

        Stage::Ptr stage(const unsigned int i){return _stages[i];}
        horizon& getHorizon(){return _stages;}

        unsigned int getNumberOfNodes();


    private:
        horizon _stages;

};

}

#endif

#ifndef __OPENSOT_OC_H__
#define __OPENSOT_OC_H__

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <memory>
#include <xbot2_interface/xbotinterface2.h>
#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/utils/Affine.h>
#include <OpenSoT/tasks/Aggregated.h>
#include <OpenSoT/tasks/GenericTask.h>
#include <OpenSoT/utils/LieGroupsUtils.h>
#include <OpenSoT/oc/Manifolds.h>


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
                _w0.resize(x->getInputSize());
                _w0.setZero();
                _w0.head(x0.size()) = x0;
                _w0.tail(u0.size()) = u0;


                //1 update model
                model->setJointPosition(q->getValue(_w0));
                model->setJointVelocity(v->getValue(_w0));
                if(this->a)
                    model->setJointAcceleration(a->getValue(_w0));
                model->update();

                //2 update and evaluate state variables
                x->update();
                x->getValue(_w0);

                //3 update and evaluate control variables (may depends on model)
                if(this->u)
                {
                    u->update();
                    u->getValue(_w0);
                };

                //4 update and evaluate variables
                for(unsigned int i = 0; i < variables.size(); ++i)
                {
                    variables[i]->update();
                    variables[i]->getValue(_w0);
                }

                //3 update dynamics_derivative
                if(this->dynamics_derivative)
                    dynamics_derivative->update();

                //4 update stack
                if(this->stack)
                    stack->update();

            }

            double stage_cost()
            {
                double cost = 0.;
                if(this->stack)
                {
                    cost = 0.5 * (stack->getStack()[0]->getb().transpose() * stack->getStack()[0]->getWb())[0];
                }
                return cost;
            }

            Eigen::VectorXd stage_dcost_dw()
            {
                // _dw0.resize(this->dx->getInputSize());
                // _dw0.setZero();
                // _dw0.head(dx.size()) = dx;
                // _dw0.tail(du.size()) = du;

                // this->dx->getValue(_dw0);
                // if(this->du)
                //     this->du->getValue(_dw0);

                Eigen::VectorXd der;
                if(stack)
                {
                    der = ((-1.0 * stack->getStack()[0]->getA().transpose() * stack->getStack()[0]->getWb()).transpose());
                }

                return der;
            }

            double stage_constraint_violation()
            {
                double inf_norm =0.;
                if(stack->getBounds()->getAineq().rows() > 0) //there are constraints
                {
                    
                    Eigen::VectorXd lviolations = ((stack->getBounds()->getbLowerBound()).cwiseMax(0.0));
                    Eigen::VectorXd uviolations = ((-stack->getBounds()->getbUpperBound()).cwiseMax(0.0));

                    inf_norm = std::max(uviolations.maxCoeff(), lviolations.maxCoeff());
                }

                return inf_norm;
            }

            double stage_dynamics_defect()
            {
                double inf_norm = 0.;
                if(this->dynamics_derivative)
                {
                    inf_norm = this->dynamics_derivative->getb().cwiseAbs().maxCoeff();
                }
                return inf_norm;

            }


            std::shared_ptr<XBot::ModelInterface> model;
            std::vector<std::shared_ptr<AffineHelper>> variables;
            tasks::Aggregated::TaskPtr dynamics_derivative;
            std::shared_ptr<AffineHelper> x, u, q, v, a, dx, du;
            AutoStack::Ptr stack;
            Space::Ptr state_space;

            private:
                Eigen::VectorXd _w0, _dw0;
        };

        typedef std::vector<Stage::Ptr> horizon;


        ocp();

        /**
         * @brief cost compute cumulative costs for all stages
         * @return cumulative cost
         */
        double cost();
        double dynamics_defect();
        double constraint_violation();


        double computeConstraintViolation();

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

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

            Stage(){}

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
                if(a)
                    model->setJointAcceleration(a->getValue(_w0));
                model->update();

                //2 update and evaluate state variables
                x->update();
                x->getValue(_w0);

                if (xdot){
                    xdot->update();
                    xdot->getValue(_w0);
                }
                

                //3 update and evaluate control variables (may depends on model)
                if(u)
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
                if(dynamics_derivative)
                    dynamics_derivative->update();

                //4 update stack
                if(stack)
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

                Eigen::VectorXd der = Eigen::VectorXd::Zero(dx->getInputSize());
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

            Eigen::VectorXd stage_dviolation_dw(double beta = 10.0)
            {
                Eigen::VectorXd gradient = Eigen::VectorXd::Zero(dx->getInputSize());
                
                if(stack->getBounds()->getAineq().rows() == 0)
                    return gradient;
                
                Eigen::VectorXd lviolations = ((stack->getBounds()->getbLowerBound()).cwiseMax(0.0));
                Eigen::VectorXd uviolations = ((-stack->getBounds()->getbUpperBound()).cwiseMax(0.0));
                
                // Combine all violations
                std::vector<double> all_violations;
                std::vector<int> all_indices;
                std::vector<int> all_types;
                
                for(int i = 0; i < lviolations.size(); ++i) {
                    all_violations.push_back(lviolations(i));
                    all_indices.push_back(i);
                    all_types.push_back(0);
                }
                
                for(int i = 0; i < uviolations.size(); ++i) {
                    all_violations.push_back(uviolations(i));
                    all_indices.push_back(i);
                    all_types.push_back(1);
                }
                
                // Softmax weights: w_i = exp(beta * v_i) / sum_j(exp(beta * v_j))
                double max_v = *std::max_element(all_violations.begin(), all_violations.end());
                
                std::vector<double> exp_vals(all_violations.size());
                double sum_exp = 0.0;
                
                for(size_t i = 0; i < all_violations.size(); ++i) {
                    exp_vals[i] = std::exp(beta * (all_violations[i] - max_v)); // avoid overflow
                    sum_exp += exp_vals[i];
                }
                
                const auto& Aineq = stack->getBounds()->getAineq();

                // Compute weighted gradient
                for(size_t i = 0; i < all_violations.size(); ++i) {
                    double weight = exp_vals[i] / sum_exp;
                    
                    int idx = all_indices[i];
                    int type = all_types[i];
                    
                    const auto& constraint_gradient = Aineq.row(idx);
                    
                    int sign = (type == 0) ? -1 : 1;
                    
                    gradient += weight * sign * constraint_gradient;
                    
                }
                return gradient;
            }

            double stage_dynamics_defect()
            {
                double inf_norm = 0.;
                if(dynamics_derivative)
                {
                    inf_norm = dynamics_derivative->getb().cwiseAbs().maxCoeff();
                }
                return inf_norm;

            }

            Eigen::VectorXd stage_ddefect_dw(double beta = 10.0)
            {
                Eigen::VectorXd gradient = Eigen::VectorXd::Zero(dx->getInputSize());
                
                if(!dynamics_derivative || dynamics_derivative->getb().size() == 0)
                    return gradient;
                
                Eigen::VectorXd b = dynamics_derivative->getb();
                Eigen::VectorXd abs_b = b.cwiseAbs();
                
                // Find the index of maximum absolute violation
                int max_idx;
                double max_val = abs_b.maxCoeff(&max_idx);
                
                if(max_val == 0.0)
                    return gradient;
                
                // For smooth approximation using softmax
                std::vector<double> violations;
                std::vector<int> indices;
                std::vector<int> signs;
                
                for(int i = 0; i < abs_b.size(); ++i) {
                    violations.push_back(abs_b(i));
                    indices.push_back(i);
                    signs.push_back((b(i) >= 0) ? 1 : -1);
                }
                
                // Softmax weights
                double max_v = *std::max_element(violations.begin(), violations.end());
                
                std::vector<double> exp_vals(violations.size());
                double sum_exp = 0.0;
                
                for(size_t i = 0; i < violations.size(); ++i) {
                    exp_vals[i] = std::exp(beta * (violations[i] - max_v));
                    sum_exp += exp_vals[i];
                }
                
                const auto& A = dynamics_derivative->getA(); // or appropriate matrix
                
                // Compute weighted gradient
                for(size_t i = 0; i < violations.size(); ++i) {
                    double weight = exp_vals[i] / sum_exp;
                    int idx = indices[i];
                    int sign = signs[i];
                    
                    gradient += weight * sign * A.row(idx);
                }
                
                return gradient;
            }

            std::shared_ptr<XBot::ModelInterface> model;
            std::vector<std::shared_ptr<AffineHelper>> variables;
            tasks::Aggregated::TaskPtr dynamics_derivative;
            std::shared_ptr<AffineHelper> x, xdot, u, q, v, a, dx, du;
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

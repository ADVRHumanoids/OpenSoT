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

class Space
{
public:
    typedef std::shared_ptr<Space> Ptr;

    Space(const unsigned int nq, const unsigned int nv)
    {
        _nq = nq;
        _nv = nv;
    }

    unsigned int nq(){ return _nq;}

    unsigned int nv(){ return _nv;}

    virtual void integrate(const Eigen::VectorXd& x0, const Eigen::VectorXd& dx0, Eigen::VectorXd& x1) = 0;

protected:
    unsigned int _nq;
    unsigned int _nv;
};

class VectorSpace: public Space
{
public:
    typedef std::shared_ptr<VectorSpace> Ptr;

    VectorSpace(const unsigned int dimension):
        Space(dimension, dimension)
    {}

    virtual void integrate(const Eigen::VectorXd& x0, const Eigen::VectorXd& dx0, Eigen::VectorXd& x1)
    {
        if(x0.size() != this->nq())
            throw std::runtime_error("x0.size() != _nq");
        if(x1.size() != this->nq())
            throw std::runtime_error("x1.size() != _nq");
        if(dx0.size() != this->nv())
            throw std::runtime_error("dx0.size() != _nv");
        if(x0.size() != dx0.size())
            throw std::runtime_error("x0.size() != dx0.size()");

        x1 = x0 + dx0;
    }
};

class QuaternionSpace: public Space
{
public:
    typedef std::shared_ptr<QuaternionSpace> Ptr;

    QuaternionSpace():
        Space(4, 3)
    {}

    void integrate(const Eigen::VectorXd& x0, const Eigen::VectorXd& dx0, Eigen::VectorXd& x1)
    {
        if(x0.size() != this->nq())
            throw std::runtime_error("x0.size() != _nq");
        if(x1.size() != this->nq())
            throw std::runtime_error("x1.size() != _nq");
        if(dx0.size() != this->nv())
            throw std::runtime_error("dx0.size() != _nv");

        throw std::runtime_error("sum NOT IMPLEMENTED!!!!");

    }
};

class CompositeSpace : public Space
{
private:
     std::vector<Space::Ptr> _spaces;
public:
     typedef std::shared_ptr<CompositeSpace> Ptr;

    // Constructor for manual list of Space
    CompositeSpace(const std::vector<Space::Ptr>& list):
         Space(0,0)
    {
        for(auto & state_space_representation : list)
        {
            _nq += state_space_representation->nq();
            _nv += state_space_representation->nv();
        }

        for(unsigned int i = 0; i < list.size(); ++i)
        {
            Space::Ptr spacei = list[i];
            if(auto composite = std::dynamic_pointer_cast<CompositeSpace>(spacei))
            {
                std::vector<Space::Ptr> ssr = composite->getSpaces();
                _spaces.insert(_spaces.end(), ssr.begin(), ssr.end());
            }
            else
            {
                _spaces.push_back(spacei);
            }
        }

    }

    const std::vector<Space::Ptr>& getSpaces()
    {
        return _spaces;
    }

    void integrate(const Eigen::VectorXd& x0, const Eigen::VectorXd& dx0, Eigen::VectorXd& x1)
    {
        std::unordered_map<Space::Ptr, std::pair<start_index_state_space, start_index_tangent_space>> space_indices_map = get_spaces_and_indices();
        for(auto& space : _spaces)
        {
            unsigned int x0id = space_indices_map[space].first;
            unsigned int dx0id = space_indices_map[space].second;

            Eigen::VectorXd tmp(space->nq());
            tmp.setZero();
            space->integrate(x0.segment(x0id, space->nq()), dx0.segment(dx0id, space->nv()), tmp);
            x1.segment(x0id, space->nq()) = tmp;
        }
    }

private:
    typedef unsigned int start_index_state_space;
    typedef unsigned int start_index_tangent_space;

    std::unordered_map<Space::Ptr, std::pair<start_index_state_space, start_index_tangent_space>> get_spaces_and_indices() const {
        std::unordered_map<Space::Ptr, std::pair<start_index_state_space, start_index_tangent_space>> map;
        unsigned int offset_state = 0;
        unsigned int offset_tangent = 0;

        for(unsigned int i = 0; i < _spaces.size(); ++i)
        {
            std::pair<start_index_state_space, start_index_tangent_space> tmp;
            tmp.first = offset_state;
            tmp.second = offset_tangent;

            map[_spaces[i]] = tmp;

            offset_state += _spaces[i]->nq();
            offset_tangent += _spaces[i]->nv();
        }

        return map;
    }


};




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
                model->update();

                //2 update and evaluate state variables
                x->update();
                x->getValue(_w0);

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

            double cost()
            {
                double cost = 0.;
                if(stack)
                {
                    cost = 0.5 * (stack->getStack()[0]->getb().transpose() * stack->getStack()[0]->getWb())[0];
                }
                return cost;
            }

            double der(const Eigen::MatrixXd& dx, const Eigen::MatrixXd& du)
            {
                //TODO: this needs to be better implemented for the SE(3) case:
                // _dw0.resize(_w0.size()) iff states and controls are both in R^n
                // _dw0.resize( ... ) iff states are in SE(3) x R^n, and controls are both in R^n
                _dw0.resize(_w0.size());
                _dw0.head(dx.size()) = dx;
                _dw0.tail(du.size()) = du;

                double der = 0.;
                if(stack)
                {
                    der = ((-1.0 * stack->getStack()[0]->getA().transpose() * stack->getStack()[0]->getWb()).transpose() * _dw0)[0];
                }

                return der;
            }

            std::shared_ptr<XBot::ModelInterface> model;
            std::vector<std::shared_ptr<AffineHelper>> variables;
            tasks::Aggregated::TaskPtr dynamics_derivative;
            std::shared_ptr<AffineHelper> x, u, q, v;
            AutoStack::Ptr stack;

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

        /**
         * @brief cost return cost of stage i
         * @param i
         * @return cost of stage i
         */
        double cost(const unsigned int i);
        double der(const unsigned int i, const Eigen::MatrixXd& dx, const Eigen::MatrixXd& du);

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

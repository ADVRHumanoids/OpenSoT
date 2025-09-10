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

class R3: public Space
{
public:
    typedef std::shared_ptr<R3> Ptr;

    R3(std::shared_ptr<XBot::ModelInterface> model, const std::string& base, const std::string& distal):
        Space(3, 3),
        _model(model),
        _distal(distal),
        _base(base)
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

        if(_base == _distal)
            _b_T_d.setIdentity();
        else
            _b_T_d = _model->getPose(_distal, _base);

        x1 = x0 + _b_T_d.linear() * dx0;
    }
private:
    std::shared_ptr<XBot::ModelInterface> _model;
    std::string _distal;
    std::string _base;
    Eigen::Affine3d _b_T_d;
};

class RobotSpace: public Space
{
public:
    typedef std::shared_ptr<RobotSpace> Ptr;

    RobotSpace(std::shared_ptr<XBot::ModelInterface> model):
        Space(model->getNq(), model->getNv()),
        _model(model)
    {}

    virtual void integrate(const Eigen::VectorXd& x0, const Eigen::VectorXd& dx0, Eigen::VectorXd& x1)
    {
        if(x0.size() != this->nq())
            throw std::runtime_error("x0.size() != _nq");
        if(x1.size() != this->nq())
            throw std::runtime_error("x1.size() != _nq");
        if(dx0.size() != this->nv())
            throw std::runtime_error("dx0.size() != _nv");

        x1 = _model->sum(x0, dx0);
    }
private:
    std::shared_ptr<XBot::ModelInterface> _model;

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

        x1 = Log_quat(Exp_quat(x0) * Exp3(dx0));

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
            if(auto composite = std::dynamic_pointer_cast<CompositeSpace>(list[i]))
            {
                _spaces.insert(_spaces.end(), composite->getSpaces().begin(), composite->getSpaces().end());
            }
            else
            {
                _spaces.push_back(list[i]);
            }
        }

        _x.resize(_spaces.size());

        compute_spaces_and_indices();
    }

    const std::vector<Space::Ptr>& getSpaces()
    {
        return _spaces;
    }

    void integrate(const Eigen::VectorXd& x0, const Eigen::VectorXd& dx0, Eigen::VectorXd& x1)
    {
        unsigned int i = 0;
        for(auto& space : _spaces)
        {
            unsigned int x0id = _map[space].first;
            unsigned int dx0id = _map[space].second;

            _x[i].resize(space->nq());
            _x[i].setZero();
            space->integrate(x0.segment(x0id, space->nq()), dx0.segment(dx0id, space->nv()), _x[i]);
            x1.segment(x0id, space->nq()) = _x[i];

            i+=1;
        }
    }

private:
    typedef unsigned int start_index_state_space;
    typedef unsigned int start_index_tangent_space;

    std::unordered_map<Space::Ptr, std::pair<start_index_state_space, start_index_tangent_space>> _map;

    std::vector<Eigen::VectorXd> _x;



    void compute_spaces_and_indices() {

        unsigned int offset_state = 0;
        unsigned int offset_tangent = 0;

        for(unsigned int i = 0; i < _spaces.size(); ++i)
        {
            _map.emplace(_spaces[i], std::make_pair(offset_state, offset_tangent));

            offset_state += _spaces[i]->nq();
            offset_tangent += _spaces[i]->nv();
        }
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
                _dw0.resize(this->dx->getInputSize());
                _dw0.setZero();
                _dw0.head(dx.size()) = dx;
                _dw0.tail(du.size()) = du;

                this->dx->getValue(_dw0);
                if(this->du)
                    this->du->getValue(_dw0);

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
            std::shared_ptr<AffineHelper> x, u, q, v, dx, du;
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

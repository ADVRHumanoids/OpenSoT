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


namespace OpenSoT {

// Hat operator: R^3 -> so(3)
inline Eigen::Matrix3d hat(const Eigen::Vector3d& v) {
    Eigen::Matrix3d m;
    m <<     0, -v.z(),  v.y(),
          v.z(),     0, -v.x(),
         -v.y(),  v.x(),     0;
    return m;
}

// Vee (unhat) operator: so(3) -> R^3
inline Eigen::Vector3d unhat(const Eigen::Matrix3d& m) {
    return Eigen::Vector3d(m(2,1), m(0,2), m(1,0));
}

// Exponential map from so(3) to SO(3)
inline Eigen::Matrix3d Exp3(const Eigen::Vector3d& p, double epsilon = 1e-8) {
    double theta = p.norm();
    if (theta < epsilon) {
        return Eigen::Matrix3d::Identity();
    }

    Eigen::Vector3d a = p / theta;
    double c = std::cos(theta);
    double s = std::sin(theta);

    return c * Eigen::Matrix3d::Identity()
         + (1.0 - c) * (a * a.transpose())
         + s * hat(a);
}

// Logarithm map: SO(3) -> so(3)
inline Eigen::Vector3d Log3(const Eigen::Matrix3d& R, double eps = 1e-8) {
    // Clamp argument of arccos to [-1,1]
    double cos_theta = (R.trace() - 1.0) / 2.0;
    cos_theta = std::max(-1.0, std::min(1.0, cos_theta));
    double theta = std::acos(cos_theta);

    // Case 1: theta ~ 0
    if (std::abs(theta) < eps) {
        return Eigen::Vector3d::Zero();
    }

    // Case 2: theta ~ pi
    if (std::abs(theta - M_PI) < eps) {
        double r00 = R(0,0), r11 = R(1,1), r22 = R(2,2);
        double r02 = R(0,2), r12 = R(1,2);
        double r01 = R(0,1), r21 = R(2,1);
        double r10 = R(1,0), r20 = R(2,0);

        if (std::abs(r22 + 1.0) > eps) {
            double multiplier = theta / std::sqrt(2.0 * (1.0 + r22));
            return multiplier * Eigen::Vector3d(r02, r12, 1.0 + r22);
        } else if (std::abs(r11 + 1.0) > eps) {
            double multiplier = theta / std::sqrt(2.0 * (1.0 + r11));
            return multiplier * Eigen::Vector3d(r01, 1.0 + r11, r21);
        } else if (std::abs(r00 + 1.0) > eps) {
            double multiplier = theta / std::sqrt(2.0 * (1.0 + r00));
            return multiplier * Eigen::Vector3d(1.0 + r00, r10, r20);
        }
        // In theory, one of the above should always apply.
        throw std::runtime_error("log_rotation: numerical issue near pi");
    }

    // Case 3: general case
    Eigen::Matrix3d mat = R - R.transpose();
    Eigen::Vector3d r = unhat(mat);   // uses our earlier unhat()
    return (theta / (2.0 * std::sin(theta))) * r;
}

// Convert quaternion to rotation matrix
inline Eigen::Matrix3d Exp_quat(const Eigen::Vector4d& q, bool normalize = false) {
    Eigen::Vector4d Q = q;
    if (normalize) {
        Q /= Q.norm();
    }

    // TODO: Check the quaternion convention
    double qw = Q(0);
    double qx = Q(1);
    double qy = Q(2);
    double qz = Q(3);

    Eigen::Matrix3d R;
    R(0,0) = 2.0 * (qw*qw + qx*qx) - 1.0;
    R(0,1) = 2.0 * (qx*qy - qw*qz);
    R(0,2) = 2.0 * (qx*qz + qw*qy);

    R(1,0) = 2.0 * (qx*qy + qw*qz);
    R(1,1) = 2.0 * (qw*qw + qy*qy) - 1.0;
    R(1,2) = 2.0 * (qy*qz - qw*qx);

    R(2,0) = 2.0 * (qx*qz - qw*qy);
    R(2,1) = 2.0 * (qy*qz + qw*qx);
    R(2,2) = 2.0 * (qw*qw + qz*qz) - 1.0;

    return R;
}

// Log map of a quaternion: q = [qw, qx, qy, qz]^T -> so(3)
// inline Eigen::Vector3d Log_quat(const Eigen::Vector4d& q, double epsilon = 1e-8) {
//     double qw = q(0);
//     Eigen::Vector3d qv = q.tail<3>();   // vector part [qx, qy, qz]
//     double s = qv.norm() + epsilon;

//     return 2.0 * qv * std::atan2(s, qw) / s;
// }

inline Eigen::Vector4d Log_quat(const Eigen::Matrix3d& R) {
    Eigen::Quaterniond q(R);
    return Eigen::Vector4d(q.w(), q.x(), q.y(), q.z());  // scalar first
}


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

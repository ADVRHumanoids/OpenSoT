#ifndef __OPENSOT_MANIFOLDS_H__
#define __OPENSOT_MANIFOLDS_H__

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <memory>
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

    virtual void plus(const Eigen::VectorXd& x0, const Eigen::VectorXd& dx0, Eigen::VectorXd& x1){
        assert(x0.size() == this->nq());
        assert(x1.size() == this->nq());
        assert(dx0.size() == this->nv());
    }

    virtual void minus(const Eigen::VectorXd& x0, const Eigen::VectorXd& x1, Eigen::VectorXd& dx){
        assert(x0.size() == this->nq());
        assert(x1.size() == this->nq());
        assert(dx.size() == this->nv());
    };

    Space getTangentSpace(){
        return Space(_nv,_nv);
    }

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

    void plus(const Eigen::VectorXd& x0, const Eigen::VectorXd& dx0, Eigen::VectorXd& x1)
    {
        x1 = x0 + dx0;
    }

    void minus(const Eigen::VectorXd& x0, const Eigen::VectorXd& x1, Eigen::VectorXd& dx)
    {
        dx = x0 - x1;
    }
};

class SE3Space: public Space
{
public:
    typedef std::shared_ptr<SE3Space> Ptr;

    SE3Space():
        Space(7, 6)
    {}

    void plus(const Eigen::VectorXd& x0, const Eigen::VectorXd& dx0, Eigen::VectorXd& x1)
    {
        x1 = SE3toXYZQUAT(XYZQUATtoSE3(x0) * Exp6(dx0));
    }


    void minus(const Eigen::VectorXd& x0, const Eigen::VectorXd& x1, Eigen::VectorXd& dx)
    {
        dx = Log6(XYZQUATtoSE3(x0).inverse() * XYZQUATtoSE3(x1));
    }
};

class CompositeSpace : public Space
{
private:
     std::vector<Space::Ptr> _spaces;

public:
     typedef std::shared_ptr<CompositeSpace> Ptr;

    // Constructor for manual spaces_list of Space
    CompositeSpace(const std::vector<Space::Ptr>& spaces_list):
         Space(0,0)
    {
        for(auto & state_space_representation : spaces_list)
        {
            _nq += state_space_representation->nq();
            _nv += state_space_representation->nv();
        }

        for(unsigned int i = 0; i < spaces_list.size(); ++i)
        {
            if(auto composite = std::dynamic_pointer_cast<CompositeSpace>(spaces_list[i]))
            {
                _spaces.insert(_spaces.end(), composite->getSpaces().begin(), composite->getSpaces().end());
            }
            else
            {
                _spaces.push_back(spaces_list[i]);
            }
        }

        _x.resize(_spaces.size());

        compute_spaces_and_indices();
    }

    const std::vector<Space::Ptr>& getSpaces()
    {
        return _spaces;
    }

    void plus(const Eigen::VectorXd& x0, const Eigen::VectorXd& dx0, Eigen::VectorXd& x1)
    {
        unsigned int i = 0;
        for(auto& space : _spaces)
        {
            unsigned int x0id = _map[space].first;
            unsigned int dx0id = _map[space].second;

            _x[i].resize(space->nq());
            _x[i].setZero();
            space->plus(x0.segment(x0id, space->nq()), dx0.segment(dx0id, space->nv()), _x[i]);
            x1.segment(x0id, space->nq()) = _x[i];

            i+=1;
        }
    }

    void minus(const Eigen::VectorXd& x0, const Eigen::VectorXd& x1, Eigen::VectorXd& dx)
    {
        unsigned int i = 0;
        for(auto& space : _spaces)
        {
            unsigned int x0id = _map[space].first;
            unsigned int x1id = _map[space].second;

            _x[i].resize(space->nv());
            _x[i].setZero();
            space->minus(x0.segment(x0id, space->nq()), x1.segment(x1id, space->nv()), _x[i]);
            dx.segment(x0id, space->nv()) = _x[i];

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

}

#endif
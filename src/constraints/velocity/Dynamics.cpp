#include <OpenSoT/constraints/velocity/Dynamics.h>
#include <yarp/math/Math.h>
#include <idynutils/cartesian_utils.h>

using namespace OpenSoT::constraints::velocity;
using namespace yarp::math;

#define YELLOW "\033[0;33m"
#define DEFAULT "\033[0m"

Dynamics::Dynamics(const yarp::sig::Vector &q, const yarp::sig::Vector &q_dot,
                   const yarp::sig::Vector &jointTorquesMax, iDynUtils &robot_model,
                   const double dT,
                   const double boundScaling):
    Constraint(q.size()),
    _jointTorquesMin(-1.0*jointTorquesMax),
    _jointTorquesMax(jointTorquesMax),
    _robot_model(robot_model),
    _dT(dT),
    _b(q.size()),
    _M(6+q.size(), 6+q.size()),
    _base_link("Waist"), //Here we have to use /Waist since /base_link is not recognized by iDynTree
    _Jc(0,0),
    _Fc(0),
    _tmp_wrench_in_sensor_frame(6,0.0),
    _tmp_wrench_in_base_link_frame(6, 0.0),
    _boundScaling(boundScaling)
{

    _Aineq.resize(_x_size, _x_size);
    _bLowerBound.resize(_x_size);
    _bUpperBound.resize(_x_size);

    /**
     * Since the constraint is specified in velocity we approximate the acceleration
     * to the first order, for these reason the jointTorquesMin/Max has to be multiplied by dT
    **/
    _jointTorquesMax = _dT*_jointTorquesMax;
    _jointTorquesMin = _dT*_jointTorquesMin;

    update(yarp::math::cat(q, q_dot));
}

yarp::sig::Vector Dynamics::Dynamics::getTorqueLimits()
{
    return _jointTorquesMax / _dT;
}

void Dynamics::Dynamics::setTorqueLimits(const yarp::sig::Vector torqueLimits)
{
    _jointTorquesMax = torqueLimits;
    _jointTorquesMin = -1.0*torqueLimits;

    /**
     * Since the constraint is specified in velocity we approximate the acceleration
     * to the first order, for these reason the jointTorquesMin/Max has to be multiplied by dT
    **/
    _jointTorquesMax = _dT*_jointTorquesMax;
    _jointTorquesMin = _dT*_jointTorquesMin;
}

void Dynamics::crawlLinks(const std::vector<std::string>& ft_links_list,
                const std::list<std::string>& contact_link_list, iDynUtils &robot,
                std::vector<std::string>& ft_in_contact_list)
{
    ft_in_contact_list.clear();
    std::vector<std::string> recursive_ft_links = ft_links_list;
    std::vector<std::string> ft_final_links;

    boost::shared_ptr<const urdf::ModelInterface> _urdf = robot.moveit_robot_model->getURDF();
    for(unsigned int ii = 0; ii < ft_links_list.size(); ++ii)
    {
        recursive_ft_links.clear();
        recursive_ft_links.push_back(ft_links_list[ii]);

        for(unsigned int i = 0; i < recursive_ft_links.size(); ++i)
        {
            boost::shared_ptr<const urdf::Link> l = _urdf->getLink(recursive_ft_links[i]);
            if(l->child_links.size() != 0)
            {
                for(unsigned int j = 0; j < l->child_links.size(); ++j)
                {
                    boost::shared_ptr<urdf::Link> l_child = l->child_links[j];
                    if(!(std::find(ft_links_list.begin(), ft_links_list.end(), l_child->name) != ft_links_list.end()))
                        recursive_ft_links.push_back(l_child->name);
                }
            }
            else
            {
                if(std::find(ft_links_list.begin(), ft_links_list.end(), recursive_ft_links[i]) != ft_links_list.end())
                    recursive_ft_links.push_back(l->getParent()->name);
                else
                    ft_final_links.push_back(recursive_ft_links[i]);
            }
        }

        bool a = false;
        for(unsigned int i  = 0; i < ft_final_links.size(); ++i)
        {

            if(std::find(contact_link_list.begin(), contact_link_list.end(), ft_final_links[i]) != contact_link_list.end())
            {
                ft_in_contact_list.push_back(ft_links_list[ii]);
                a = true;
            }
            if(a)
                break;
        }

        ft_final_links.clear();
    }



}

void Dynamics::updateActualWrench()
{
    std::vector<std::string> ft_in_contact;
    crawlLinks(_robot_model.getForceTorqueFrameNames(),
               _robot_model.getLinksInContact(),
               _robot_model,
               ft_in_contact);


    _Fc.clear();
    _Jc.resize(0,0);
    for(unsigned int i = 0; i < ft_in_contact.size(); ++i)
    {
        /**
         * For each ft in contact we first take the measured wrench and we
         * transform the wrench from the sensor frame to the base_link frame.
         */
        moveit::core::LinkModel* ft_link =
                _robot_model.moveit_robot_model->getLinkModel(ft_in_contact[i]);

        unsigned int ft_index = _robot_model.iDyn3_model.getFTSensorIndex(ft_link->getParentJointModel()->getName());
        _robot_model.iDyn3_model.getSensorMeasurement(ft_index, _tmp_wrench_in_sensor_frame);

        if(_tmp_wrench_in_sensor_frame.size() > 0)
        {
            KDL::Wrench wrench_in_sensor_frame_KDL;
            cartesian_utils::fromYarpVectortoKDLWrench(_tmp_wrench_in_sensor_frame, wrench_in_sensor_frame_KDL);

            // Wrench has to be transformed from ft_frame to base_link **/
            yarp::sig::Matrix ft_frame_in_base_link;
            ft_frame_in_base_link = _robot_model.iDyn3_model.getPosition(
                    _robot_model.iDyn3_model.getLinkIndex(_base_link),
                    _robot_model.iDyn3_model.getLinkIndex(ft_in_contact[i]));

            KDL::Frame ft_frame_in_base_link_KDL;
            cartesian_utils::fromYARPMatrixtoKDLFrame(ft_frame_in_base_link, ft_frame_in_base_link_KDL);

            KDL::Wrench wrench_in_base_link = ft_frame_in_base_link_KDL.M * wrench_in_sensor_frame_KDL;
            cartesian_utils::fromKDLWrenchtoYarpVector(wrench_in_base_link, _tmp_wrench_in_base_link_frame);

            /**
              * Here we concatenate the wrenches
             **/
            if(_Fc.size() == 0){
                _Fc.resize(_tmp_wrench_in_base_link_frame.size(), 0.0);
                _Fc = _tmp_wrench_in_base_link_frame;}
            else
                _Fc = yarp::math::cat(_Fc, _tmp_wrench_in_base_link_frame);

            /**
              * Then we need to compute the Jacobian of the contact: from base_link to
              * ft_frame.
             **/
            unsigned int base_link_index = _robot_model.iDyn3_model.getLinkIndex(_base_link);
            unsigned int ft_link_index = _robot_model.iDyn3_model.getLinkIndex(ft_in_contact[i]);
            yarp::sig::Matrix A(0,0);
            _robot_model.iDyn3_model.getRelativeJacobian(ft_link_index, base_link_index, A, true);
            yarp::sig::Matrix base_R_world = _robot_model.iDyn3_model.getPosition(base_link_index).submatrix(0,2,0,2).transposed();
            yarp::sig::Matrix Adj(6,6); Adj = Adj.eye();
            Adj.setSubmatrix(base_R_world, 0,0);
            Adj.setSubmatrix(base_R_world, 3,3);
            A = Adj*A;

            /**
              * Here we concatenate the Jacobians of the contacts
             **/
            if(_Jc.rows() == 0){
                _Jc.resize(A.rows(), A.cols()); _Jc = A;}
            else
                _Jc = yarp::math::pile(_Jc, A);
        }
        else
            std::cout<<YELLOW<<"WARNING: can not read wrench @ "<<ft_in_contact[i]<<" sensor, skip."<<DEFAULT<<std::endl;
    }
}

void Dynamics::update(const yarp::sig::Vector &x)
{
    assert(x.size() == 2*_x_size || x.size() == _x_size);

    /**
     * Here I suppose that q and dq have been set in the iDynTree Model,
     * this basically save time since I do not have to compute anything more than ID:
     *
     *      ID(q, q_dot, zero) = C(q,q_dot)q_dot + g(q) = -b1
    **/
    _b = _robot_model.iDyn3_model.getTorques();

    /**
     * Since the constraint is specified in velocity we approximate the acceleration
     * to the first order, for these reason b1 has to be multiplied by dT
    **/
    _b = -1.0*_dT*_b;

    /**
     * Here we need the Mass matrix to compute the term:
     *
     *      Mq_dot = b2
     *
     * and summed to the previous one
    **/
    _M.resize(6+_x_size, 6+_x_size);
    _robot_model.iDyn3_model.getFloatingBaseMassMatrix(_M);
    _M = _M.removeCols(0,6); _M = _M.removeRows(0,6);
    if(x.size() == 2*_x_size)
        _b = _b + _M*x.subVector(_x_size, (2*_x_size)-1);
    else
        _b = _b + _M*_robot_model.iDyn3_model.getDAng();

    /**
      * Here we compute the torque due to contacts (if present):
      *
      *     -dT*Jc'Fc = b3
     **/
    updateActualWrench();
    if(_Fc.size() > 0)
        _b = _b - _dT*_Jc.transposed()*_Fc;


    /**
     * The final constraint is given by:
    **/
    _bLowerBound = _boundScaling*(_jointTorquesMin + _b);
    _bUpperBound = _boundScaling*(_jointTorquesMax + _b);
    _Aineq = (1.0/_dT)*_M;
}

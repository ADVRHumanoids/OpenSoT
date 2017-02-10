#ifndef __INTERFACES_YPOSTURAL_H__
#define __INTERFACES_YPOSTURAL_H__

#include <OpenSoT/tasks/velocity/Postural.h>
#include <boost/smart_ptr/scoped_ptr.hpp>
#include <OpenSoT/interfaces/yarp/yarp_msgs/yarp_position_joint_msg.h>
#include <boost/thread/mutex.hpp>
#include <Eigen/Dense>
#include <idynutils/idynutils.h>


namespace OpenSoT {
    using namespace tasks::velocity;
    namespace interfaces {
        namespace yarp {
            namespace tasks{

class RPCCallBackPostural : public ::yarp::os::PortReader
{
public:
    enum output_type{
        SUCCEED,
        ERROR_NEGATIVE_W_GAIN,
        ERROR_WRONG_VECTOR_SIZE,
        ERROR_NEGATIVE_LAMBDA_GAIN,
        ERROR_LAMBA_GAIN_MORE_THAN_1
    };

    RPCCallBackPostural(Postural::Ptr task, iDynUtils &idynutils):
        _W(task->getWeight()),
        _lambda(task->getLambda()),
        _help_string("help"),
        _W_string("W"),
        _lambda_string("lambda"),
        _set_string("set "),
        _get_string("get "),
        _actual_position("actual_position"),
        _actual_posture("actual_posture"),
        _task(task),
        _idynutils(idynutils)
    {

    }

    virtual bool read(::yarp::os::ConnectionReader& connection)
    {
        _in.clear();
        _out.clear();

        bool ok = _in.read(connection);

        if (!ok)
            return false;

        prepareInputAndOutput();
        ::yarp::os::ConnectionWriter *returnToSender = connection.getWriter();
        if(returnToSender != NULL)
            _out.write(*returnToSender);

        return true;
    }

private:
    ::yarp::os::Bottle _in;
    ::yarp::os::Bottle _out;
    Eigen::MatrixXd _W;
    double _lambda;
    boost::mutex _mtx;

    std::string _help_string;
    std::string _W_string;
    std::string _lambda_string;
    std::string _set_string;
    std::string _get_string;
    std::string _actual_position;
    std::string _actual_posture;

    Postural::Ptr _task;
    iDynUtils& _idynutils;

    void prepareInputAndOutput()
    {
        std::string command = _in.get(0).asString();
        if(command == _help_string)
            help();
        else if(command == (_set_string + _W_string))
            setW();
        else if(command == (_set_string + _lambda_string))
            setLambda();
        else if(command == (_get_string + _W_string))
            getW();
        else if(command == (_get_string + _lambda_string))
            getLambda();
        else if(command == (_get_string + _actual_position))
            getActualPositions();
        else if(command == (_get_string + _actual_posture))
            getActualPosture();
        else
            std::cout<<"Unknown command! Run help instead!"<<std::endl;
    }

    bool setW()
    {
        boost::unique_lock<boost::mutex>lck(_mtx);

        ::yarp::sig::Vector v;

        for(unsigned int i = 1; i < _in.size(); ++i)
        {
            double w_ii = _in.get(i).asDouble();
            if(w_ii >= 0.0)
                v.push_back(w_ii);
            else
            {
                _out.addInt(ERROR_NEGATIVE_W_GAIN);
                return false;
            }
        }

        if(v.size() == _W.rows())
        {
            for(unsigned int i = 0; i < v.size(); ++i)
                _W(i,i) = v(i);

            _task->setWeight(_W);
            _out.addInt(SUCCEED);
        }
        else
        {
            _out.addInt(ERROR_WRONG_VECTOR_SIZE);
            return false;
        }

        return true;
    }

    bool setLambda()
    {
        boost::unique_lock<boost::mutex>lck(_mtx);

        double lambda = _in.get(1).asDouble();

        if(lambda <= 0.0)
        {
            _out.addInt(ERROR_NEGATIVE_LAMBDA_GAIN);
            return false;
        }

        if (lambda > 1.0)
        {
            _out.addInt(ERROR_LAMBA_GAIN_MORE_THAN_1);
            return false;
        }

        _lambda = lambda;
        _task->setLambda(_lambda);
        _out.addInt(SUCCEED);
        return true;
    }

    void getW()
    {
        _W = _task->getWeight();
        for(unsigned int i = 0; i < _W.rows(); ++i)
            _out.addDouble(_W(i,i));
    }

    void getLambda()
    {
        _lambda = _task->getLambda();
        _out.addDouble(_lambda);
    }

    void getActualPositions()
    {
        Eigen::VectorXd q = _task->getActualPositions();
        for(unsigned int i = 0; i < q.rows(); ++i)
            _out.addDouble(q[i]);
    }

    void getActualPosture()
    {
        Eigen::VectorXd q = _task->getActualPositions();
        std::map<std::string, double> joint_map;
        std::vector<std::string> joint_names = _idynutils.getJointNames();
        for(unsigned int i = 0; i < q.rows(); ++i)
            joint_map[joint_names[i]] = q[i];
        msgs::yarp_position_joint_msg joint_msg(joint_map);
        joint_msg.serializeMsg(_out);
    }

    void help()
    {
        std::cout<<"help: "<<std::endl;
        std::cout<<std::endl;
        std::cout<<"    set W:"<<std::endl;
        std::cout<<"        in: w00 w11 ... wnn"<<std::endl;
        std::cout<<"        out: 0 if succeed, ERROR # otherwise"<<std::endl;
        std::cout<<"    set lambda:"<<std::endl;
        std::cout<<"        in: lambda"<<std::endl;
        std::cout<<"        out: 0 if succeed, ERROR # otherwise"<<std::endl;
        std::cout<<std::endl;
        std::cout<<"    get W:"<<std::endl;
        std::cout<<"        in: "<<std::endl;
        std::cout<<"        out: diag(W)"<<std::endl;
        std::cout<<"    get lambda:"<<std::endl;
        std::cout<<"        in: "<<std::endl;
        std::cout<<"        out: lambda as double"<<std::endl;
        std::cout<<"    get actual_position:"<<std::endl;
        std::cout<<"        in: "<<std::endl;
        std::cout<<"        out: vector of actual joint positions"<<std::endl;
        std::cout<<std::endl;
    }
};

class YPostural : public ::yarp::os::BufferedPort<msgs::yarp_position_joint_msg_portable>
{
public:
    typedef boost::shared_ptr<YPostural> Ptr;

    YPostural(const std::string& robot_name,
               const std::string& module_prefix,
               iDynUtils& idynutils,
               Postural::Ptr postural_task);

    YPostural(const std::string& robot_name,
               const std::string& module_prefix,
              iDynUtils& idynutils,
               const Eigen::VectorXd& x);

    void cleanPorts();

    Postural::Ptr taskPostural;

    std::string getTaskID(){return taskPostural->getTaskID();}
    std::string getPortPrefix(){return _port_prefix;}

private:
    std::string _port_prefix;
    RPCCallBackPostural _rpc_cb;
    ::yarp::os::RpcServer _rpc;
    iDynUtils& _idynutils;

    bool computePortPrefix(const std::string& robot_name,
                           const std::string& module_prefix,
                           const std::string& task_id);

    void onRead(msgs::yarp_position_joint_msg_portable& ref_position_joint_msg);
    void printInitialError();
};

            }
        }
    }
}

#endif

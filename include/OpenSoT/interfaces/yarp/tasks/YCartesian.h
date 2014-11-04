#ifndef __INTERFACES_YCARTESIAN_H__
#define __INTERFACES_YCARTESIAN_H__

#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <boost/smart_ptr/scoped_ptr.hpp>
#include <OpenSoT/interfaces/yarp/yarp_msgs/yarp_trj_msg.h>
#include <mutex>


namespace OpenSoT {
    using namespace tasks::velocity;
    namespace interfaces {
        namespace yarp {
            namespace tasks{

class RPCCallBack : public ::yarp::os::PortReader
{
public:
    enum output_type{
        SUCCEED,
        ERROR_NEGATIVE_W_GAIN,
        ERROR_WRONG_VECTOR_SIZE,
        ERROR_NEGATIVE_LAMBDA_GAIN,
        ERROR_LAMBA_GAIN_MORE_THAN_1,
        ERROR_NEGATIVE_ORIENTATION_GAIN
    };

    RPCCallBack(Cartesian::Ptr task):
        _W(task->getWeight()),
        _lambda(task->getLambda()),
        _orientation_gain(task->getOrientationErrorGain()),
        _help_string("help"),
        _W_string("W"),
        _lambda_string("lambda"),
        _orientation_gain_string("orientation_gain"),
        _set_string("set "),
        _get_string("get "),
        _task(task)
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
    ::yarp::sig::Matrix _W;
    double _lambda;
    double _orientation_gain;
    std::mutex _mtx;

    std::string _help_string;
    std::string _W_string;
    std::string _lambda_string;
    std::string _orientation_gain_string;
    std::string _set_string;
    std::string _get_string;

    Cartesian::Ptr _task;

    void prepareInputAndOutput()
    {
        std::string command = _in.get(0).asString();
        if(command == _help_string)
            help();
        else if(command == (_set_string + _W_string))
            setW();
        else if(command == (_set_string + _lambda_string))
            setLambda();
        else if(command == (_set_string + _orientation_gain_string))
            setOrientationGain();
        else if(command == (_get_string + _W_string))
            getW();
        else if(command == (_get_string + _lambda_string))
            getLambda();
        else if(command == (_get_string + _orientation_gain_string))
            getOrientationGain();
        else
            std::cout<<"Unknown command! Run help instead!"<<std::endl;
    }

    bool setW()
    {
        std::unique_lock<std::mutex>lck(_mtx);

        ::yarp::sig::Vector v;

        for(unsigned int i = 1; i < _in.size(); ++i)
        {
            double w_ii = _in.get(i).asDouble();
            if(w_ii >= 0.0)
                v.push_back(w_ii);
            else
            {
                _out.addInt(output_type::ERROR_NEGATIVE_W_GAIN);
                return false;
            }
        }

        if(v.size() == _W.rows())
        {
            for(unsigned int i = 0; i < v.size(); ++i)
                _W(i,i) = v(i);

            _task->setWeight(_W);
            _out.addInt(output_type::SUCCEED);
        }
        else
        {
            _out.addInt(output_type::ERROR_WRONG_VECTOR_SIZE);
            return false;
        }

    }

    bool setOrientationGain()
    {
        std::unique_lock<std::mutex>lck(_mtx);

        double orientation_gain = _in.get(1).asDouble();

        if(orientation_gain < 0.0)
        {
            _out.addInt(output_type::ERROR_NEGATIVE_ORIENTATION_GAIN);
            return false;
        }

        _orientation_gain = orientation_gain;
        _task->setOrientationErrorGain(_orientation_gain);
        _out.addInt(output_type::SUCCEED);
        return true;
    }

    bool setLambda()
    {
        std::unique_lock<std::mutex>lck(_mtx);

        double lambda = _in.get(1).asDouble();

        if(lambda <= 0.0)
        {
            _out.addInt(output_type::ERROR_NEGATIVE_LAMBDA_GAIN);
            return false;
        }

        if (lambda > 1.0)
        {
            _out.addInt(output_type::ERROR_LAMBA_GAIN_MORE_THAN_1);
            return false;
        }

        _lambda = lambda;
        _task->setLambda(_lambda);
        _out.addInt(output_type::SUCCEED);
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

    void getOrientationGain()
    {
        _orientation_gain = _task->getOrientationErrorGain();
        _out.addDouble(_orientation_gain);
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
        std::cout<<"    set orientation_gain:"<<std::endl;
        std::cout<<"        in: orientation_gain"<<std::endl;
        std::cout<<"        out: 0 if succeed, ERROR # otherwise"<<std::endl;
        std::cout<<std::endl;
        std::cout<<"    get W:"<<std::endl;
        std::cout<<"        in: "<<std::endl;
        std::cout<<"        out: diag(W)"<<std::endl;
        std::cout<<"    get lambda:"<<std::endl;
        std::cout<<"        in: "<<std::endl;
        std::cout<<"        out: lambda as double"<<std::endl;
        std::cout<<"    get orientation_gain:"<<std::endl;
        std::cout<<"        in: "<<std::endl;
        std::cout<<"        out: orientation_gain as double"<<std::endl;
        std::cout<<std::endl;
    }
};

class YCartesian : public ::yarp::os::BufferedPort<msgs::yarp_trj_msg_portable>
{
public:
    YCartesian(const std::string& robot_name,
               const std::string& module_prefix,
               Cartesian::Ptr cartesian_task);

    YCartesian(const std::string& robot_name,
               const std::string& module_prefix,
               std::string task_id,
               const ::yarp::sig::Vector& x,
               iDynUtils &robot,
               std::string distal_link,
               std::string base_link);

    void cleanPorts();

    Cartesian::Ptr taskCartesian;

    std::string getDistalLink(){return taskCartesian->getDistalLink();}
    std::string getBaseLink(){return taskCartesian->getBaseLink();}
    std::string getTaskID(){return taskCartesian->getTaskID();}
    std::string getPortPrefix(){return _port_prefix;}

private:
    std::string _port_prefix;
    RPCCallBack _rpc_cb;
    ::yarp::os::RpcServer _rpc;

    bool computePortPrefix(const std::string& robot_name,
                           const std::string& module_prefix,
                           const std::string& task_id);

    void onRead(msgs::yarp_trj_msg_portable& ref_trj_msg);
    void printInitialError();
};

            }
        }
    }
}

#endif

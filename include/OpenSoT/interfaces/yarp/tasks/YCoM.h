#ifndef __INTERFACES_YCOM_H__
#define __INTERFACES_YCOM_H__

#include <OpenSoT/tasks/velocity/CoM.h>
#include <boost/smart_ptr/scoped_ptr.hpp>
#include <OpenSoT/interfaces/yarp/yarp_msgs/yarp_trj_msg.h>
#include <boost/thread/mutex.hpp>


namespace OpenSoT {
    using namespace tasks::velocity;
    namespace interfaces {
        namespace yarp {
            namespace tasks{

class RPCCallBackCoM : public ::yarp::os::PortReader
{
public:
    enum output_type{
        SUCCEED,
        ERROR_NEGATIVE_W_GAIN,
        ERROR_WRONG_VECTOR_SIZE,
        ERROR_NEGATIVE_LAMBDA_GAIN,
        ERROR_LAMBA_GAIN_MORE_THAN_1
    };

    RPCCallBackCoM(CoM::Ptr task):
        _W(task->getWeight()),
        _lambda(task->getLambda()),
        _help_string("help"),
        _W_string("W"),
        _lambda_string("lambda"),
        _actual_position_string("actual_position"),
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
    ::yarp::sig::Vector _task_position;
    boost::mutex _mtx;

    std::string _help_string;
    std::string _W_string;
    std::string _lambda_string;
    std::string _actual_position_string;
    std::string _set_string;
    std::string _get_string;

    CoM::Ptr _task;

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
        else if(command == (_get_string + _actual_position_string))
            getActualPosition();
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

    void getActualPosition()
    {
        _task_position = _task->getActualPosition();
        for(unsigned int i = 0; i < 3; ++i){
            _out.addDouble(_task_position(i));
        }
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
        std::cout<<"        out: position as double"<<std::endl;
        std::cout<<std::endl;
    }
};

class YCoM : public ::yarp::os::BufferedPort<msgs::yarp_trj_msg_portable>
{
public:
    typedef boost::shared_ptr<YCoM> Ptr;

    YCoM(const std::string& robot_name,
               const std::string& module_prefix,
               CoM::Ptr cartesian_task);

    YCoM(const std::string& robot_name,
               const std::string& module_prefix,
               const ::yarp::sig::Vector& x,
               iDynUtils &robot);

    void cleanPorts();

    CoM::Ptr taskCoM;

    std::string getDistalLink(){return taskCoM->getDistalLink();}
    std::string getBaseLink(){return taskCoM->getBaseLink();}
    std::string getTaskID(){return taskCoM->getTaskID();}
    std::string getPortPrefix(){return _port_prefix;}

private:
    std::string _port_prefix;
    RPCCallBackCoM _rpc_cb;
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

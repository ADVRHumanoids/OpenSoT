#include <OpenSoT/interfaces/yarp/tasks/YPostural.h>

using namespace OpenSoT::interfaces::yarp::tasks;

YPostural::YPostural(const std::string &robot_name, const std::string &module_prefix, iDynUtils &idynutils,
                       const ::yarp::sig::Vector &x):
    ::yarp::os::BufferedPort<msgs::yarp_position_joint_msg_portable>(),
    taskPostural(new Postural(x)),
    _port_prefix(),
    _rpc_cb(taskPostural, idynutils),
    _rpc(),
    _idynutils(idynutils)
{
    bool res = computePortPrefix(robot_name, module_prefix, taskPostural->getTaskID());
    assert(res);
    open(_port_prefix+"set_ref:i");

    useCallback();
    _rpc.setReader(_rpc_cb);
    _rpc.open(_port_prefix+"rpc");

    printInitialError();
}

YPostural::YPostural(const std::string& robot_name,
                     const std::string& module_prefix, iDynUtils &idynutils,
                     Postural::Ptr postural_task):
    ::yarp::os::BufferedPort<msgs::yarp_position_joint_msg_portable>(),
    taskPostural(postural_task),
    _port_prefix(),
    _rpc_cb(taskPostural, idynutils),
    _rpc(),
    _idynutils(idynutils)
{
    bool res = computePortPrefix(robot_name, module_prefix, postural_task->getTaskID());
    assert(res);
    open(_port_prefix+"set_ref:i");
    useCallback();

    _rpc.setReader(_rpc_cb);
    _rpc.open(_port_prefix+"rpc");

    printInitialError();
}

void YPostural::cleanPorts()
{
    std::cout<<"Cleaning Ports for "<<taskPostural->getTaskID()<<"...";

    ::yarp::os::Bottle* foo;

    int pendings = getPendingReads();
    for(unsigned int i = 0; i < pendings; ++i)
        read(foo);

    std::cout<<"clean!"<<std::endl;
}

bool YPostural::computePortPrefix(const std::string& robot_name, const std::string& module_prefix, const std::string &task_id)
{
    _port_prefix = "/";
    if(robot_name != "")
        _port_prefix += robot_name+"/";
    if(module_prefix != "")
        _port_prefix += module_prefix+"/";
    if(task_id == "")
    {
        std::cout<<"ERROR: task_id can not be an empty string!"<<std::endl;
        return false;
    }
    _port_prefix += task_id+"/";
    return true;
}

void YPostural::onRead(msgs::yarp_position_joint_msg_portable& ref_position_joint_msg)
{
    ::yarp::sig::Vector q_ref = taskPostural->getReference();

    for(std::map<std::string, double>::iterator i = ref_position_joint_msg.joints.begin(); i != ref_position_joint_msg.joints.end(); ++i)
       q_ref[_idynutils.iDyn3_model.getDOFIndex(i->first)] = i->second;

    taskPostural->setReference(q_ref);
}

void YPostural::printInitialError()
{
    std::cout<<"Initial Reference for "<<taskPostural->getTaskID()<<":"<<std::endl;
    ::yarp::sig::Vector tmp = taskPostural->getReference();
    std::cout<<"[ "<<tmp.toString()<<" ]"<<std::endl;
    std::cout<<std::endl;
}

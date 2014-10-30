#include <OpenSoT/interfaces/yarp/tasks/YCartesian.h>

using namespace OpenSoT::interfaces::yarp::tasks;

YCartesian::YCartesian(const std::string &robot_name, const std::string &module_prefix, std::string task_id,
                       const ::yarp::sig::Vector &x, iDynUtils &robot, std::string distal_link, std::string base_link):
    taskCartesian(new Cartesian(task_id,x,robot, distal_link, base_link)),
    _port_prefix(),
    _bot(new ::yarp::os::Bottle())
{
    assert(computePortPrefix(robot_name, module_prefix, task_id));
    _cart_ref_port.open(_port_prefix+"set_ref:i");
}

YCartesian::YCartesian(const std::string& robot_name,
                       const std::string& module_prefix,
                       Cartesian::Ptr cartesian_task):
    taskCartesian(cartesian_task),
    _bot(new ::yarp::os::Bottle())
{
    assert(computePortPrefix(robot_name, module_prefix, cartesian_task->getTaskID()));
    _cart_ref_port.open(_port_prefix+"set_ref:i");
}

YCartesian::~YCartesian()
{
    std::cout<<"Cleaning Ports for "<<taskCartesian->getTaskID()<<"...";

    ::yarp::os::Bottle* foo;

    int pendings = _cart_ref_port.getPendingReads();
    for(unsigned int i = 0; i < pendings; ++i)
        _cart_ref_port.read(foo);

    std::cout<<"clean!"<<std::endl;
}

bool YCartesian::computePortPrefix(const std::string& robot_name, const std::string& module_prefix, const std::string &task_id)
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

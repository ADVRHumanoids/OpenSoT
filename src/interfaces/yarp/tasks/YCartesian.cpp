#include <OpenSoT/interfaces/yarp/tasks/YCartesian.h>

YCartesian::YCartesian(const std::string &robot_name, const std::string &module_prefix, std::string task_id,
                       const yarp::sig::Vector &x, iDynUtils &robot, std::string distal_link, std::string base_link):
    taskCartesian(new Cartesian(task_id,x,robot, distal_link, base_link)),
    _port_prefix("/"+robot_name+"/"+module_prefix+"/"+task_id+"/"),
    _distal_link(distal_link), _base_link(base_link),
    _bot(new yarp::os::Bottle())
{
    _cart_ref_port.open(_port_prefix+"set_ref:i");
}

YCartesian::~YCartesian()
{
    yarp::os::Bottle* foo;

    int pendings = _cart_ref_port.getPendingReads();
    for(unsigned int i = 0; i < pendings; ++i)
        _cart_ref_port.read(foo);
}

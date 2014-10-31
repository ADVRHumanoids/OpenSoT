#ifndef __INTERFACES_YCARTESIAN_H__
#define __INTERFACES_YCARTESIAN_H__

#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <boost/smart_ptr/scoped_ptr.hpp>
#include <OpenSoT/interfaces/yarp/yarp_msgs/yarp_trj_msg.h>


namespace OpenSoT {
    using namespace tasks::velocity;
    namespace interfaces {
        namespace yarp {
            namespace tasks{

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

    bool computePortPrefix(const std::string& robot_name,
                           const std::string& module_prefix,
                           const std::string& task_id);

    void onRead(msgs::yarp_trj_msg_portable& ref_trj_msg);
};

            }
        }
    }
}

#endif

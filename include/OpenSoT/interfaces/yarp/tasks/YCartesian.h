#ifndef __INTERFACES_YCARTESIAN_H__
#define __INTERFACES_YCARTESIAN_H__

#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <boost/smart_ptr/scoped_ptr.hpp>

namespace OpenSoT {
    using namespace tasks::velocity;
    namespace interfaces {
        namespace yarp {
            namespace tasks{

class YCartesian
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
    ~YCartesian();

    Cartesian::Ptr taskCartesian;

    std::string getDistalLink(){return taskCartesian->getDistalLink();}
    std::string getBaseLink(){return taskCartesian->getBaseLink();}
    std::string getTaskID(){return taskCartesian->getTaskID();}
    std::string getPortPrefix(){return _port_prefix;}

private:
    std::string _port_prefix;

    ::yarp::os::BufferedPort<::yarp::os::Bottle> _cart_ref_port;
    boost::scoped_ptr<::yarp::os::Bottle> _bot;

    bool computePortPrefix(const std::string& robot_name,
                           const std::string& module_prefix,
                           const std::string& task_id);
};

            }
        }
    }
}

#endif

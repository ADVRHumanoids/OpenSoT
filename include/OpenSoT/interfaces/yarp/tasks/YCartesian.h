#ifndef __INTERFACES_YCARTESIAN_H__
#define __INTERFACES_YCARTESIAN_H__

#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <boost/smart_ptr/scoped_ptr.hpp>

using namespace OpenSoT::tasks::velocity;

class YCartesian
{
public:
    YCartesian(const std::string& robot_name,
               const std::string& module_prefix,
               std::string task_id,
               const yarp::sig::Vector& x,
               iDynUtils &robot,
               std::string distal_link,
               std::string base_link);
    ~YCartesian();

    Cartesian::Ptr taskCartesian;

    std::string getDistalLink(){return _distal_link;}
    std::string getBaseLink(){return _base_link;}

private:
    std::string _port_prefix;
    std::string _distal_link;
    std::string _base_link;


    yarp::os::BufferedPort<yarp::os::Bottle> _cart_ref_port;
    boost::scoped_ptr<yarp::os::Bottle> _bot;
};

#endif

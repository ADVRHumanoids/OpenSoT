#include <OpenSoT/interfaces/yarp/tasks/YCoM.h>

using namespace OpenSoT::interfaces::yarp::tasks;

YCoM::YCoM(const std::string &robot_name, const std::string &module_prefix,
                       const ::yarp::sig::Vector &x, iDynUtils &robot):
    ::yarp::os::BufferedPort<msgs::yarp_trj_msg_portable>(),
    taskCoM(new CoM(x,robot)),
    _port_prefix(),
    _rpc_cb(taskCoM),
    _rpc()
{
    bool res = computePortPrefix(robot_name, module_prefix, taskCoM->getTaskID());
    assert(res);
    open(_port_prefix+"set_ref:i");

    useCallback();
    _rpc.setReader(_rpc_cb);
    _rpc.open(_port_prefix+"rpc");

    printInitialError();
}

YCoM::YCoM(const std::string& robot_name,
                       const std::string& module_prefix,
                       CoM::Ptr com_task):
    ::yarp::os::BufferedPort<msgs::yarp_trj_msg_portable>(),
    taskCoM(com_task),
    _port_prefix(),
    _rpc_cb(taskCoM),
    _rpc()
{
    bool res = computePortPrefix(robot_name, module_prefix, com_task->getTaskID());
    assert(res);
    open(_port_prefix+"set_ref:i");
    useCallback();

    _rpc.setReader(_rpc_cb);
    _rpc.open(_port_prefix+"rpc");

    printInitialError();
}

void YCoM::cleanPorts()
{
    std::cout<<"Cleaning Ports for "<<taskCoM->getTaskID()<<"...";

    ::yarp::os::Bottle* foo;

    int pendings = getPendingReads();
    for(unsigned int i = 0; i < pendings; ++i)
        read(foo);

    std::cout<<"clean!"<<std::endl;
}

bool YCoM::computePortPrefix(const std::string& robot_name, const std::string& module_prefix, const std::string &task_id)
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

void YCoM::onRead(msgs::yarp_trj_msg_portable& ref_trj_msg)
{
    if(!(ref_trj_msg.base_frame == taskCoM->getBaseLink()))
        std::cout<<"WARNING: Reference Trajectory has "<<ref_trj_msg.base_frame<<
                   " instead of "<<taskCoM->getBaseLink()<<" as base_frame"<<std::endl;
    else
    {
        if(!(ref_trj_msg.distal_frame == taskCoM->getDistalLink()))
            std::cout<<"WARNING: Reference Trajectory has "<<ref_trj_msg.distal_frame<<
                       " instead of "<<taskCoM->getDistalLink()<<" as distal_frame"<<std::endl;
        else
        {
            ::yarp::sig::Vector tmp(3, 0.0);
            tmp[0] = ref_trj_msg.pose.p.x();
            tmp[1] = ref_trj_msg.pose.p.y();
            tmp[2] = ref_trj_msg.pose.p.z();

//            ::yarp::sig::Vector tmp2(6, 0.0);
//            tmp2[0] = ref_trj_msg.twist.vel.x();
//            tmp2[1] = ref_trj_msg.twist.vel.y();
//            tmp2[2] = ref_trj_msg.twist.vel.z();

            taskCoM->setReference(tmp);
        }
    }
}

void YCoM::printInitialError()
{
    std::cout<<"Initial Reference for "<<taskCoM->getTaskID()<<":"<<std::endl;
    ::yarp::sig::Vector tmp = taskCoM->getReference();
    KDL::Frame com_pose;
    com_pose.Identity();
    com_pose.p[0] = tmp [0];
    com_pose.p[1] = tmp [1];
    com_pose.p[2] = tmp [2];
    cartesian_utils::printKDLFrame(com_pose);
    std::cout<<std::endl;
}

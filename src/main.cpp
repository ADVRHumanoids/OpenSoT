#include <yarp/os/all.h>
#include "sot_VelKinCon_ctrl.h"

#define dT 0.001 //[s]

class sot_VelKinCon_module: public yarp::os::RFModule
{
protected:
    sot_VelKinCon_ctrl *thr;
    bool ctrl_started;
    yarp::os::BufferedPort<yarp::os::Bottle> idle_port;
public:
    bool configure(int argc, char* argv[])
    {
        thr = new sot_VelKinCon_ctrl(dT, argc, argv);
        if(!thr->start())
        {
            delete thr;
            return false;
        }
        std::cout<<"Starting Module"<<std::endl;
        ctrl_started = true;

        idle_port.open("/sot_VelKinCon/idle:i");
        return true;
    }

    virtual bool close()
    {
        thr->stop();
        delete thr;
        idle_port.close();
        return true;
    }

    virtual double getPeriod(){return 1.0;}
    virtual bool updateModule()
    {
        std::string start = "start";
        std::string stop = "stop";

        yarp::os::Bottle* bot = idle_port.read();

        if(!bot==NULL)
        {
            if((start.compare(bot->get(0).asString()) == 0) && !ctrl_started)
            {
                ctrl_started = true;
                thr->start();
                std::cout<<"Starting Module"<<std::endl;
            }
            if((stop.compare(bot->get(0).asString()) == 0) && ctrl_started)
            {
                ctrl_started = false;
                thr->stop();
                std::cout<<"Stopping Module"<<std::endl;
            }
        }

        return true;
    }
};

int main(int argc, char* argv[])
{
    yarp::os::Network yarp;
    if(!yarp.checkNetwork()){
        std::cout<<"yarpserver not running, pls run yarpserver"<<std::endl;
        return 0;}

    sot_VelKinCon_module mod;
    mod.configure(argc, argv);

    return mod.runModule();
}

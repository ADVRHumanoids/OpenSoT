/*
 * Copyright: (C) 2014 Walkman Consortium
 * Authors: Enrico Mingo, Alessio Rocchi
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/


#include <yarp/os/all.h>
#include <paramHelp/paramHelperServer.h>
#include "sot_VelKinCon_ctrl.h"

// module name, time constant, DEBUG mode are defined here
#include "sot_VelKinCon_constants.h"

class sot_VelKinCon_module: public yarp::os::RFModule
{
    paramHelp::ParamHelperServer *paramHelper;
    yarp::os::Port                rpcPort;		// a port to handle rpc messages
    yarp::os::BufferedPort<yarp::os::Bottle> switch_port;

    bool ctrl_started;
    wb_sot::sot_VelKinCon_ctrl *thr;

    int     period;
    double  dT;
    bool    left_arm_impedance_control, right_arm_impedance_control, torso_impedance_control;
    double  avgTime, stdDev, avgTimeUsed, stdDevUsed;
public:
    bool configure(yarp::os::ResourceFinder &rf)
    {
        //--------------------------PARAMETER HELPER--------------------------
        paramHelper = new ParamHelperServer(wb_sot::sot_VelKinCon_ParamDescr, wb_sot::PARAM_ID_SIZE,
                                            wb_sot::sot_VelKinCon_CommandDescr, wb_sot::COMMAND_ID_SIZE);
        paramHelper->linkParam(wb_sot::PARAM_ID_DT, &dT);
        paramHelper->linkParam(wb_sot::PARAM_ID_LEFT_ARM_IMPEDANCE_CONTROL, &left_arm_impedance_control);
        paramHelper->linkParam(wb_sot::PARAM_ID_RIGHT_ARM_IMPEDANCE_CONTROL, &right_arm_impedance_control);
        paramHelper->linkParam(wb_sot::PARAM_ID_TORSO_IMPEDANCE_CONTROL, &torso_impedance_control);

        /*
        if(rf.check("dT")) period = rf.find("dT").asDouble()  * 1000;
        else { std::cerr << "Could not load parameter dT, exiting" << std::endl; return false; }
        */

        // Read parameters from configuration file (or command line)
        yarp::os::Bottle initMsg;
        paramHelper->initializeParams(rf, initMsg);
        paramHelp::printBottle(initMsg);

        // Open ports for communicating with other modules
        if(!paramHelper->init(std::string(MODULE_NAME).c_str())){ fprintf(stderr, "Error while initializing parameter helper. Closing module.\n"); return false; }
        rpcPort.open(("/"+std::string(MODULE_NAME)+"/rpc").c_str());
        setName(MODULE_NAME);
        attach(rpcPort);

        period = dT*1000.0;
        thr = new wb_sot::sot_VelKinCon_ctrl(period, left_arm_impedance_control,
                                                     right_arm_impedance_control,
                                                     torso_impedance_control,
                                                     paramHelper);

        /** if has flag startNow, start contol right away */
        ctrl_started = rf.check("startNow");

        switch_port.open("/sot_VelKinCon/switch:i");

        if(ctrl_started)
        {
            if(!thr->start())
            {
                std::cerr<<"Problems Starting module, closing!"<<std::endl;
                delete thr;
                return false;
            }
            std::cout<<"Starting Module"<<std::endl;
        }
        else
            std::cout<<"Ctrl started in STOP mode! To run it send a true to idle:i port!"<<std::endl;

        return true;
    }

    bool interruptModule()
    {
        if(thr)
            thr->suspend();
        rpcPort.interrupt();
        return true;
    }

    virtual bool close()
    {
        if(paramHelper){    paramHelper->close();       delete paramHelper;     paramHelper = 0;    }
        if(thr){            thr->stop();                delete thr;             thr = 0;            }

        //closing ports
        switch_port.close();
        rpcPort.close();

        printf("[PERFORMANCE INFORMATION]:\n");
        printf("Expected period %d ms.\nReal period: %3.1f+/-%3.1f ms.\n", period, avgTime, stdDev);
        printf("Real duration of 'run' method: %3.1f+/-%3.1f ms.\n", avgTimeUsed, stdDevUsed);
        if(avgTimeUsed<0.5*period)
            printf("Next time you could set a lower period to improve the controller performance.\n");
        else if(avgTime>1.3*period)
            printf("The period you set was impossible to attain. Next time you could set a higher period.\n");

        return true;
    }

    bool respond(const yarp::os::Bottle& cmd, yarp::os::Bottle& reply)
    {
        if(thr && ctrl_started) {
            paramHelper->lock();
            if(!paramHelper->processRpcCommand(cmd, reply))
                reply.addString( (string("Command ")+cmd.toString().c_str()+" not recognized.").c_str());
            paramHelper->unlock();
        } else ROS_ERROR("Trying to call rpc while the module is not started");

        // if reply is empty put something into it, otherwise the rpc communication gets stuck
        if(reply.size()==0)
            reply.addString( (string("Command ")+cmd.toString().c_str()+" received.").c_str());
        return true;
    }


    virtual double getPeriod(){return 1.0;}


    virtual bool updateModule()
    {
        std::string start = "start";
        std::string stop = "stop";

        yarp::os::Bottle* bot = switch_port.read(false);

        if(!bot==NULL)
        {
            if((start.compare(bot->get(0).asString()) == 0) && !ctrl_started)
            {
                ctrl_started = true;
                if(!thr->start())
                {
                    std::cout<<"Problems Starting module, closing!"<<std::endl;
                    delete thr;
                    return false;
                }
                std::cout<<"Starting Module"<<std::endl;
            }

            if((stop.compare(bot->get(0).asString()) == 0) && ctrl_started)
            {
                ctrl_started = false;
                thr->stop();
                std::cout<<"Stopping Module"<<std::endl;
            }
        }


        if (thr==0)
        {
            printf("ControlThread pointers are zero\n");
            return false;
        }

        thr->getEstPeriod(avgTime, stdDev);
        thr->getEstUsed(avgTimeUsed, stdDevUsed);     // real duration of run()
        if(avgTime > 1.3 * period)
        {
            printf("[WARNING] Control loop is too slow. Real period: %3.3f+/-%3.3f. Expected period %d.\n", avgTime, stdDev, period);
            printf("Duration of 'run' method: %3.3f+/-%3.3f.\n", avgTimeUsed, stdDevUsed);
        }

        return true;
    }
};

int main(int argc, char* argv[])
{
    //Creating and preparing the Resource Finder
    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile(CONF_NAME); //default config file name.
    rf.setDefaultContext(MODULE_NAME);  //when no parameters are given to the module this is the default context
    rf.configure(argc,argv);

    if (rf.check("help"))
    {
        cout<< "Possible parameters"                                                                                                    << endl << endl;
        cout<< "\t--context          :Where to find an user defined .ini file within $ICUB_ROOT/app e.g. /locomotionCtrl/conf"          <<endl;
        cout<< "\t--from             :Name of the file.ini to be used for calibration."                                                 <<endl;
        cout<< "\t--dT               :Period used by the module. Default set to 0.025s."                                                <<endl;
        return 0;
    }

    yarp::os::Network yarp;

    if (!yarp.checkNetwork())
    {
        fprintf(stderr,"Sorry YARP network is not available\n");
        return -1;
    }

    sot_VelKinCon_module mod;
    mod.configure(rf);

    return mod.runModule();
}

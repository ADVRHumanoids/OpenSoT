/*
 * Copyright (C) 2014 Walkman
 * Author: Alessio Rocchi, Enrico Mingo
 * email:  alessio.rocchi@iit.it, enrico.mingo@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU Lesser General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#include <OpenSoT/utils/logger/L.h>

void OpenSoT::L::udpate(double t, const yarp::sig::Vector &q_dot)
{
    _current_log << "(" << t << ",";
    for(unsigned int i = 0; i < q_dot.size(); ++i)
        _current_log << q_dot[i] << ",";
                    /*
        << (DHS.leftArm->getActualPose())(0,3) << ","       // 1
        << (DHS.leftArm->getActualPose())(1,3) << ","
        << (DHS.leftArm->getActualPose())(2,3) << ","
        << (DHSns.leftArm->getActualPose())(0,3) << ","     // 4
        << (DHSns.leftArm->getActualPose())(1,3) << ","
        << (DHSns.leftArm->getActualPose())(2,3) << ","
        << (DHS.leftArm->getReference())(0,3) << ","        // 7
        << (DHS.leftArm->getReference())(1,3) << ","
        << (DHS.leftArm->getReference())(2,3) << ","
        << r  << ","                                        // 10
        << p  << ","
        << y  << ","
        << r_ns  << ","                                     // 13
        << p_ns  << ","
        << y_ns  << ","
        << r_ref  << ","                                    // 16
        << p_ref  << ","
        << y_ref  << ","
        << e_com << ","                                     // 19
        << e_com_ns << ","
        << e_arms << ","                                    // 21
        << e_arms_ns << ","
        << e_post << ","                                    // 23
        << e_post_ns << ","
        << t_loop << ","                                    // 25
        << t_loopns */
    {
        typedef std::map<TaskPtr, flushers::TaskFlusher::Ptr>::const_iterator it_t;
        typedef std::map<ConstraintPtr, flushers::ConstraintFlusher::Ptr>::const_iterator it_c;

        for(it_t it = taskFlushers.begin(); it != taskFlushers.end(); ++it)
        {
            it->second->updateSolution(q_dot);
            _current_log << it->second;
        }

        for(it_c it = constraintFlushers.begin(); it != constraintFlushers.end(); ++it)
        {
            it->second->updateSolution(q_dot);
            _current_log << it->second;
        }
    }
    _current_log << ")," << std::endl;
}

bool OpenSoT::L::open(std::string logName)
{
    if(must_append.count(logName) > 0)
    {
        _current_log.open(logName.c_str(), std::fstream::app);
        must_append[logName]++;
    }
    else
    {
        _current_log.open(logName.c_str());
        must_append[logName] = 1;
    }

    _current_log << "#! /usr/bin/env python" << std::endl
         << std::endl
         << "import numpy as np" << std::endl
         << "import matplotlib" << std::endl
         << "from matplotlib.pyplot import *" << std::endl;
    /* @TODO this should be done by the flusher
    _log << "#t, x, y, z, r, p, y,"                 // 0-6
         << " xns, yns, zns, rns, pns, yns,"        // 7-12
         << " xref, yref, zref, rref, pref, yref,"  // 13-18
         << " e_com, e_com_ns,"                     // 19-20
         << " e_arms, e_arms_ns,"                   // 21-22
         << " e_post, e_post_ns,"                   // 23-24
         << " t_loop, t_loop_nva" << std::endl;     // 25-26
     */

    // @TODO here we should append a number to test_data
    _current_log << "data = np.array((";

    return true;
}

bool OpenSoT::L::close()
{

    /*

    _log << "));" << std::endl;

    _log << "se = figure('"<< smoothing_strategy_stream.str() << "- Cartesian Errors',figsize=(10.27,7.68));" << std::endl;
    std::string smoothing = smoothing_params_stream.str();
    std::string no_smoothing = regular_params_stream.str();

    _log << "subplot(3,2,1); p = plot(test_data[:,0], test_data[:,(1,4,7)]); title('l_arm x');" << std::endl;
    _log << "legend(p,('" << smoothing << "', '" << no_smoothing << "', 'reference'));" << std::endl;
    _log << "ylabel('hand position [m]'); xlabel('t [s]');" << std::endl << std::endl;

    _log << "subplot(3,2,2); p = plot(test_data[:,0], test_data[:,(2,5,8)]); title('l_arm y');" << std::endl;
    _log << "legend(p,('" << smoothing << "', '" << no_smoothing << "', 'reference'));" << std::endl;
    _log << "ylabel('hand position [m]'); xlabel('t [s]');" << std::endl << std::endl;

    _log << "subplot(3,2,3); p = plot(test_data[:,0], test_data[:,(3,6,9)]); title('l_arm z');" << std::endl;
    _log << "legend(p,('" << smoothing << "', '" << no_smoothing << "', 'reference'));" << std::endl;
    _log << "ylabel('hand position [m]'); xlabel('t [s]');" << std::endl << std::endl;

    _log << "subplot(3,2,4); p = plot(test_data[:,0], test_data[:,(10,13,16)]); title('l_arm r');" << std::endl;
    _log << "legend(p,('" << smoothing << "', '" << no_smoothing << "', 'reference'));" << std::endl;
    _log << "ylabel('hand orientation [rad]'); xlabel('t [s]');" << std::endl << std::endl;

    _log << "subplot(3,2,5); p = plot(test_data[:,0], test_data[:,(11,14,17)]); title('l_arm p');" << std::endl;
    _log << "legend(p,('" << smoothing << "', '" << no_smoothing << "', 'reference'));" << std::endl;
    _log << "ylabel('hand orientation [rad]'); xlabel('t [s]');" << std::endl << std::endl;

    _log << "subplot(3,2,6); p = plot(test_data[:,0], test_data[:,(12,15,18)]); title('l_arm y');" << std::endl;
    _log << "legend(p,('" << smoothing << "', '" << no_smoothing << "', 'reference'));" << std::endl;
    _log << "ylabel('hand orientation [rad]'); xlabel('t [s]');" << std::endl << std::endl;

    _log << "et = figure('"<< smoothing_strategy_stream.str() << "- Task Errors',figsize=(8,6));" << std::endl << std::endl;

    _log << "subplot(2,2,1); p = plot(test_data[:,0],test_data[:,(25, 26)]);" << std::endl;
    _log << "title('Computation Time');" << std::endl;
    _log << "legend(p,('" << smoothing << "', '" << no_smoothing << "'));" << std::endl;
    _log << "ylabel('Solve Time [s]'); xlabel('t [s]');" << std::endl << std::endl;

    _log << "subplot(2,2,2); p = plot(test_data[:,0],test_data[:,(19, 20)]);" << std::endl;
    _log << "title('CoM_XY Task Error');" << std::endl;
    _log << "legend(p,('" << smoothing << "', '" << no_smoothing << "'));" << std::endl;
    _log << "ylabel('norm2 of task error'); xlabel('t [s]');" << std::endl << std::endl;

    _log << "subplot(2,2,3); p = plot(test_data[:,0],test_data[:,(21, 22)]);" << std::endl;
    _log << "title('l_arm + r_arm Task Error');" << std::endl;
    _log << "legend(p,('" << smoothing << "', '" << no_smoothing << "'));" << std::endl;
    _log << "ylabel('norm2 of task error'); xlabel('t [s]');" << std::endl << std::endl;

    _log << "subplot(2,2,4); p = plot(test_data[:,0],test_data[:,(23, 24)]);" << std::endl;
    _log << "title('Postural Task Error');" << std::endl;
    _log << "legend(p,('" << smoothing << "', '" << no_smoothing << "'));" << std::endl;
    _log << "ylabel('norm2 of task error'); xlabel('t [s]');" << std::endl << std::endl;

    if(params.strategy == STRATEGY_CARTESIAN_TUNING_1) {
        _log << "se.savefig('" << getPlotFilename(STRATEGY_CARTESIAN_TUNING_1, TEST_SCA_CT1_DISTANCES_FILE) << "', format='eps', transparent=True);" << std::endl;
        _log << "et.savefig('" << getPlotFilename(STRATEGY_CARTESIAN_TUNING_1, TEST_SCA_CT1_ERRORS_FILE)    << "',format='eps',transparent=True);" << std::endl;
    } else if(params.strategy == STRATEGY_COM_TUNING) {
        _log << "se.savefig('" << getPlotFilename(STRATEGY_COM_TUNING, TEST_SCA_COM_DISTANCES_FILE) << "', format='eps', transparent=True);" << std::endl;
        _log << "et.savefig('" << getPlotFilename(STRATEGY_COM_TUNING, TEST_SCA_COM_ERRORS_FILE)    << "',format='eps',transparent=True);" << std::endl;
    } else if(params.strategy == STRATEGY_BOUNDSCALING_TUNING) {
        _log << "se.savefig('" << getPlotFilename(STRATEGY_BOUNDSCALING_TUNING, TEST_SCA_BST_DISTANCES_FILE) << "', format='eps', transparent=True);" << std::endl;
        _log << "et.savefig('" << getPlotFilename(STRATEGY_BOUNDSCALING_TUNING, TEST_SCA_BST_ERRORS_FILE)    << "',format='eps',transparent=True);" << std::endl;
    } else if(params.strategy == STRATEGY_EPS_TUNING) {
        _log << "se.savefig('" << getPlotFilename(STRATEGY_EPS_TUNING, TEST_SCA_EPS_DISTANCES_FILE) << "', format='eps', transparent=True);" << std::endl;
        _log << "et.savefig('" << getPlotFilename(STRATEGY_EPS_TUNING, TEST_SCA_EPS_ERRORS_FILE)    << "',format='eps',transparent=True);" << std::endl;
    } else if(params.strategy == STRATEGY_SMALLER_STACK) {
        _log << "se.savefig('" << getPlotFilename(STRATEGY_SMALLER_STACK, TEST_SCA_SS_DISTANCES_FILE) << "', format='eps', transparent=True);" << std::endl;
        _log << "et.savefig('" << getPlotFilename(STRATEGY_SMALLER_STACK, TEST_SCA_SS_ERRORS_FILE)    << "',format='eps',transparent=True);" << std::endl;
    } else if(params.strategy == STRATEGY_POSTURAL_TUNING) {
        _log << "se.savefig('" << getPlotFilename(STRATEGY_POSTURAL_TUNING, TEST_SCA_PT_DISTANCES_FILE) << "', format='eps', transparent=True);" << std::endl;
        _log << "et.savefig('" << getPlotFilename(STRATEGY_POSTURAL_TUNING, TEST_SCA_PT_ERRORS_FILE)    << "',format='eps',transparent=True);" << std::endl;
    } else if(params.strategy == STRATEGY_DISTANCE_SMOOTHING) {
        _log << "se.savefig('" << TEST_SCA_BST_DISTANCES_FILE << ".eps', format='eps', transparent=True);" << std::endl;
        _log << "et.savefig('" << TEST_SCA_BST_ERRORS_FILE << ".eps',format='eps',transparent=True);" << std::endl;
    } else {
        std::cerr << "Unhandled exception at line " << __LINE__ << std::endl;
        exit(1);
    }
    _log << "show(block=True)" << std::endl;

     */
    _current_log.close();
}

OpenSoT::L::~L()
{
    if(_current_log.is_open())
        this->close();
}

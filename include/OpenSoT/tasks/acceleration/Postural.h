/*
 * Copyright (C) 2017 IIT-ADVR
 * Authors: Arturo Laurenzi
 * email:  arturo.laurenzi@iit.it
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


#ifndef __TASKS_ACCELERATION_POSTURAL_H__
#define __TASKS_ACCELERATION_POSTURAL_H__

#include <OpenSoT/Task.h>
#include <OpenSoT/utils/Affine.h>
#include <XBotInterface/ModelInterface.h>

namespace OpenSoT { namespace tasks { namespace acceleration {
    
    class Postural : public OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd> {
      
    public:
        
        typedef boost::shared_ptr<Postural> Ptr;
        
        Postural(const XBot::ModelInterface& robot,
                 AffineHelper qddot = AffineHelper());

        Postural(const XBot::ModelInterface& robot,
                 const int x_size);
        
        virtual void _update(const Eigen::VectorXd& x);
        
        void setReference(const Eigen::VectorXd& qref);
        void setReference(const Eigen::VectorXd& qref, const Eigen::VectorXd& dqref);
        void setReference(const Eigen::VectorXd& qref, const Eigen::VectorXd& dqref,
                          const Eigen::VectorXd& ddqref);

        void setLambda(double lambda1, double lambda2);
        virtual void setLambda(double lambda);
        
        virtual void _log(XBot::MatLogger::Ptr logger);

        
        
    private:
        
        const XBot::ModelInterface& _robot;
        AffineHelper _qddot;
        AffineHelper _postural_task;
        
        int _na;
        
        Eigen::VectorXd _qddot_d, _qddot_ref, _qref, _qdot, _q, _qdot_ref;
        Eigen::MatrixXd _Jpostural;

        double _lambda2;
        
        
    };
    
    
} } }





#endif

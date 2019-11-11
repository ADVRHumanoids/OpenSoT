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


#ifndef __CONSTRAINT_ACCELERATION_DYNFEAS_H__
#define __CONSTRAINT_ACCELERATION_DYNFEAS_H__

#include <OpenSoT/Constraint.h>
#include <OpenSoT/utils/Affine.h>
#include <XBotInterface/ModelInterface.h>

namespace OpenSoT { namespace constraints { namespace acceleration {
    
class DynamicFeasibility : public Constraint<Eigen::MatrixXd, Eigen::VectorXd> {
  
public:
    
    typedef boost::shared_ptr<DynamicFeasibility> Ptr;
    
    DynamicFeasibility(const std::string constraint_id, 
                       const XBot::ModelInterface& robot,
                       const AffineHelper& qddot, 
                       const std::vector<AffineHelper>& wrenches,
                       const std::vector<std::string>& contact_links
                       );

    virtual void update(const Eigen::VectorXd& x);
    
    Eigen::VectorXd checkConstraint(const Eigen::VectorXd& x);
    
    
    
private:
    
        const XBot::ModelInterface& _robot;
        AffineHelper _qddot;
        std::vector<AffineHelper> _wrenches;
        std::vector<std::string> _contact_links;
        AffineHelper _dyn_constraint;
        
        std::vector<bool> _enabled_contacts;
        
        Eigen::VectorXd _h, _hu;
        Eigen::MatrixXd _B, _Bu, _Jtmp;
        Eigen::MatrixXd _Jf;
    
};
    
} } }





#endif

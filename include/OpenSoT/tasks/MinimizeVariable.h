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


#ifndef __CONSTRAINT_MINIMIZE_VARIABLE_H__
#define __CONSTRAINT_MINIMIZE_VARIABLE_H__

#include <OpenSoT/Task.h>
#include <OpenSoT/utils/Affine.h>
#include <XBotInterface/ModelInterface.h>

namespace OpenSoT { namespace tasks  {
    
class MinimizeVariable : public Task<Eigen::MatrixXd, Eigen::VectorXd> {
  
public:
    
    typedef boost::shared_ptr<MinimizeVariable> Ptr;
    
    MinimizeVariable(std::string task_id, 
                      const AffineHelper& variable
                      );
    
    bool setReference(const Eigen::VectorXd& ref);
    
    virtual void _update(const Eigen::VectorXd& x);
    
    
    
private:
    
        AffineHelper _var;
        Eigen::VectorXd _ref;
    
};
    
} } 





#endif

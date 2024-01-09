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


#ifndef __CONSTRAINT_GENERIC_CONSTRAINT_H__
#define __CONSTRAINT_GENERIC_CONSTRAINT_H__

#include <OpenSoT/Constraint.h>
#include <OpenSoT/utils/Affine.h>
#include <xbot2_interface/xbotinterface2.h>

namespace OpenSoT { namespace constraints  {
    
class GenericConstraint : public Constraint<Eigen::MatrixXd, Eigen::VectorXd> {
  
public:
    
    enum class Type {
        /** The GenericConstraint is of the type:
          *     lb <= x <= ub
         **/
        BOUND,
        /** The GenericConstraint is of the type:
         *      L <= Ax <= U
         **/
        CONSTRAINT
    };
    
    typedef std::shared_ptr<GenericConstraint> Ptr;

    /**
     * @brief GenericConstraint specific constructor for BOUND type
     * @param constraint_id
     * @param upper_bound
     * @param lower_bound
     */
    GenericConstraint(std::string constraint_id,
                      const Eigen::VectorXd& upper_bound,
                      const Eigen::VectorXd& lower_bound,
                      const int x_size);
    
    GenericConstraint(std::string constraint_id,
                      const AffineHelper& variable,
                      const Eigen::VectorXd& upper_bound,
                      const Eigen::VectorXd& lower_bound,
                      const Type constraint_type
                      );

    bool setConstraint(const AffineHelper& var,
                       const Eigen::VectorXd& upper_bound,
                       const Eigen::VectorXd& lower_bound);
    
    bool setBounds(const Eigen::VectorXd& upper_bound,
                   const Eigen::VectorXd& lower_bound);

    virtual void update(const Eigen::VectorXd& x);

    Type getType(){return _type;}
    
    
    
private:
    
        AffineHelper _var;
        
        Eigen::VectorXd _ub, _lb;
        
        Type _type;


    
};
    
} } 



#endif

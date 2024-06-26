/*
 * Copyright (C) 2018 Cogimon
 * Authors: Enrico Mingo Hoffman, Arturo Laurenzi
 * email:  enrico.mingo@iit.it, arturo.laurenzi@iit.it
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

#ifndef _CONSTRAINT_FORCE_COP_H_
#define _CONSTRAINT_FORCE_COP_H_

#include <OpenSoT/Constraint.h>
#include <xbot2_interface/xbotinterface2.h>
#include <kdl/frames.hpp>
#include <OpenSoT/utils/Affine.h>
#include <OpenSoT/utils/Piler.h>
#include <OpenSoT/constraints/Aggregated.h>


#include <Eigen/Dense>

namespace OpenSoT {
   namespace constraints {
       namespace force {
       class CoP: public Constraint<Eigen::MatrixXd, Eigen::VectorXd> {
       public:
           typedef std::shared_ptr<CoP> Ptr;

        /**
         * @brief CoP constructor of the CoP constraint
         * @param model of the robot
         * @param wrenche affine helper
         * @param contact_link frame in contact with the environment
         * @param X_Lims [xl, xu] limits w.r.t. contact frame
         * @param Y_Lims [yl, yu] limits
         */
            CoP(const std::string& contact_link,
                const AffineHelper& wrench,
                XBot::ModelInterface& model,
                const Eigen::Vector2d& X_Lims,
                const Eigen::Vector2d& Y_Lims);

            virtual void update();


       private:


            std::string _contact_link;

            XBot::ModelInterface& _model;

            double _xl, _xu;
            double _yl, _yu;

            Eigen::MatrixXd _Ai;
            Eigen::MatrixXd _tmp;

            Eigen::MatrixXd __A;

            Eigen::Affine3d _T;
            Eigen::Affine3d _Ti;
            Eigen::MatrixXd _Ad;

            OpenSoT::AffineHelper _wrench;
            OpenSoT::AffineHelper _CoP;
       };

       class CoPs: public Constraint<Eigen::MatrixXd, Eigen::VectorXd> {
       public:
           typedef std::shared_ptr<CoPs> Ptr;

           CoPs(const std::vector<AffineHelper>& wrench,
                const std::vector<std::string>& contact_name,
                XBot::ModelInterface &robot,
                const std::vector<Eigen::Vector2d>& X_Lims,
                const std::vector<Eigen::Vector2d>& Y_Lims);

           void update();


       private:
           OpenSoT::constraints::Aggregated::Ptr _internal_constraint;
           void generateBounds();

       };

/** OLD API **
//       /**
//        * @brief The CoP class implements a constraint which constraints the contact wrenches to create a CoP which lies inside the
//        * contact foot/hand
//        */
//       class CoP: public Constraint<Eigen::MatrixXd, Eigen::VectorXd> {
//       public:
//        typedef std::shared_ptr<CoP> Ptr;

//           /**
//         * @brief CoP constructor of the CoP constraint
//         * @param model of the robot
//         * @param wrenches affine helper
//         * @param contact_links vector of robot links which considered in contact with the environment
//         * @param X_Lims [xl, xu] limits (we assume same limits for all the contacts)
//         * @param Y_Lims [yl, yu] limits (we assume same limits for all the contacts)
//         */
//        CoP(XBot::ModelInterface& model,
//            const std::vector<OpenSoT::AffineHelper>& wrenches,
//            const std::vector<std::string>& contact_links,
//            const Eigen::Vector2d& X_Lims, const Eigen::Vector2d& Y_Lims);



//       private:
//        virtual void update(const Eigen::VectorXd& x);
//        std::vector<std::string> _contact_links;

//        std::vector<bool> _enabled_contacts;
//        XBot::ModelInterface& _model;

//        double _xl, _xu;
//        double _yl, _yu;

//        Eigen::MatrixXd _Ai;
//        Eigen::MatrixXd _tmp;

//        Eigen::MatrixXd __A;

//        Eigen::Affine3d _T;
//        Eigen::Affine3d _Ti;
//        Eigen::MatrixXd _Ad;

//        OpenSoT::AffineHelper _wrenches;
//        OpenSoT::AffineHelper _CoP;

//       };
       }
   }
}

#endif

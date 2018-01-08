/*
 * Copyright (C) 2017 IIT-ADVR
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

#ifndef _OPENSOT_FLOATING_BASE_ESTIMATION_H_
#define _OPENSOT_FLOATING_BASE_ESTIMATION_H_

#include <boost/shared_ptr.hpp>
#include <XBotInterface/ModelInterface.h>

namespace OpenSoT {
    /**
     * @brief The FloatingBaseEstimation class interface to a floating base estimator
     */
    class FloatingBaseEstimation{
    public:
        typedef boost::shared_ptr<FloatingBaseEstimation> Ptr;

        FloatingBaseEstimation(XBot::ModelInterface::Ptr model,
                               XBot::ImuSensor::ConstPtr imu,
                               std::vector<std::string> contact_links,
                               const Eigen::MatrixXd& contact_matrix):
            _model(model),
            _imu(imu),
            _contact_matrix(contact_matrix)
        {
            if(_contact_matrix.rows()>6)
                throw std::invalid_argument("contact_matrix rows > 6 is invalid");
            if(_contact_matrix.cols() != 6)
                throw std::invalid_argument("contact_matrix cols != 6 is invalid");

            for(unsigned int i = 0; i < contact_links.size(); ++i)
                _contact_links[contact_links[i]] = true;

        }

        ~FloatingBaseEstimation()
        {

        }

        virtual bool update(double dT) = 0;

        virtual void log(XBot::MatLogger::Ptr logger)
        {

        }

        /**
         * @brief getFloatingBaseVelocity
         * @return a vector of six dimension with the floating base virtual joint velocities
         */
        const Eigen::VectorXd& getFloatingBaseVelocity()
        {
            _Qdot = _qdot.segment(0,6);
            return _Qdot;
        }

        /**
         * @brief getFloatingBasePose
         * @return the transformation of the floating_base in world frame
         */
        const Eigen::Affine3d& getFloatingBasePose()
        {
            _model->getFloatingBasePose(_fb_pose);
            return _fb_pose;
        }

        /**
         * @brief getFloatingBaseJoints
         * @return the value of the virtual joint position of the floating base
         */
        const Eigen::Vector6d& getFloatingBaseJoints()
        {
            _Q = _q.segment(0,6);
            return _Q;
        }

        /**
         * @brief setContactState permits to set the stus of a link between contact and
         * free
         * @param contact_link
         * @param state true if the link is in contact
         * @return false if the contact is not in the contact list
         */
        virtual bool setContactState(const std::string& contact_link, const bool state)
        {
            auto it = _contact_links.find(contact_link);
            if(it != _contact_links.end())
                it->second = state;
            else{
                XBot::Logger::error("contact_link %s is not in contact_link list \n", contact_link.c_str());
                return false;}

            return true;
        }

    protected:
        XBot::ModelInterface::Ptr _model;
        XBot::ImuSensor::ConstPtr _imu;
        std::map<std::string,bool> _contact_links;
        Eigen::MatrixXd _contact_matrix;
        Eigen::VectorXd _q;
        Eigen::VectorXd _qdot;
        Eigen::Affine3d _fb_pose;

        Eigen::VectorXd _Qdot;
        Eigen::Vector6d _Q;

    };
}

#endif

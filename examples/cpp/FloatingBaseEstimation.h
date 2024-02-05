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

#include <memory>
#include <xbot2_interface/xbotinterface2.h>
#include <matlogger2/matlogger2.h>
#include <xbot2_interface/logger.h>

namespace OpenSoT {
    /**
     * @brief The FloatingBaseEstimation class interface to a floating base estimator
     */
    class FloatingBaseEstimation{

        /**
         * @brief The Update enum define the type of update of the model for the estimation classes
         */
    public: enum Update
        {
            None = 0, // NO update is done
            Linear = 1, //ONLY the linear part is updated
            Angular = 2, //ONLY the angular part is updated
            All = 3 //BOTH linear and angular part are updated
        };

    public:
        typedef std::shared_ptr<FloatingBaseEstimation> Ptr;

        FloatingBaseEstimation(XBot::ModelInterface::Ptr model,
                               std::vector<std::string> contact_links,
                               const Eigen::MatrixXd& contact_matrix):
            _model(model),
            _contact_matrix(contact_matrix)
        {
            _Qdot.setZero(6);
            model->getJointVelocity(_qdot);

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

        virtual bool update(OpenSoT::FloatingBaseEstimation::Update update =
                OpenSoT::FloatingBaseEstimation::Update::None) = 0;

        virtual void log(XBot::MatLogger2::Ptr logger)
        {

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

        const Eigen::VectorXd& getFloatingBaseTwist() const
        {
            return _Qdot;
        }

        void getFloatingBaseTwist(Eigen::VectorXd& Q_dot)
        {
            Q_dot = _Qdot;
        }

    protected:
        XBot::ModelInterface::Ptr _model;
        std::map<std::string,bool> _contact_links;
        Eigen::MatrixXd _contact_matrix;
        Eigen::VectorXd _qdot;
        Eigen::VectorXd _Qdot;

    };
}

#endif

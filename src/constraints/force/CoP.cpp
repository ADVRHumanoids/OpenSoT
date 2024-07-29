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

#include <OpenSoT/constraints/force/CoP.h>
#include <memory>

using namespace OpenSoT::constraints::force;

/* CoP */
CoP::CoP(const std::string& contact_link,
         const AffineHelper& wrench,
         XBot::ModelInterface& model,
         const Eigen::Vector2d& X_Lims,
         const Eigen::Vector2d& Y_Lims):
    Constraint(contact_link+"_CoP", wrench.getInputSize()),
    _model(model),
    _xl(X_Lims[0]), _xu(X_Lims[1]),
    _yl(Y_Lims[0]),_yu(Y_Lims[1]),
    _contact_link(contact_link),
    _wrench(wrench)
{
    _Ai.resize(4,6); _Ai.setZero(4,6);
    _Ai(0,2) =  _xl;  _Ai(0,4) =  1.;
    _Ai(1,2) = -_xu;  _Ai(1,4) = -1.;
    _Ai(2,2) =  _yl;  _Ai(2,3) = -1.;
    _Ai(3,2) = -_yu;  _Ai(3,3) =  1.;

    _Ad.resize(6,6); _Ad.setZero(6,6);

    __A.resize(4, 6);
    __A.setZero(__A.rows(), __A.cols());

    update();
}

void CoP::update()
{
    __A.setZero(__A.rows(), __A.cols());


    _Ad.setZero(6,6);

    _model.getPose(_contact_link, _T);
    _Ti = _T.inverse();

    _Ad.block<3,3>(0,0) = _Ti.linear();
    _Ad.block<3,3>(3,3) = _Ti.linear();

    __A = _Ai*_Ad;

    _CoP = __A * _wrench;
    _Aineq = _CoP.getM();
    _bUpperBound = -_CoP.getq();
    _bLowerBound = -1.0e20*Eigen::VectorXd::Ones(__A.rows());
}

/* CoPs */
CoPs::CoPs(const std::vector<AffineHelper>& wrench,
           const std::vector<std::string>& contact_name,
           XBot::ModelInterface &robot,
           const std::vector<Eigen::Vector2d>& X_Lims,
           const std::vector<Eigen::Vector2d>& Y_Lims):
    Constraint("CoPs", wrench[0].getInputSize())
{
    std::list<ConstraintPtr> constraint_list;
    for(unsigned int i = 0; i < contact_name.size(); ++i){
        auto cop = ::std::make_shared<CoP>(contact_name[i], wrench[i], robot, X_Lims[i], Y_Lims[i]);
        constraint_list.push_back(cop);
    }

    _internal_constraint = std::make_shared<OpenSoT::constraints::Aggregated>
            (constraint_list, wrench[0].getInputSize());

    update();
}

void CoPs::update()
{
    _internal_constraint->update();
    generateBounds();
}

void CoPs::generateBounds()
{
    _Aineq = _internal_constraint->getAineq();
    _bUpperBound = _internal_constraint->getbUpperBound();
    _bLowerBound = _internal_constraint->getbLowerBound();
}


///* CoPs  OLD IMPLEMENTATION */
//CoP::CoP(XBot::ModelInterface &model, const std::vector<OpenSoT::AffineHelper> &wrenches,
//         const std::vector<std::string> &contact_links, const Eigen::Vector2d &X_Lims, const Eigen::Vector2d &Y_Lims):
//    Constraint("CoP", wrenches[0].getInputSize()),
//    _model(model),
//    _xl(X_Lims[0]),_xu(X_Lims[1]),
//    _yl(Y_Lims[0]),_yu(Y_Lims[1]),
//    _enabled_contacts(wrenches.size(), true),
//    _contact_links(contact_links)
//{
//    if(wrenches.size() != contact_links.size())
//        throw std::invalid_argument("wrench and contact_links has different sizes!");

//     _wrenches.setZero(wrenches[0].getInputSize(), 0);
//     for(auto& w : wrenches)
//         _wrenches = _wrenches / w;


//    _Ai.resize(4,6); _Ai.setZero(4,6);
//    _Ai(0,2) =  _xl;  _Ai(0,4) =  1.;
//    _Ai(1,2) = -_xu;  _Ai(1,4) = -1.;
//    _Ai(2,2) =  _yl;  _Ai(2,3) = -1.;
//    _Ai(3,2) = -_yu;  _Ai(3,3) =  1.;
//    _tmp.resize(4,6); _tmp.setZero(4,6);

//    _Ad.resize(6,6); _Ad.setZero(6,6);

//    __A.resize(4*_contact_links.size(), 6*_contact_links.size());
//    __A.setZero(__A.rows(), __A.cols());

//    update(Eigen::VectorXd());
//}

//void CoP::update(const Eigen::VectorXd &x)
//{
//    __A.setZero(__A.rows(), __A.cols());
//    for(unsigned int i = 0; i < _contact_links.size(); ++i)
//    {
//        _Ad.setZero(6,6);

//        _model.getPose(_contact_links[i], _T);
//        _Ti = _T.inverse();

//        _Ad.block<3,3>(0,0) = _Ti.linear();
//        _Ad.block<3,3>(3,3) = _Ti.linear();

//        _tmp = _Ai*_Ad;

//        __A.block<4,6>(4*i,6*i) = _tmp;
//    }

//    _CoP = __A * _wrenches;
//    _Aineq = _CoP.getM();
//    _bUpperBound = -_CoP.getq();
//    _bLowerBound = -1.0e20*Eigen::VectorXd::Ones(__A.rows());
//}



/*
 * Copyright (C) 2014 Walkman
 * Author: Cheng Fang
 * email:  cheng.fang@iit.it
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

#include <OpenSoT/constraints/velocity/SelfCollisionAvoidance.h>
#include <yarp/math/Math.h>
#include <eigen_conversions/eigen_kdl.h>

using namespace yarp::math;

using namespace OpenSoT::constraints::velocity;

using namespace Eigen;

const double SMALL_NUM = pow(10.0, -5);

SelfCollisionAvoidance::SelfCollisionAvoidance(const yarp::sig::Vector& x,
                                               iDynUtils &robot,
                                               double linkPair_threshold,
                                               const double boundScaling):
    Constraint(x.size()),
    _linkPair_threshold(linkPair_threshold),
    computeLinksDistance(robot),
    robot_col(robot),
    _x_cache(x),
    _boundScaling(boundScaling) {

    std::string base_name = "Waist";
    base_index = robot_col.iDyn3_model.getLinkIndex(base_name);

    if(base_index == -1)
        std::cout << "Failed to get base_index" << std::endl;

    // used for online prediction of link pair selection

    x_larm_rarm = (struct svm_node *) malloc(40*sizeof(struct svm_node));
    x_larm_torso = (struct svm_node *) malloc(40*sizeof(struct svm_node));
    x_rarm_torso = (struct svm_node *) malloc(40*sizeof(struct svm_node));

    model_larm_rarm = svm_load_model("SCAFoI.model.larm.rarm");
    model_larm_torso = svm_load_model("SCAFoI.model.larm.torso");
    model_rarm_torso = svm_load_model("SCAFoI.model.rarm.torso");

    scale_larm_rarm.open("scale.larm.rarm");
    scale_larm_torso.open("scale.larm.torso");
    scale_rarm_torso.open("scale.rarm.torso");

    number_larm = robot_col.left_arm.joint_numbers.size();
    number_rarm = robot_col.right_arm.joint_numbers.size();
    number_torso = robot_col.torso.joint_numbers.size();
    number_head = robot_col.head.joint_numbers.size();
    number_lleg = robot_col.left_leg.joint_numbers.size();
    number_rleg = robot_col.right_leg.joint_numbers.size();

    number_larm_rarm = number_larm + number_rarm;
    number_larm_torso = number_larm + number_torso;
    number_rarm_torso = number_rarm + number_torso;

    double temp_min, temp_max;
    for (unsigned int i = 0; i<number_larm_rarm; i++)
    {
        scale_larm_rarm>>temp_min;
        min_larm_rarm.push_back(temp_min);

        scale_larm_rarm>>temp_max;
        max_larm_rarm.push_back(temp_max);

        temp_larm_rarm.push_back( 2 / (temp_max-temp_min) );
    }

    for (unsigned int i = 0; i<number_larm_torso; i++)
    {
        scale_larm_torso>>temp_min;
        min_larm_torso.push_back(temp_min);

        scale_larm_torso>>temp_max;
        max_larm_torso.push_back(temp_max);

        temp_larm_torso.push_back( 2 / (temp_max-temp_min) );
    }

    for (unsigned int i = 0; i<number_rarm_torso; i++)
    {
        scale_rarm_torso>>temp_min;
        min_rarm_torso.push_back(temp_min);

        scale_rarm_torso>>temp_max;
        max_rarm_torso.push_back(temp_max);

        temp_rarm_torso.push_back( 2 / (temp_max-temp_min) );
    }

    d_recent_L_R_Arms[0] = 0.0;
    d_recent_L_R_Arms[1] = 0.0;
    d_recent_L_R_Arms[2] = 0.0;

    d_recent_L_Arm_Torso[0] = 0.0;
    d_recent_L_Arm_Torso[1] = 0.0;
    d_recent_L_Arm_Torso[2] = 0.0;

    d_recent_R_Arm_Torso[0] = 0.0;
    d_recent_R_Arm_Torso[1] = 0.0;
    d_recent_R_Arm_Torso[2] = 0.0;

    d_threshold_upper = 0.2;
    d_threshold_lower = 0.15;

    /*//////////////////whiteList_L_R_Arms////////////////////*/

    std::string linkA1 = "LSoftHandLink";
    std::string linkA2 = "LWrMot3";
    std::string linkA3 = "LWrMot2";
    std::string linkA4 = "LForearm";
    std::string linkA5 = "LElb";
    std::string linkA6 = "LShy";
    std::string linkA7 = "LShr";
    std::string linkA8 = "LShp";

    std::string linkB1 = "RSoftHandLink";
    std::string linkB2 = "RWrMot3";
    std::string linkB3 = "RWrMot2";
    std::string linkB4 = "RForearm";
    std::string linkB5 = "RElb";
    std::string linkB6 = "RShy";
    std::string linkB7 = "RShr";
    std::string linkB8 = "RShp";

    whiteList_L_R_Arms.push_back(std::pair<std::string,std::string>(linkA1,linkB1));
    whiteList_L_R_Arms.push_back(std::pair<std::string,std::string>(linkA1,linkB2));
    whiteList_L_R_Arms.push_back(std::pair<std::string,std::string>(linkA1,linkB3));
    whiteList_L_R_Arms.push_back(std::pair<std::string,std::string>(linkA1,linkB4));
    whiteList_L_R_Arms.push_back(std::pair<std::string,std::string>(linkA1,linkB5));
    whiteList_L_R_Arms.push_back(std::pair<std::string,std::string>(linkA1,linkB6));
    whiteList_L_R_Arms.push_back(std::pair<std::string,std::string>(linkA1,linkB7));
    whiteList_L_R_Arms.push_back(std::pair<std::string,std::string>(linkA1,linkB8));

    whiteList_L_R_Arms.push_back(std::pair<std::string,std::string>(linkA2,linkB1));
    whiteList_L_R_Arms.push_back(std::pair<std::string,std::string>(linkA2,linkB2));
    whiteList_L_R_Arms.push_back(std::pair<std::string,std::string>(linkA2,linkB3));
    whiteList_L_R_Arms.push_back(std::pair<std::string,std::string>(linkA2,linkB4));
    whiteList_L_R_Arms.push_back(std::pair<std::string,std::string>(linkA2,linkB5));
    whiteList_L_R_Arms.push_back(std::pair<std::string,std::string>(linkA2,linkB6));
    whiteList_L_R_Arms.push_back(std::pair<std::string,std::string>(linkA2,linkB7));
    whiteList_L_R_Arms.push_back(std::pair<std::string,std::string>(linkA2,linkB8));

    whiteList_L_R_Arms.push_back(std::pair<std::string,std::string>(linkA3,linkB1));
    whiteList_L_R_Arms.push_back(std::pair<std::string,std::string>(linkA3,linkB2));
    whiteList_L_R_Arms.push_back(std::pair<std::string,std::string>(linkA3,linkB3));
    whiteList_L_R_Arms.push_back(std::pair<std::string,std::string>(linkA3,linkB4));
    whiteList_L_R_Arms.push_back(std::pair<std::string,std::string>(linkA3,linkB5));
    whiteList_L_R_Arms.push_back(std::pair<std::string,std::string>(linkA3,linkB6));
    whiteList_L_R_Arms.push_back(std::pair<std::string,std::string>(linkA3,linkB7));
    whiteList_L_R_Arms.push_back(std::pair<std::string,std::string>(linkA3,linkB8));

    whiteList_L_R_Arms.push_back(std::pair<std::string,std::string>(linkA4,linkB1));
    whiteList_L_R_Arms.push_back(std::pair<std::string,std::string>(linkA4,linkB2));
    whiteList_L_R_Arms.push_back(std::pair<std::string,std::string>(linkA4,linkB3));
    whiteList_L_R_Arms.push_back(std::pair<std::string,std::string>(linkA4,linkB4));
    whiteList_L_R_Arms.push_back(std::pair<std::string,std::string>(linkA4,linkB5));
    whiteList_L_R_Arms.push_back(std::pair<std::string,std::string>(linkA4,linkB6));
    whiteList_L_R_Arms.push_back(std::pair<std::string,std::string>(linkA4,linkB7));
    whiteList_L_R_Arms.push_back(std::pair<std::string,std::string>(linkA4,linkB8));

    whiteList_L_R_Arms.push_back(std::pair<std::string,std::string>(linkA5,linkB1));
    whiteList_L_R_Arms.push_back(std::pair<std::string,std::string>(linkA5,linkB2));
    whiteList_L_R_Arms.push_back(std::pair<std::string,std::string>(linkA5,linkB3));
    whiteList_L_R_Arms.push_back(std::pair<std::string,std::string>(linkA5,linkB4));
    whiteList_L_R_Arms.push_back(std::pair<std::string,std::string>(linkA5,linkB5));
    whiteList_L_R_Arms.push_back(std::pair<std::string,std::string>(linkA5,linkB6));
    whiteList_L_R_Arms.push_back(std::pair<std::string,std::string>(linkA5,linkB7));
    whiteList_L_R_Arms.push_back(std::pair<std::string,std::string>(linkA5,linkB8));

    whiteList_L_R_Arms.push_back(std::pair<std::string,std::string>(linkA6,linkB1));
    whiteList_L_R_Arms.push_back(std::pair<std::string,std::string>(linkA6,linkB2));
    whiteList_L_R_Arms.push_back(std::pair<std::string,std::string>(linkA6,linkB3));
    whiteList_L_R_Arms.push_back(std::pair<std::string,std::string>(linkA6,linkB4));
    whiteList_L_R_Arms.push_back(std::pair<std::string,std::string>(linkA6,linkB5));
    whiteList_L_R_Arms.push_back(std::pair<std::string,std::string>(linkA6,linkB6));
    whiteList_L_R_Arms.push_back(std::pair<std::string,std::string>(linkA6,linkB7));
    whiteList_L_R_Arms.push_back(std::pair<std::string,std::string>(linkA6,linkB8));

    whiteList_L_R_Arms.push_back(std::pair<std::string,std::string>(linkA7,linkB1));
    whiteList_L_R_Arms.push_back(std::pair<std::string,std::string>(linkA7,linkB2));
    whiteList_L_R_Arms.push_back(std::pair<std::string,std::string>(linkA7,linkB3));
    whiteList_L_R_Arms.push_back(std::pair<std::string,std::string>(linkA7,linkB4));
    whiteList_L_R_Arms.push_back(std::pair<std::string,std::string>(linkA7,linkB5));
    whiteList_L_R_Arms.push_back(std::pair<std::string,std::string>(linkA7,linkB6));
    whiteList_L_R_Arms.push_back(std::pair<std::string,std::string>(linkA7,linkB7));
    whiteList_L_R_Arms.push_back(std::pair<std::string,std::string>(linkA7,linkB8));

    whiteList_L_R_Arms.push_back(std::pair<std::string,std::string>(linkA8,linkB1));
    whiteList_L_R_Arms.push_back(std::pair<std::string,std::string>(linkA8,linkB2));
    whiteList_L_R_Arms.push_back(std::pair<std::string,std::string>(linkA8,linkB3));
    whiteList_L_R_Arms.push_back(std::pair<std::string,std::string>(linkA8,linkB4));
    whiteList_L_R_Arms.push_back(std::pair<std::string,std::string>(linkA8,linkB5));
    whiteList_L_R_Arms.push_back(std::pair<std::string,std::string>(linkA8,linkB6));
    whiteList_L_R_Arms.push_back(std::pair<std::string,std::string>(linkA8,linkB7));
    whiteList_L_R_Arms.push_back(std::pair<std::string,std::string>(linkA8,linkB8));

    /*//////////////////whiteList_L_Arm_Torso////////////////////*/

    linkA1 = "LSoftHandLink";
    linkA2 = "LWrMot3";
    linkA3 = "LWrMot2";
    linkA4 = "LForearm";
    linkA5 = "LElb";
    linkA6 = "LShy";
    linkA7 = "LShr";
    linkA8 = "LShp";

    linkB1 = "Waist";
    linkB2 = "DWL";
    linkB3 = "DWS";
    linkB4 = "DWYTorso";
    linkB5 = "TorsoProtections";

    whitelist_L_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA1,linkB1));
    whitelist_L_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA1,linkB2));
    whitelist_L_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA1,linkB3));
    whitelist_L_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA1,linkB4));
    whitelist_L_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA1,linkB5));

    whitelist_L_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA2,linkB1));
    whitelist_L_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA2,linkB2));
    whitelist_L_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA2,linkB3));
    whitelist_L_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA2,linkB4));
    whitelist_L_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA2,linkB5));

    whitelist_L_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA3,linkB1));
    whitelist_L_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA3,linkB2));
    whitelist_L_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA3,linkB3));
    whitelist_L_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA3,linkB4));
    whitelist_L_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA3,linkB5));

    whitelist_L_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA4,linkB1));
    whitelist_L_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA4,linkB2));
    whitelist_L_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA4,linkB3));
    whitelist_L_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA4,linkB4));
    whitelist_L_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA4,linkB5));

    whitelist_L_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA5,linkB1));
    whitelist_L_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA5,linkB2));
    whitelist_L_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA5,linkB3));
    whitelist_L_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA5,linkB4));
    whitelist_L_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA5,linkB5));

    whitelist_L_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA6,linkB1));
    whitelist_L_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA6,linkB2));
    whitelist_L_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA6,linkB3));
    whitelist_L_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA6,linkB4));
    whitelist_L_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA6,linkB5));

    whitelist_L_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA7,linkB1));
    whitelist_L_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA7,linkB2));
    whitelist_L_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA7,linkB3));
    whitelist_L_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA7,linkB4));
    //whitelist_L_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA7,linkB5));

    whitelist_L_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA8,linkB1));
    whitelist_L_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA8,linkB2));
    whitelist_L_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA8,linkB3));
    whitelist_L_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA8,linkB4));
    whitelist_L_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA8,linkB5));

    /*//////////////////whiteList_R_Arm_Torso////////////////////*/

    linkA1 = "RSoftHandLink";
    linkA2 = "RWrMot3";
    linkA3 = "RWrMot2";
    linkA4 = "RForearm";
    linkA5 = "RElb";
    linkA6 = "RShy";
    linkA7 = "RShr";
    linkA8 = "RShp";

    linkB1 = "Waist";
    linkB2 = "DWL";
    linkB3 = "DWS";
    linkB4 = "DWYTorso";
    linkB5 = "TorsoProtections";

    whitelist_R_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA1,linkB1));
    whitelist_R_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA1,linkB2));
    whitelist_R_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA1,linkB3));
    whitelist_R_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA1,linkB4));
    whitelist_R_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA1,linkB5));

    whitelist_R_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA2,linkB1));
    whitelist_R_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA2,linkB2));
    whitelist_R_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA2,linkB3));
    whitelist_R_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA2,linkB4));
    whitelist_R_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA2,linkB5));

    whitelist_R_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA3,linkB1));
    whitelist_R_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA3,linkB2));
    whitelist_R_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA3,linkB3));
    whitelist_R_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA3,linkB4));
    whitelist_R_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA3,linkB5));

    whitelist_R_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA4,linkB1));
    whitelist_R_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA4,linkB2));
    whitelist_R_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA4,linkB3));
    whitelist_R_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA4,linkB4));
    whitelist_R_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA4,linkB5));

    whitelist_R_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA5,linkB1));
    whitelist_R_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA5,linkB2));
    whitelist_R_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA5,linkB3));
    whitelist_R_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA5,linkB4));
    whitelist_R_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA5,linkB5));

    whitelist_R_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA6,linkB1));
    whitelist_R_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA6,linkB2));
    whitelist_R_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA6,linkB3));
    whitelist_R_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA6,linkB4));
    whitelist_R_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA6,linkB5));

    whitelist_R_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA7,linkB1));
    whitelist_R_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA7,linkB2));
    whitelist_R_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA7,linkB3));
    whitelist_R_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA7,linkB4));
    //whitelist_R_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA7,linkB5));

    whitelist_R_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA8,linkB1));
    whitelist_R_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA8,linkB2));
    whitelist_R_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA8,linkB3));
    whitelist_R_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA8,linkB4));
    whitelist_R_Arm_Torso.push_back(std::pair<std::string,std::string>(linkA8,linkB5));

    // used for online prediction of link pair selection

    update(x);

}

SelfCollisionAvoidance::~SelfCollisionAvoidance()
{
    free(x_larm_rarm);
    free(x_larm_torso);

    scale_larm_rarm.close();
    scale_larm_torso.close();
}

double SelfCollisionAvoidance::getLinkPairThreshold()
{
    return _linkPair_threshold;
}



void SelfCollisionAvoidance::setLinkPairThreshold(const double linkPair_threshold)
{
    _linkPair_threshold = std::fabs(linkPair_threshold);
    //this->update();
}



void SelfCollisionAvoidance::update(const yarp::sig::Vector &x)
{
    // we update _Aineq and _bupperBound only if x has changed
    if(!(x == _x_cache)) {
        _x_cache = x;

        this->predict_SCAFoIs( x, Linkpair_updated_list_all, Linkpair_constrained_list_all );
        this->calculate_Aineq_bUpperB( _Aineq, _bUpperBound, Linkpair_constrained_list_all );

    }
//    std::cout << "_Aineq" << _Aineq.toString() << std::endl << std::endl;
//    std::cout << "_bUpperBound" << _bUpperBound.toString() << std::endl << std::endl;
}

bool OpenSoT::constraints::velocity::SelfCollisionAvoidance::setCollisionWhiteList(std::list<LinkPairDistance::LinksPair> whiteList)
{
    bool ok = computeLinksDistance.setCollisionWhiteList(whiteList);
    //this->calculate_Aineq_bUpperB(_Aineq, _bUpperBound);
    return ok;
}

bool OpenSoT::constraints::velocity::SelfCollisionAvoidance::setCollisionBlackList(std::list<LinkPairDistance::LinksPair> blackList)
{
    bool ok = computeLinksDistance.setCollisionBlackList(blackList);
    //this->calculate_Aineq_bUpperB(_Aineq, _bUpperBound);
    return ok;
}

Eigen::MatrixXd SelfCollisionAvoidance::skewSymmetricOperator (const Eigen::Vector3d & r_cp)
{
    MatrixXd J_transform(3,6);
    Matrix3d left_part, right_part;
    left_part = MatrixXd::Identity(3,3);
    right_part << 0, r_cp(2), -r_cp(1),
                   -r_cp(2), 0, r_cp(0),
                   r_cp(1), -r_cp(0), 0;
    J_transform.block(0,0,3,3) = left_part;
    J_transform.block(0,3,3,3) = right_part;

    return J_transform;
}

void SelfCollisionAvoidance::store_l_r_arms(const double & d_current)
{
    d_recent_L_R_Arms[2] = d_recent_L_R_Arms[1];
    d_recent_L_R_Arms[1] = d_recent_L_R_Arms[0];
    d_recent_L_R_Arms[0] = d_current;
}

void SelfCollisionAvoidance::store_larm_torso(const double & d_current)
{
    d_recent_L_Arm_Torso[2] = d_recent_L_Arm_Torso[1];
    d_recent_L_Arm_Torso[1] = d_recent_L_Arm_Torso[0];
    d_recent_L_Arm_Torso[0] = d_current;
}

void SelfCollisionAvoidance::store_rarm_torso(const double & d_current)
{
    d_recent_R_Arm_Torso[2] = d_recent_R_Arm_Torso[1];
    d_recent_R_Arm_Torso[1] = d_recent_R_Arm_Torso[0];
    d_recent_R_Arm_Torso[0] = d_current;
}

void SelfCollisionAvoidance::predict_SCAFoIs( const yarp::sig::Vector & q,
                                              std::list<std::pair<std::string,std::string>> & linkpair_updated_list,
                                              std::list<LinkPairDistance> & linkpair_constrained_list)
{

    linkpair_updated_list.clear();
    linkpair_constrained_list.clear();

    double d_p_lrarms, d_r_lrarms;

    /*//////////////////SCAFoI:whiteList_L_R_Arms////////////////////*/
    /*//////////////////SCAFoI:whiteList_L_R_Arms////////////////////*/

    if ( (d_recent_L_R_Arms[0] != 0.0) && ((d_recent_L_R_Arms[1] != 0.0)) && (d_recent_L_R_Arms[2] != 0.0)  )
    {
        d_p_lrarms = d_recent_L_R_Arms[2] - 3*d_recent_L_R_Arms[1] + 3*d_recent_L_R_Arms[0];

        if ( d_p_lrarms < d_threshold_upper )
        {
            bool ok = computeLinksDistance.setCollisionWhiteList(whiteList_L_R_Arms);
            if(!ok)
            {
                std::cout<<"fail to set whitelist for the SCAFoI of the pair of leftarm and rightarm!" <<std::endl;
            }
            else
            {
                std::list<LinkPairDistance> results = computeLinksDistance.getLinkDistances();

                std::list<LinkPairDistance>::iterator i;
                for (i = results.begin(); i != results.end(); ++i)
                {
                    if ( (*i).getDistance() < d_threshold_lower )
                    {
                        linkpair_constrained_list.push_back(*i);
                        linkpair_updated_list.push_back( (*i).getLinkNames() );
                    }
                    else
                    {
                        linkpair_updated_list.push_back( (*i).getLinkNames() );
                    }
                }

                LinkPairDistance result = results.front();
                d_r_lrarms = result.getDistance();

                if ( d_r_lrarms < d_threshold_upper )
                {
                    store_l_r_arms(d_r_lrarms);
                }
                else
                {
                    store_l_r_arms(0.0);
                }

            }

        }
        else
        {
            store_l_r_arms(0.0);
        }
    }
    else
    {
        for(unsigned int i = 0; i < number_larm; i++)
        {
            x_larm_rarm[i].index = i+1;
            x_larm_rarm[i].value = ( q[robot_col.left_arm.joint_numbers[i]] - min_larm_rarm[i] ) * temp_larm_rarm[i] - 1;
        }
        for(unsigned int j = 0; j < number_rarm; j++)
        {
            x_larm_rarm[number_larm + j].index = number_larm + j + 1;
            x_larm_rarm[number_larm + j].value = ( q[robot_col.right_arm.joint_numbers[j]] - min_larm_rarm[number_larm + j] ) * temp_larm_rarm[number_larm + j] - 1;
        }
        x_larm_rarm[number_larm_rarm].index = -1;

        double predict_label = svm_predict(model_larm_rarm,x_larm_rarm);
        int predict_label_int = (int)predict_label;

        if ( predict_label_int == 1 )
        {
            bool ok = computeLinksDistance.setCollisionWhiteList(whiteList_L_R_Arms);
            if(!ok)
            {
                std::cout<<"fail to set whitelist for the SCAFoI of the pair of leftarm and rightarm!" <<std::endl;
            }
            else
            {
                std::list<LinkPairDistance> results = computeLinksDistance.getLinkDistances();

                std::list<LinkPairDistance>::iterator i;
                for (i = results.begin(); i != results.end(); ++i)
                {
                    if ( (*i).getDistance() < d_threshold_lower )
                    {
                        linkpair_constrained_list.push_back(*i);
                        linkpair_updated_list.push_back( (*i).getLinkNames() );
                    }
                    else
                    {
                        linkpair_updated_list.push_back( (*i).getLinkNames() );
                    }
                }

                LinkPairDistance result = results.front();
                d_r_lrarms = result.getDistance();

                if ( d_r_lrarms < d_threshold_upper )
                {
                    store_l_r_arms(d_r_lrarms);
                }
                else
                {
                    store_l_r_arms(0.0);
                }

            }

        }
        else
        {
            store_l_r_arms(0.0);
        }

    }


    /*//////////////////whiteList_L_Arm_Torso////////////////////*/
    /*//////////////////whiteList_L_Arm_Torso////////////////////*/


    double d_p_larmtorso, d_r_larmtorso;

    if ( (d_recent_L_Arm_Torso[0] != 0.0) && ((d_recent_L_Arm_Torso[1] != 0.0)) && (d_recent_L_Arm_Torso[2] != 0.0)  )
    {
        d_p_larmtorso = d_recent_L_Arm_Torso[2] - 3*d_recent_L_Arm_Torso[1] + 3*d_recent_L_Arm_Torso[0];

        if ( d_p_larmtorso < d_threshold_upper )
        {
            bool ok = computeLinksDistance.setCollisionWhiteList(whitelist_L_Arm_Torso);
            if(!ok)
            {
                std::cout<<"fail to set whitelist for the SCAFoI of the pair of leftarm and torso!" <<std::endl;
            }
            else
            {
                std::list<LinkPairDistance> results = computeLinksDistance.getLinkDistances();

                std::list<LinkPairDistance>::iterator i;
                for (i = results.begin(); i != results.end(); ++i)
                {
                    if ( (*i).getDistance() < d_threshold_lower )
                    {
                        linkpair_constrained_list.push_back(*i);
                        linkpair_updated_list.push_back( (*i).getLinkNames() );
                    }
                    else
                    {
                        linkpair_updated_list.push_back( (*i).getLinkNames() );
                    }
                }

                LinkPairDistance result = results.front();
                d_r_larmtorso = result.getDistance();

                if ( d_r_larmtorso < d_threshold_upper )
                {
                    store_larm_torso(d_r_larmtorso);
                }
                else
                {
                    store_larm_torso(0.0);
                }

            }

        }
        else
        {
            store_larm_torso(0.0);
        }
    }
    else
    {
        for(unsigned int i = 0; i < number_larm; i++)
        {
            x_larm_torso[i].index = i+1;
            x_larm_torso[i].value = ( q[robot_col.left_arm.joint_numbers[i]] - min_larm_torso[i] ) * temp_larm_torso[i] - 1;
        }
        for(unsigned int j = 0; j < number_torso; j++)
        {
            x_larm_torso[number_larm + j].index = number_larm + j + 1;
            x_larm_torso[number_larm + j].value = ( q[robot_col.torso.joint_numbers[j]] - min_larm_torso[number_larm + j] ) * temp_larm_torso[number_larm + j] - 1;
        }
        x_larm_torso[number_larm_torso].index = -1;

        double predict_label = svm_predict(model_larm_torso,x_larm_torso);
        int predict_label_int = (int)predict_label;

        if ( predict_label_int == 1 )
        {
            bool ok = computeLinksDistance.setCollisionWhiteList(whitelist_L_Arm_Torso);
            if(!ok)
            {
                std::cout<<"fail to set whitelist for the SCAFoI of the pair of leftarm and torso!" <<std::endl;
            }
            else
            {
                std::list<LinkPairDistance> results = computeLinksDistance.getLinkDistances();

                std::list<LinkPairDistance>::iterator i;
                for (i = results.begin(); i != results.end(); ++i)
                {
                    if ( (*i).getDistance() < d_threshold_lower )
                    {
                        linkpair_constrained_list.push_back(*i);
                        linkpair_updated_list.push_back( (*i).getLinkNames() );
                    }
                    else
                    {
                        linkpair_updated_list.push_back( (*i).getLinkNames() );
                    }
                }

                LinkPairDistance result = results.front();
                d_r_larmtorso = result.getDistance();

                if ( d_r_larmtorso < d_threshold_upper )
                {
                    store_larm_torso(d_r_larmtorso);
                }
                else
                {
                    store_larm_torso(0.0);
                }
            }

        }
        else
        {
            store_larm_torso(0.0);
        }

    }

    /*//////////////////whiteList_R_Arm_Torso////////////////////*/
    /*//////////////////whiteList_R_Arm_Torso////////////////////*/


    double d_p_rarmtorso, d_r_rarmtorso;

    if ( (d_recent_R_Arm_Torso[0] != 0.0) && ((d_recent_R_Arm_Torso[1] != 0.0)) && (d_recent_R_Arm_Torso[2] != 0.0)  )
    {
        d_p_rarmtorso = d_recent_R_Arm_Torso[2] - 3*d_recent_R_Arm_Torso[1] + 3*d_recent_R_Arm_Torso[0];

        if ( d_p_rarmtorso < d_threshold_upper )
        {
            bool ok = computeLinksDistance.setCollisionWhiteList(whitelist_R_Arm_Torso);
            if(!ok)
            {
                std::cout<<"fail to set whitelist for the SCAFoI of the pair of rightarm and torso!" <<std::endl;
            }
            else
            {
                std::list<LinkPairDistance> results = computeLinksDistance.getLinkDistances();

                std::list<LinkPairDistance>::iterator i;
                for (i = results.begin(); i != results.end(); ++i)
                {
                    if ( (*i).getDistance() < d_threshold_lower )
                    {
                        linkpair_constrained_list.push_back(*i);
                        linkpair_updated_list.push_back( (*i).getLinkNames() );
                    }
                    else
                    {
                        linkpair_updated_list.push_back( (*i).getLinkNames() );
                    }
                }

                LinkPairDistance result = results.front();
                d_r_rarmtorso = result.getDistance();

                if ( d_r_rarmtorso < d_threshold_upper )
                {
                    store_rarm_torso(d_r_rarmtorso);
                }
                else
                {
                    store_rarm_torso(0.0);
                }

            }

        }
        else
        {
            store_rarm_torso(0.0);
        }
    }
    else
    {
        for(unsigned int i = 0; i < number_rarm; i++)
        {
            x_rarm_torso[i].index = i+1;
            x_rarm_torso[i].value = ( q[robot_col.right_arm.joint_numbers[i]] - min_rarm_torso[i] ) * temp_rarm_torso[i] - 1;
        }
        for(unsigned int j = 0; j < number_torso; j++)
        {
            x_rarm_torso[number_rarm + j].index = number_rarm + j + 1;
            x_rarm_torso[number_rarm + j].value = ( q[robot_col.torso.joint_numbers[j]] - min_rarm_torso[number_rarm + j] ) * temp_rarm_torso[number_rarm + j] - 1;
        }
        x_rarm_torso[number_rarm_torso].index = -1;

        double predict_label = svm_predict(model_rarm_torso,x_rarm_torso);
        int predict_label_int = (int)predict_label;

        if ( predict_label_int == 1 )
        {
            bool ok = computeLinksDistance.setCollisionWhiteList(whitelist_R_Arm_Torso);
            if(!ok)
            {
                std::cout<<"fail to set whitelist for the SCAFoI of the pair of rightarm and torso!" <<std::endl;
            }
            else
            {
                std::list<LinkPairDistance> results = computeLinksDistance.getLinkDistances();

                std::list<LinkPairDistance>::iterator i;
                for (i = results.begin(); i != results.end(); ++i)
                {
                    if ( (*i).getDistance() < d_threshold_lower )
                    {
                        linkpair_constrained_list.push_back(*i);
                        linkpair_updated_list.push_back( (*i).getLinkNames() );
                    }
                    else
                    {
                        linkpair_updated_list.push_back( (*i).getLinkNames() );
                    }
                }

                LinkPairDistance result = results.front();
                d_r_rarmtorso = result.getDistance();

                if ( d_r_rarmtorso < d_threshold_upper )
                {
                    store_rarm_torso(d_r_rarmtorso);
                }
                else
                {
                    store_rarm_torso(0.0);
                }
            }

        }
        else
        {
            store_rarm_torso(0.0);
        }

    }



}



void SelfCollisionAvoidance::calculate_Aineq_bUpperB (yarp::sig::Matrix & Aineq_fc,
                                                      yarp::sig::Vector & bUpperB_fc,
                                                      std::list<LinkPairDistance> & interested_LinkPairs)
{

//    robot_col.updateiDyn3Model(x, false);

    MatrixXd Aineq_fc_Eigen(interested_LinkPairs.size(), robot_col.iDyn3_model.getNrOfDOFs());
    VectorXd bUpperB_fc_Eigen(interested_LinkPairs.size());

    double Dm_LinkPair;
    KDL::Frame Link1_T_CP,Link2_T_CP;
    std::string Link1_name, Link2_name;

    int Link1_index, Link2_index;

    KDL::Frame Waist_T_Link1, Waist_T_Link2, Waist_T_Link1_CP, Waist_T_Link2_CP;
    KDL::Vector Link1_origin_kdl, Link2_origin_kdl, Link1_CP_kdl, Link2_CP_kdl;
    Eigen::Matrix<double, 3, 1> Link1_origin, Link2_origin, Link1_CP, Link2_CP;

    Vector3d closepoint_dir;

    yarp::sig::Matrix Link1_CP_Jaco_temp, Link2_CP_Jaco_temp;
    MatrixXd Link1_CP_Jaco, Link2_CP_Jaco;

    yarp::sig::Matrix Waist_frame_world = robot_col.iDyn3_model.getPosition(base_index, true);
    MatrixXd Waist_frame_world_Eigen = from_yarp_to_Eigen_matrix(Waist_frame_world);
    Matrix3d Waist_frame_world_Eigen_Ro = Waist_frame_world_Eigen.block(0,0,3,3);
    MatrixXd temp_trans_matrix(6,6);
    temp_trans_matrix.block(0,0,3,3) = Waist_frame_world_Eigen_Ro;
    temp_trans_matrix.block(3,3,3,3) = Waist_frame_world_Eigen_Ro;
    temp_trans_matrix.block(0,3,3,3) = MatrixXd::Zero(3,3);
    temp_trans_matrix.block(3,0,3,3) = MatrixXd::Zero(3,3);

    int linkPairIndex = 0;
    std::list<LinkPairDistance>::iterator j;
    for (j = interested_LinkPairs.begin(); j != interested_LinkPairs.end(); ++j)
    {

        LinkPairDistance& linkPair(*j);

        Dm_LinkPair = linkPair.getDistance();
        Link1_T_CP = linkPair.getLink_T_closestPoint().first;
        Link2_T_CP = linkPair.getLink_T_closestPoint().second;
        Link1_name = linkPair.getLinkNames().first;
        Link2_name = linkPair.getLinkNames().second;

        Link1_index = robot_col.iDyn3_model.getLinkIndex(Link1_name);
        if(Link1_index == -1)
            std::cout << "Failed to get " << Link1_name << std::endl;
        Link2_index = robot_col.iDyn3_model.getLinkIndex(Link2_name);
        if(Link2_index == -1)
            std::cout << "Failed to get " << Link2_name << std::endl;

        Waist_T_Link1 = robot_col.iDyn3_model.getPositionKDL( base_index , Link1_index );
        Waist_T_Link2 = robot_col.iDyn3_model.getPositionKDL( base_index , Link2_index );
        Waist_T_Link1_CP = Waist_T_Link1 * Link1_T_CP;
        Waist_T_Link2_CP = Waist_T_Link2 * Link2_T_CP;
        Link1_origin_kdl = Waist_T_Link1.p;
        Link2_origin_kdl = Waist_T_Link2.p;
        Link1_CP_kdl = Waist_T_Link1_CP.p;
        Link2_CP_kdl = Waist_T_Link2_CP.p;

        tf::vectorKDLToEigen(Link1_origin_kdl, Link1_origin);
        tf::vectorKDLToEigen(Link2_origin_kdl, Link2_origin);
        tf::vectorKDLToEigen(Link1_CP_kdl, Link1_CP);
        tf::vectorKDLToEigen(Link2_CP_kdl, Link2_CP);


        closepoint_dir = Link2_CP - Link1_CP;
        closepoint_dir = closepoint_dir / Dm_LinkPair;


        robot_col.iDyn3_model.getRelativeJacobian( Link1_index, base_index, Link1_CP_Jaco_temp, true);
        Link1_CP_Jaco = from_yarp_to_Eigen_matrix (Link1_CP_Jaco_temp);
        Link1_CP_Jaco = temp_trans_matrix * Link1_CP_Jaco;
        Link1_CP_Jaco = skewSymmetricOperator(Link1_CP - Link1_origin) * Link1_CP_Jaco;


        robot_col.iDyn3_model.getRelativeJacobian( Link2_index, base_index, Link2_CP_Jaco_temp, true);
        Link2_CP_Jaco = from_yarp_to_Eigen_matrix (Link2_CP_Jaco_temp);
        Link2_CP_Jaco = temp_trans_matrix * Link2_CP_Jaco;
        Link2_CP_Jaco = skewSymmetricOperator(Link2_CP - Link2_origin) * Link2_CP_Jaco;


        Aineq_fc_Eigen.row(linkPairIndex) = closepoint_dir.transpose() * ( Link1_CP_Jaco - Link2_CP_Jaco );
        bUpperB_fc_Eigen(linkPairIndex) = (Dm_LinkPair - _linkPair_threshold) * _boundScaling;

        ++linkPairIndex;

    }

    Aineq_fc = from_Eigen_to_Yarp_matrix(Aineq_fc_Eigen);
    bUpperB_fc = from_Eigen_to_Yarp_vector(bUpperB_fc_Eigen);

}


MatrixXd SelfCollisionAvoidance::from_yarp_to_Eigen_matrix(const yarp::sig::Matrix& Y_M)
{
      int rows = Y_M.rows();
      int columns = Y_M.cols();

      MatrixXd A_M(rows, columns);

      for(int i=0;i<rows;i++)
      {
       for(int j=0;j<columns;j++)
       {

        A_M(i,j) = Y_M(i,j);

       }

      }
      return A_M;
}

yarp::sig::Matrix SelfCollisionAvoidance::from_Eigen_to_Yarp_matrix(const MatrixXd& E_M)
{

      int rows = E_M.rows();
      int columns = E_M.cols();

      yarp::sig::Matrix A_M(rows, columns);

      for(int i=0;i<rows;i++)
      {
       for(int j=0;j<columns;j++)
       {

        A_M(i,j) = E_M(i,j);

       }

      }
      return A_M;

}

VectorXd SelfCollisionAvoidance::from_yarp_to_Eigen_vector(const yarp::sig::Vector& Y_V)
{

      int length = Y_V.length();

      VectorXd A_V(length);

      for(int i=0;i<length;i++)
      {

         A_V(i) = Y_V(i);

      }
      return A_V;

}

yarp::sig::Vector SelfCollisionAvoidance::from_Eigen_to_Yarp_vector(const VectorXd& E_V)
{

      int rows = E_V.rows();

      yarp::sig::Vector A_V(rows);

      for(int i=0;i<rows;i++)
      {

        A_V(i) = E_V(i);

      }
      return A_V;

}

void SelfCollisionAvoidance::setBoundScaling(const double boundScaling)
{
    _boundScaling = boundScaling;
}


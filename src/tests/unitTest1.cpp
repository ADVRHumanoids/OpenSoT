/*
 * Copyright: (C) 2014 Walkman Consortium
 * Authors: Enrico Mingo
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/

#include "sot_VelKinCon_ctrl.h"

#define deg2rad(x) (x*M_PI/180.0)

using namespace yarp::math;

int main()
{
    std::cout<<"************Unit Test 1**********"<<std::endl;

    std::cout<<"\n\n ************* computeW *************"<<std::endl;

    // We consider: 2 joints for each arm and 1 joint for the waist, total 5 joints
    yarp::sig::Vector qMin(5, 0.0);
    qMin[0] = deg2rad(-90.0);
    qMin[1] = deg2rad(0.0);
    qMin[2] = deg2rad(-10.0);
    qMin[3] = deg2rad(0.0);
    qMin[4] = deg2rad(-10.0);
    std::cout<<"qMin: "<<qMin.toString()<<std::endl;

    yarp::sig::Vector qMax(5, 0.0);
    qMax[0] = deg2rad(90.0);
    qMax[1] = deg2rad(110.0);
    qMax[2] = deg2rad(30.0);
    qMax[3] = deg2rad(110.0);
    qMax[4] = deg2rad(30.0);
    std::cout<<"qMax: "<<qMax.toString()<<std::endl;

    yarp::sig::Vector deltaq = qMax - qMin;
    std::cout<<"deltaq: "<<deltaq.toString()<<std::endl;

    std::vector<unsigned int> right_arm_joint_number;
    std::vector<unsigned int> left_arm_joint_number;
    std::vector<unsigned int> torso_arm_joint_number;

    std::cout<<"joint torso: ";
    for(unsigned int i = 0; i < 1; ++i){
        torso_arm_joint_number.push_back(i);
        std::cout<<torso_arm_joint_number[i]<<" ";}
    std::cout<<std::endl;
    std::cout<<"joint left_arm: ";
    for(unsigned int i = 0; i < 2; ++i){
        left_arm_joint_number.push_back(i+1);
        std::cout<<left_arm_joint_number[i]<<" ";}
    std::cout<<std::endl;
    std::cout<<"joint right_arm: ";
    for(unsigned int i = 0; i < 2; ++i){
        right_arm_joint_number.push_back(i+3);
        std::cout<<right_arm_joint_number[i]<<" ";}
    std::cout<<std::endl;


    std::cout<<"diag(W): "<< wb_sot::sot_VelKinCon_ctrl::computeW(qMin, qMax, right_arm_joint_number, left_arm_joint_number,
                                 torso_arm_joint_number).toString()<<std::endl;


    return 0;
}

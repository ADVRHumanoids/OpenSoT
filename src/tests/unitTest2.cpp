/*
 * Copyright: (C) 2014 Walkman Consortium
 * Authors: Enrico Mingo
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/

#include "cartesian_utils.h"

int main()
{
    std::cout<<"************Unit Test 2**********"<<std::endl;

    double x = 1.0;
    double y = 2.0;
    double z = 3.0;

    double Roll = M_PI;
    double Pitch = 0.0;
    double Yaw = 0.0;

    yarp::sig::Matrix M;
    cartesian_utils::homogeneousMatrixFromRPY(M, x, y, z, Roll, Pitch, Yaw);

    double xd = 2.0;
    double yd = 3.0;
    double zd = 4.0;

    double Rolld = 0.0;
    double Pitchd = 0.0;
    double Yawd = 0.0;

    yarp::sig::Matrix Md;
    cartesian_utils::homogeneousMatrixFromRPY(Md, xd, yd, zd, Rolld, Pitchd, Yawd);

    std::cout<<"From:"<<std::endl;
    std::cout<<"x: "<<x<<std::endl;
    std::cout<<"y: "<<y<<std::endl;
    std::cout<<"z: "<<z<<std::endl;
    std::cout<<"Roll: "<<Roll<<std::endl;
    std::cout<<"Pitch: "<<Pitch<<std::endl;
    std::cout<<"Yaw: "<<Yaw<<std::endl;
    std::cout<<"to:"<<std::endl;
    std::cout<<M.toString()<<std::endl;

    std::cout<<std::endl;

    std::cout<<"From:"<<std::endl;
    std::cout<<"xd: "<<xd<<std::endl;
    std::cout<<"yd: "<<yd<<std::endl;
    std::cout<<"zd: "<<zd<<std::endl;
    std::cout<<"Rolld: "<<Rolld<<std::endl;
    std::cout<<"Pitchd: "<<Pitchd<<std::endl;
    std::cout<<"Yawd: "<<Yawd<<std::endl;
    std::cout<<"to:"<<std::endl;
    std::cout<<Md.toString()<<std::endl;

    yarp::sig::Vector error_p(3), error_o(3);
    cartesian_utils::computeCartesianError(M, Md, error_p, error_o);
    std::cout<<"Pos error: "<<error_p.toString()<<std::endl;
    std::cout<<"Orient error: "<<error_o.toString()<<std::endl;

    return 0;
}

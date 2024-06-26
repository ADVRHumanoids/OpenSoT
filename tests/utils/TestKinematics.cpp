#include <gtest/gtest.h>
#include <memory>
#include <xbot2_interface/xbotinterface2.h>

#include "../common.h"



namespace{
class testKinematics: public TestBase
{
protected:

    testKinematics():TestBase("coman_floating_base")
    {

    }

    virtual ~testKinematics() {

    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }

};

void initializeIfNeeded()
{
    static bool is_initialized = false;

    if(!is_initialized) {
        time_t seed = time(NULL);
        seed48((unsigned short*)(&seed));
        srand((unsigned int)(seed));

        is_initialized = true;
    }

}

double getRandomAngle()
{
    initializeIfNeeded();
    return drand48()*2.0*M_PI-M_PI;
}

double getRandomAngle(const double min, const double max)
{
    initializeIfNeeded();
    assert(min <= max);
    if(min < -M_PI || max > M_PI)
        return getRandomAngle();

    return (double)rand()/RAND_MAX * (max-min) + min;
}


Eigen::Matrix3d skew(const Eigen::Vector3d& p)
{
    Eigen::Matrix3d s; s.setZero();
    /* 0 */         s(0,1) = -p[2];     s(0,2) =  p[1];
    s(1,0) =  p[2]; /* 0 */             s(1,2) = -p[0];
    s(2,0) = -p[1]; s(2,1) =  p[0];     /* 0 */

    return s;
}

void EXPECT_MATRIX_NEAR(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const double eps)
{
    EXPECT_EQ(A.rows(), B.rows())<<"A rows: "<<A.rows()<<" B rows: "<<B.rows()<<std::endl;
    EXPECT_EQ(A.cols(), B.cols())<<"A cols: "<<A.cols()<<" B cols: "<<B.cols()<<std::endl;

    if(A.rows() == B.rows() && A.cols() == B.cols())
    {
        for(unsigned int i = 0; i < A.rows(); ++i)
        {
            for(unsigned int j = 0; j < A.cols(); ++j)
            {
                EXPECT_NEAR(A(i,j), B(i,j), eps)<<"A("<<i<<","<<j<<") = "<<A(i,j)<<" B("<<i<<","<<j<<") = "<<B(i,j)<<std::endl;
            }
        }
    }
}

int getRandomInt(const int min, const int max)
{
    initializeIfNeeded();
    int range = max - min + 1;
    return std::rand() % range + min;
}

TEST_F(testKinematics, testFloatingBaseJacobian)
{
    for(unsigned int k = 0; k < 100; ++k)
    {
        Eigen::VectorXd qmin, qmax;
        _model_ptr->getJointLimits(qmin, qmax);

        //std::cout<<"# Joints: "<<_model_ptr->getJointNum()<<std::endl;

        _model_ptr->setJointPosition(_model_ptr->generateRandomQ());
        _model_ptr->update();

        urdf::ModelConstSharedPtr urdf_model = _model_ptr->getUrdf();
        std::vector<urdf::LinkSharedPtr> links;
        urdf_model->getLinks(links);

        std::string base_link =   "Waist";
        std::string distal_link = links[getRandomInt(0, links.size()-1)]->name; //"RSoftHand";
        //std::cout<<"base_link = "<<base_link<<"     distal_link = "<<distal_link<<std::endl;

        Eigen::MatrixXd J; _model_ptr->getJacobian(distal_link, J); //Pose in world of distal_link
        //std::cout<<"J"<<std::endl<<J.block(0,0,6,6)<<std::endl;

        Eigen::MatrixXd Je; _model_ptr->getRelativeJacobian(distal_link, base_link, Je); //Jacobian of distal_link in base_link expressed in base_link
        //std::cout<<"Je: "<<std::endl<<Je<<std::endl;
        Eigen::MatrixXd Jfb; _model_ptr->getJacobian(base_link, base_link, Jfb); //Jacobian of base_link expressed in base_link
        Jfb *= -1;
        //std::cout<<"Jfb: "<<std::endl<<Jfb<<std::endl;

        Eigen::Affine3d wTbl; _model_ptr->getPose(base_link, wTbl); //Pose of base_link in world
        //std::cout<<"wTbl:"<<std::endl<<wTbl.matrix()<<std::endl;
        Eigen::Affine3d blTe; _model_ptr->getPose(distal_link, base_link, blTe); //Pose of distal_link in base_link

        Eigen::Matrix6d Adj1; Adj1.setZero();
        Adj1.block(0,0,3,3) = wTbl.linear();
        Adj1.block(3,3,3,3) = wTbl.linear();
        Eigen::MatrixXd J1  = Adj1*Je;

        Eigen::Vector3d p = blTe.translation();// - wTbl.inverse().translation();
        Eigen::Matrix6d Adj2; Adj2.setIdentity();
        Adj2.block(0,0,3,3) = wTbl.linear();
        Adj2.block(0,3,3,3) = -wTbl.linear()*skew(p);
        Adj2.block(3,3,3,3) = wTbl.linear();
        Eigen::MatrixXd J2 = Adj2*Jfb;

        Eigen::MatrixXd JJ = J1-J2;
        //std::cout<<"JJ"<<std::endl<<JJ.block(0,0,6,6)<<std::endl;

        EXPECT_MATRIX_NEAR(J, JJ, 1e-9);
    }

}

//TEST_F(testKinematics, testRelativeJacobian)
//{
//    for(unsigned int k = 0; k < 100; ++k)
//    {
//        Eigen::VectorXd qmin, qmax;
//        _model_ptr->getJointLimits(qmin, qmax);

//        //std::cout<<"# Joints: "<<_model_ptr->getJointNum()<<std::endl;

//        _model_ptr->setJointPosition(getRandomAngles(qmin, qmax, qmin.size()));
//        _model_ptr->update();

//        urdf::ModelInterface urdf_model = _model_ptr->getUrdf();
//        std::vector<urdf::LinkSharedPtr> links;
//        urdf_model.getLinks(links);

//        std::string base_link =   links[getRandomInt(0, links.size()-1)]->name; //"LSoftHand";
//        std::string distal_link = links[getRandomInt(0, links.size()-1)]->name; //"RSoftHand";
//        //std::cout<<"base_link = "<<base_link<<"     distal_link = "<<distal_link<<std::endl;

//        Eigen::MatrixXd J; _model_ptr->getRelativeJacobian(distal_link, base_link, J); //Jacobian of distal_link in base_link expressed in base_link

//        //std::cout<<"Jacobian of "<<distal_link<<" wrt "<<base_link<<" expressed in "<<base_link<<": "<<std::endl<<J<<std::endl;


//        Eigen::Affine3d wTb; _model_ptr->getPose(base_link, wTb); //Pose of base_link in world
//        Eigen::Affine3d wTd; _model_ptr->getPose(distal_link, wTd); //Pose of distal_link in world

//        Eigen::MatrixXd Jdistal; _model_ptr->getJacobian(distal_link, Jdistal); //Jacobian of distal_link expressed in world
//        Eigen::MatrixXd Jbase; _model_ptr->getJacobian(base_link, Jbase); //Jacobian of base_link expressed in world

//        Eigen::Matrix6d Adj1; Adj1.setZero();
//        Adj1.block(0,0,3,3) = wTb.linear().transpose();
//        Adj1.block(3,3,3,3) = wTb.linear().transpose();
//        Eigen::MatrixXd J1 = Adj1*Jdistal; //Jacobian of distal_link expressed in base_link
//        //std::cout<<"J1: "<<std::endl<<J1<<std::endl;

//        Eigen::Vector3d p = wTd.translation() - wTb.translation(); //relative position between distal_link and base_link expressed in world

//        Eigen::Matrix6d Adj2; Adj2.setZero();
//        Adj2.block(0,0,3,3) = wTb.linear().transpose();
//        Adj2.block(0,3,3,3) = -wTb.linear().transpose()*skew(p);
//        Adj2.block(3,3,3,3) = wTb.linear().transpose();
//        //std::cout<<"Adj2: "<<std::endl<<Adj2<<std::endl;
//        Eigen::MatrixXd J2 = Adj2*Jbase; //Jacobian of base_link expressed in base_link computed at distal_link
//        //std::cout<<"J2: "<<std::endl<<J2<<std::endl;

//        Eigen::MatrixXd JR = J1 - J2;
//        //std::cout<<"Computed Jacobian of "<<distal_link<<" wrt "<<base_link<<" expressed in "<<base_link<<": "<<std::endl<<JR<<std::endl;

//        EXPECT_MATRIX_NEAR(J, JR, 1e-9);
//    }
//}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

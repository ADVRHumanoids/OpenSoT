#include <OpenSoT/tasks/acceleration/CoM.h>
#include <OpenSoT/constraints/acceleration/DynamicFeasibility.h>
#include <OpenSoT/utils/InverseDynamics.h>
#include <gtest/gtest.h>
#include <boost/make_shared.hpp>
#include <OpenSoT/utils/cartesian_utils.h>

std::string robotology_root = std::getenv("ROBOTOLOGY_ROOT");
std::string relative_path = "/external/OpenSoT/tests/configs/coman/configs/config_coman_floating_base.yaml";
std::string _path_to_cfg = robotology_root + relative_path;

namespace{
class testID: public ::testing::Test
{
public:
    testID(){
        _model_ptr = XBot::ModelInterface::getModel(_path_to_cfg);

        if(_model_ptr)
            std::cout<<"pointer address: "<<_model_ptr.get()<<std::endl;
        else
            std::cout<<"pointer is NULL "<<_model_ptr.get()<<std::endl;

        std::cout<<"_model_ptr->getJointNum(): "<<_model_ptr->getJointNum()<<std::endl;

        _q = setGoodInitialPosition(_model_ptr);
        _qdot.setZero(_q.size());

        _model_ptr->setJointPosition(_q);
        _model_ptr->setJointVelocity(_qdot);
        _model_ptr->setFloatingBasePose(Eigen::Affine3d::Identity());

        _model_ptr->update();


    }

    virtual ~testID() {

    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }

    XBot::ModelInterface::Ptr _model_ptr;

    Eigen::VectorXd _q, _qdot;

private:

    Eigen::VectorXd setGoodInitialPosition(XBot::ModelInterface::Ptr _model_ptr) {
        Eigen::VectorXd _q(_model_ptr->getJointNum());
        _q.setZero();
        _q[_model_ptr->getDofIndex("RHipSag")] = -25.0*M_PI/180.0;
        _q[_model_ptr->getDofIndex("RKneeSag")] = 50.0*M_PI/180.0;
        _q[_model_ptr->getDofIndex("RAnkSag")] = -25.0*M_PI/180.0;

        _q[_model_ptr->getDofIndex("LHipSag")] = -25.0*M_PI/180.0;
        _q[_model_ptr->getDofIndex("LKneeSag")] = 50.0*M_PI/180.0;
        _q[_model_ptr->getDofIndex("LAnkSag")] = -25.0*M_PI/180.0;

        _q[_model_ptr->getDofIndex("LShSag")] =  20.0*M_PI/180.0;
        _q[_model_ptr->getDofIndex("LShLat")] = 20.0*M_PI/180.0;
        _q[_model_ptr->getDofIndex("LShYaw")] = -15.0*M_PI/180.0;
        _q[_model_ptr->getDofIndex("LElbj")] = -80.0*M_PI/180.0;

        _q[_model_ptr->getDofIndex("RShSag")] =  20.0*M_PI/180.0;
        _q[_model_ptr->getDofIndex("RShLat")] = -20.0*M_PI/180.0;
        _q[_model_ptr->getDofIndex("RShYaw")] = 15.0*M_PI/180.0;
        _q[_model_ptr->getDofIndex("RElbj")] = -80.0*M_PI/180.0;
        return _q;
    }
};

inline void MATRIX_ARE_NEAR(const Eigen::MatrixXd& m0,
                    const Eigen::MatrixXd& m1,
                    const double eps = 1e-9)
{
    bool sizeAreCompatible = (m0.rows() == m1.rows() &&
                              m0.cols() == m1.cols());
    EXPECT_TRUE(sizeAreCompatible) << "Size of compared matrices "
                                   << "are not compatible";

    for(unsigned int r = 0; r < m0.rows(); ++r)
        for(unsigned int c = 0; c < m0.cols(); ++c) {
            EXPECT_NEAR(m0(r,c), m1(r,c), eps) << "Elements in ("
                                               << r << "," << c
                                               << ") are not equal";

        }
}

inline void VECTOR_ARE_NEAR(const Eigen::VectorXd& m0,
                     const Eigen::VectorXd& m1,
                    const double eps = 1e-9)
{
    bool sizeAreCompatible = (m0.size() == m1.size());
    EXPECT_TRUE(sizeAreCompatible) << "Size of compared vectors "
                                   << "are not compatible";

    for(unsigned int c = 0; c < m0.size(); ++c) {
            EXPECT_NEAR(m0(c), m1(c), eps) << "Elements in ("
                                               << c
                                               << ") are not equal";

        }

}



TEST_F(testID, checkCentroidalLinearMomentum)
{
    srand((unsigned int) time(0));

    _q.setRandom();
    _model_ptr->setJointPosition(_q);
    _model_ptr->update();


    Eigen::MatrixXd B;
    _model_ptr->getInertiaMatrix(B);

    double m = _model_ptr->getMass();
    std::cout<<"m = "<<m<<std::endl;

    Eigen::MatrixXd Jcom;
    Eigen::Vector3d Jdotqdot_com;
    _model_ptr->getCOMJacobian(Jcom, Jdotqdot_com);
    std::cout<<"Jcom_fb = \n"<<Jcom.block(0,0,3,6)<<std::endl;

    Eigen::Matrix6d M = B.block(0,0,6,6);
    std::cout<<"M = \n"<<M<<std::endl;


    std::cout<<"m*Jcom_fb = \n"<<m*Jcom.block(0,0,3,6)<<std::endl;

    MATRIX_ARE_NEAR(m*Jcom.block(0,0,3,6), M.block(0,0,3,6), 1e-8);

    /** Next line tests that the first three rows of the floating-base inertia
     *  are equal to m*Jcom
    **/
    MATRIX_ARE_NEAR(m*Jcom, B.block(0,0,3,_q.size()), 1e-8);

    _qdot.setRandom();
    _model_ptr->setJointVelocity(_qdot);
    _model_ptr->update();
    Eigen::VectorXd h;
    Eigen::Vector3d g; g<<0.0,0.0,9.81;
    _model_ptr->computeNonlinearTerm(h);
    //_model_ptr->computeGravityCompensation(g);
    _model_ptr->getCOMJacobian(Jcom, Jdotqdot_com);


    std::cout<<"h_fb: "<<h.head(3).transpose()<<std::endl;
    std::cout<<"G_fb: "<<m*g.transpose()<<std::endl;
    std::cout<<"m*Jdotqdot_com + G: "<<(m*Jdotqdot_com+m*g).transpose() <<std::endl;

    /** Next line tests that the first three rows of the floating-base non-linear
     * terms are equal to m*(Jdotqdot_com + g)
     */
    VECTOR_ARE_NEAR(h.head(3), m*Jdotqdot_com+m*g, 1e-8);
}

Eigen::Matrix6d Adj(const Eigen::Affine3d& T)
{
    Eigen::Matrix6d X; X.setZero();

    X.block(0,0,3,3) = T.linear();
    X.block(3,3,3,3) = T.linear();
    Eigen::Matrix3d s;
    cartesian_utils::skew(T.translation(),s);
    X.block(0,3,3,3) = s*T.linear();

    return X;
}


/** Computation taken from
 * "Improved Computation of the Humanoid Centroidal Dynamics and Application for Whole-Body Control", by
 *      Patrick M. Wensing and David E. Orin
 **/
TEST_F(testID, checkCentroidalAngularMomentum)
{
    srand((unsigned int) time(0));

    _q.setRandom();
    _model_ptr->setJointPosition(_q);
    _qdot.setRandom();
    _model_ptr->setJointVelocity(_qdot);
    _model_ptr->update();



    Eigen::Vector3d com;
    _model_ptr->getCOM(com);

    Eigen::Affine3d w_T_fb;
    _model_ptr->getFloatingBasePose(w_T_fb);

    Eigen::Affine3d T; T.setIdentity();
    T.linear() = w_T_fb.linear().inverse();
    com = w_T_fb.inverse() * com;
    T.translation() = com;







    Eigen::MatrixXd J_, J,Jcom;
    std::string floating_base;
    _model_ptr->getFloatingBaseLink(floating_base);
    _model_ptr->getJacobian(floating_base,com, Jcom);
    _model_ptr->getJacobian(floating_base,floating_base, J_);
    J = J_.block(0,0,6,6);
    std::cout<<"J: \n"<<J<<std::endl;

    std::cout<<"Jcom: \n"<<Jcom.block(0,0,6,6)<<std::endl;


    Eigen::MatrixXd H;
    _model_ptr->getInertiaMatrix(H);

    Eigen::Matrix6d X = Adj(T);
    std::cout<<"X: \n"<<X<<std::endl;
    Eigen::MatrixXd A = X.transpose()*J.transpose().inverse()*H.block(0,0,6,_model_ptr->getJointNum());

    std::cout<<"A_fb: \n"<<A.block(0,0,6,6)<<std::endl;

    Eigen::MatrixXd AA;
    Eigen::Vector6d AADot_qdot;
    _model_ptr->getCentroidalMomentumMatrix(AA, AADot_qdot);
    std::cout<<"AA_fb: \n"<<AA.block(0,0,6,6)<<std::endl;

    MATRIX_ARE_NEAR(AA, A);


    Eigen::VectorXd h, g;
    _model_ptr->computeNonlinearTerm(h);
    _model_ptr->computeGravityCompensation(g);
    Eigen::MatrixXd ADot_qdot = X.transpose()*J.transpose().inverse()*(h.segment(0,6) - g.segment(0,6));
    std::cout<<"ADot_qdot: "<<ADot_qdot.transpose()<<std::endl;
    std::cout<<"AADot_qdot: "<<AADot_qdot.transpose()<<std::endl;

    VECTOR_ARE_NEAR(ADot_qdot, AADot_qdot);
}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

// #include <pinocchio/multibody/model.hpp>
// #include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <OpenSoT/oc/SE3Task.h>
#include <Eigen/Dense>

#include <gtest/gtest.h>
#include <chrono>
#include <fstream>


std::string ReadFile(std::string path)
{
    std::ifstream t(path);
    std::stringstream buffer;
    buffer << t.rdbuf();
    return buffer.str();
}

namespace {

class testSE3Task: public ::testing::Test
{
    protected:
        testSE3Task()
        {

        }

        virtual ~testSE3Task()
        {

        }

        virtual void SetUp()
        {

        }

        virtual void TearDown()
        {

        }
};

double randomDouble(double min, double max) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist(min, max);
    return dist(gen);
}

Eigen::Quaterniond randomQuaternion() {
    double u1 = randomDouble(0.0, 1.0);
    double u2 = randomDouble(0.0, 2.0 * M_PI);
    double u3 = randomDouble(0.0, 2.0 * M_PI);

    double sqrt1MinusU1 = std::sqrt(1 - u1);
    double sqrtU1 = std::sqrt(u1);

    double w = sqrt1MinusU1 * std::sin(u2);
    double x = sqrt1MinusU1 * std::cos(u2);
    double y = sqrtU1 * std::sin(u3);
    double z = sqrtU1 * std::cos(u3);

    return Eigen::Quaterniond(w, x, y, z).normalized();
}

Eigen::VectorXd generateRandomPose(double min, double max) {
    Eigen::VectorXd pose(7);
    pose.setZero();

    // Random position
    for (int i = 0; i < 3; ++i) {
        pose[i] = randomDouble(min, max);
    }

    // Random orientation (convert quaternion to Euler angles)
    Eigen::Quaterniond q = randomQuaternion();
    
    pose[3] = q.x();
    pose[4] = q.y();
    pose[5] = q.z();
    pose[6] = q.w();

    return pose;
}

Eigen::VectorXd generateRandomConfig(const Eigen::VectorXd& qmin, const Eigen::VectorXd& qmax)
{
    Eigen::VectorXd q(qmin.size());
    q.setZero();
    for(unsigned int i = 0; i < qmin.size(); ++i)
    {
        q[i] = randomDouble(qmin[i], qmax[i]);
    }
    return q;
}

TEST_F(testSE3Task, testJacobianFloatingFrame)
{
    std::string path_to_urdf = OPENSOT_TEST_PATH;
    path_to_urdf += "/robots/floating_frame/floating_frame.urdf";
    std::string frame_name = "base_link";


    pinocchio::Model model;
    pinocchio::urdf::buildModel(path_to_urdf, model);
    pinocchio::Data data(model);

    XBot::ModelInterface::Ptr _robot = XBot::ModelInterface::getModel(ReadFile(path_to_urdf), OPENSOT_TEST_MODEL_TYPE);
    
    Eigen::VectorXd q = pinocchio::neutral(model);

    _robot->setJointPosition(q);
    _robot->update();

    std::vector<std::pair<std::string, int>> var_list;
    
    var_list.emplace_back("dq", _robot->getNv());
    
    OpenSoT::OptvarHelper var(var_list);

    OpenSoT::oc::SE3Task::Ptr SE3T = std::make_shared<OpenSoT::oc::SE3Task>(OpenSoT::oc::SE3Task("SE3T", *_robot, var.getVariable("dq"), frame_name));
    SE3T->update();

    Eigen::MatrixXd J_pin(6, model.nv);
    pinocchio::FrameIndex frame_id = model.getFrameId(frame_name);
    for(unsigned int i = 0; i < 1000; ++i)
    {

        pinocchio::forwardKinematics(model, data, q);
        pinocchio::updateFramePlacements(model, data);
        pinocchio::computeFrameJacobian(model, data, q, frame_id, pinocchio::LOCAL, J_pin);

        _robot->setJointPosition(q);
        _robot->update();

        SE3T->update();


        ASSERT_EQ(J_pin.rows(), SE3T->getA().rows()) << "Jacobian row counts differ";
        ASSERT_EQ(J_pin.cols(), SE3T->getA().cols()) << "Jacobian column counts differ";


        // Use approximate equality for floating point comparison
        double tolerance = 1e-10;
        ASSERT_TRUE(J_pin.isApprox(SE3T->getA(), tolerance)) 
            << "Jacobians differ beyond tolerance " << tolerance
            << "\nPinocchio Jacobian:\n" << J_pin
            << "\nOpenSoT Jacobian:\n" << SE3T->getA()
            << "\nDifference:\n" << (J_pin - SE3T->getA());

        q = generateRandomPose(-3., 3.);
    }
}

TEST_F(testSE3Task, testJacobianManipulatorEndEffector)
{
    std::string path_to_urdf = OPENSOT_TEST_PATH;
    path_to_urdf += "/robots/panda/panda.urdf";
    std::string frame_name = "fp3_link8";


    pinocchio::Model model;
    pinocchio::urdf::buildModel(path_to_urdf, model);
    pinocchio::Data data(model);

    XBot::ModelInterface::Ptr _robot = XBot::ModelInterface::getModel(ReadFile(path_to_urdf), OPENSOT_TEST_MODEL_TYPE);
    
    Eigen::VectorXd q = pinocchio::neutral(model);

    _robot->setJointPosition(q);
    _robot->update();

    std::vector<std::pair<std::string, int>> var_list;
    
    var_list.emplace_back("dq", _robot->getNv());
    
    OpenSoT::OptvarHelper var(var_list);

    OpenSoT::oc::SE3Task::Ptr SE3T = std::make_shared<OpenSoT::oc::SE3Task>(OpenSoT::oc::SE3Task("SE3T", *_robot, var.getVariable("dq"), frame_name));
    SE3T->update();

    Eigen::MatrixXd J_pin(6, model.nv);
    pinocchio::FrameIndex frame_id = model.getFrameId(frame_name);
    Eigen::VectorXd qmin, qmax;
    _robot->getJointLimits(qmin, qmax);
    for(unsigned int i = 0; i < 1000; ++i)
    {

        pinocchio::forwardKinematics(model, data, q);
        pinocchio::updateFramePlacements(model, data);
        pinocchio::computeFrameJacobian(model, data, q, frame_id, pinocchio::LOCAL, J_pin);

        _robot->setJointPosition(q);
        _robot->update();

        SE3T->update();


        ASSERT_EQ(J_pin.rows(), SE3T->getA().rows()) << "Jacobian row counts differ";
        ASSERT_EQ(J_pin.cols(), SE3T->getA().cols()) << "Jacobian column counts differ";


        // Use approximate equality for floating point comparison
        double tolerance = 1e-10;
        ASSERT_TRUE(J_pin.isApprox(SE3T->getA(), tolerance)) 
            << "Jacobians differ beyond tolerance " << tolerance
            << "\nPinocchio Jacobian:\n" << J_pin
            << "\nOpenSoT Jacobian:\n" << SE3T->getA()
            << "\nDifference:\n" << (J_pin - SE3T->getA());

        q = generateRandomConfig(qmin, qmax);
    }
}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
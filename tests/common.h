#ifndef COMMON_H
#define COMMON_H

#include <gtest/gtest.h>
#include <xbot2_interface/xbotinterface2.h>
#include <fstream>

std::string ReadFile(std::string path)
{
    std::ifstream t(path);
    std::stringstream buffer;
    buffer << t.rdbuf();
    return buffer.str();
}

XBot::ModelInterface::Ptr GetTestModel(std::string name)
{
    std::string robot_folder = OPENSOT_TEST_PATH;
    robot_folder += "/robots/" + name;

    return XBot::ModelInterface::getModel(
        ReadFile(robot_folder + "/" + name + ".urdf"),
        ReadFile(robot_folder + "/" + name + ".srdf"),
        OPENSOT_TEST_MODEL_TYPE);
}

struct TestBase : ::testing::Test
{
    XBot::ModelInterface::Ptr _model_ptr;
    std::string _robot_name;

    TestBase(std::string robot_name):
        _robot_name(robot_name),
        _model_ptr(GetTestModel(robot_name))
    {
        std::cout << "model '" << _model_ptr->getName() <<
            "' nq = " << _model_ptr->getNq() << " nv = " << _model_ptr->getNv() <<
            std::endl;
    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }
};

#endif // COMMON_H

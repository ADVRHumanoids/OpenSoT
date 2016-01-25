#include <idynutils/idynutils.h>
#include <idynutils/tests_utils.h>
#include <OpenSoT/utils/logger/L.h>
#include <OpenSoT/utils/DefaultHumanoidStack.h>
#include <gtest/gtest.h>

#define dT 1.0e-2

using namespace yarp::math;

namespace OpenSoT {

class testLogger: public ::testing::Test
{
public:

protected:

    iDynUtils _robot;

    testLogger()
        :        _robot("coman",
                        std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.urdf",
                        std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.srdf")
    {
    }

    virtual ~testLogger() {
    }

    virtual void SetUp() {
    }

    virtual void TearDown() {
    }

};

TEST_F(testLogger, testDataFlusherWorks)
{
    yarp::sig::Vector v(3,0.0);
    yarp::sig::Vector dq(_robot.iDyn3_model.getNrOfDOFs(), 0.0);
    double t = 0.0;
    OpenSoT::flushers::Flusher::Ptr dataFlusher;

    L logger("test_dfw", _robot);
    logger.open("test_dfw_data1");
    dataFlusher = logger.add(v.data(),3);

    logger.update(t, dq);

    v[0] = 1.0;
    dq[3] = 2.0;
    t+=dT;

    logger.update(t, dq);

    v[0] = 0.0;
    dq[3] = 0.0;

    v[1] = 1.0;
    dq[4] = 2.0;
    t+=dT;

    logger.update(t, dq);

    v[1] = 0.0;
    dq[4] = 0.0;

    v[2] = 1.0;
    dq[5] = 2.0;
    t+=dT;

    logger.update(t, dq);
    logger.close();  // the final file should have 4 rows of data, 33 columns
}

TEST_F(testLogger, testDynamicsFlusherWork)
{
    yarp::sig::Vector q(_robot.iDyn3_model.getNrOfDOFs(), 0.0);
    yarp::sig::Vector dq = q*0.0;
    _robot.switchAnchorAndFloatingBase(_robot.left_leg.end_effector_name);
    _robot.updateiDyn3Model(q, true);
    OpenSoT::DefaultHumanoidStack DHS(_robot, dT, q);

    double t = 0.0;
    OpenSoT::flushers::Flusher::Ptr dynamicsFlusher;

    L logger("test_dyfw", _robot);
    logger.open("test_dyfw_data1");
    dynamicsFlusher = logger.add(DHS.torqueLimits);

    for(unsigned int j = 0; j < 100; ++j)
    {
        for(unsigned int i = 0; i < logger.model.right_arm.joint_numbers.size(); ++i)
            dq[logger.model.right_arm.joint_numbers[i]] += 0.0001;

        _robot.updateiDyn3Model(q, dq/dT, true);
        DHS.torqueLimits->update(q);
        t+=dT;
        logger.update(t, dq);
    }

    logger.close();  // the final file should have 4 rows of data, 33 columns
}

TEST_F(testLogger, testLoggerCollationWorks)
{
    yarp::sig::Vector v(3,0.0);
    yarp::sig::Vector dq(_robot.iDyn3_model.getNrOfDOFs(), 0.0);
    double t = 0.0;
    OpenSoT::flushers::Flusher::Ptr dataFlusher;

    L logger("test_cw", _robot);
    logger.open("test_cw_data1");
    dataFlusher = logger.add(v.data(),3);

    logger.update(t, dq);

    v[0] = 1.0;
    dq[3] = 2.0;
    t+=dT;

    logger.update(t, dq);

    logger.close(); // the first file should have 2 rows of data, 33 columns
    logger.open("test_cw_data2");
    dataFlusher = logger.add(v.data(),3);

    v[0] = 0.0;
    dq[3] = 0.0;

    v[1] = 1.0;
    dq[4] = 2.0;
    t+=dT;

    logger.update(t, dq);

    v[1] = 0.0;
    dq[4] = 0.0;

    v[2] = 1.0;
    dq[5] = 2.0;
    t+=dT;

    logger.update(t, dq);
    logger.close();  // the second file should have 2 rows of data, 33 columns
}

TEST_F(testLogger, testAppendingWorks)
{
    yarp::sig::Vector v(3,0.0);
    yarp::sig::Vector dq(_robot.iDyn3_model.getNrOfDOFs(), 0.0);
    double t = 0.0;
    OpenSoT::flushers::Flusher::Ptr dataFlusher;

    L logger("test_aw", _robot);
    logger.open("test_aw_data1");
    dataFlusher = logger.add(v.data(),3);

    logger.update(t, dq);

    v[0] = 1.0;
    dq[3] = 2.0;
    t+=dT;

    logger.update(t, dq);

    logger.close();
    logger.open("test_aw_data2");
    dataFlusher = logger.add(v.data(),3);

    v[0] = 0.0;
    dq[3] = 0.0;

    v[1] = 1.0;
    dq[4] = 2.0;
    t+=dT;

    logger.update(t, dq);

    v[1] = 0.0;
    dq[4] = 0.0;

    v[2] = 1.0;
    dq[5] = 2.0;
    t+=dT;

    logger.update(t, dq);
    logger.close();  // the second file should have 2 rows of data, 33 columns

    logger.open("test_aw_data1");
    dataFlusher = logger.add(v.data(),3);

    v[2] = 0.0;
    dq[5] = 0.0;

    v[1] = 1.0;
    dq[4] = 2.0;
    t+=dT;

    logger.update(t, dq);

    v[1] = 0.0;
    dq[4] = 0.0;

    v[2] = 1.0;
    dq[5] = 2.0;
    t+=dT;

    logger.update(t, dq);
    logger.close();  // the first file should have now 2 rows of data, 33 columns
}

TEST_F(testLogger, testPlotterWorks)
{
    yarp::sig::Vector v(3,0.0);
    yarp::sig::Vector dq(_robot.iDyn3_model.getNrOfDOFs(), 0.0);
    double t = 0.0;
    OpenSoT::flushers::Flusher::Ptr dataFlusher;

    L logger("test_pw", _robot);
    logger.open("test_pw_data1");
    dataFlusher = logger.add(v.data(),3);

    logger.update(t, dq);

    v[0] = 1.0;
    dq[3] = 2.0;
    t+=dT;

    logger.update(t, dq);

    v[0] = 0.0;
    dq[3] = 0.0;

    v[1] = 1.0;
    dq[4] = 2.0;
    t+=dT;

    logger.update(t, dq);

    v[1] = 0.0;
    dq[4] = 0.0;

    v[2] = 1.0;
    dq[5] = 2.0;
    t+=dT;

    logger.update(t, dq);

    OpenSoT::plotters::Plottable dataPlottable =
        dataFlusher->i(OpenSoT::flushers::DataFlusher<double>::ALL);
    logger.plotter->figure(10.24,7.68,"estimated torques for torso vs real torques");
    logger.plotter->subplot(2,2,1);
    logger.plotter->plot_t(dataPlottable);
    logger.plotter->title("Data Flusher");
    logger.plotter->autoLegend(dataPlottable);
    logger.plotter->xlabel("t [s]");
    logger.plotter->ylabel("data");

    logger.plotter->subplot(2,2,2);
    logger.plotter->plot_t(dataPlottable);
    logger.plotter->title("Data Flusher (with labels)");
    std::vector<std::string> tau_description;
    tau_description.push_back("tau 0");
    tau_description.push_back("tau 1");
    tau_description.push_back("tau 2");
    dataPlottable.first->setDescription(tau_description);
    logger.plotter->autoLegend(dataPlottable);
    logger.plotter->xlabel("t [s]");
    logger.plotter->ylabel("data");

    OpenSoT::plotters::Plottable solutionPlottable =
        logger.dq_opt();
    logger.plotter->subplot(2,2,3);
    logger.plotter->plot_t(solutionPlottable);
    logger.plotter->title("Solution");
    logger.plotter->autoLegend(solutionPlottable);
    logger.plotter->xlabel("t [s]");
    logger.plotter->ylabel("solution [rad/s]");

    OpenSoT::Indices allDqIndices = solutionPlottable.second;
    std::vector<unsigned int> allDqIndicesV = allDqIndices.asVector();
    std::vector<unsigned int> j3_5Indicesv;
    for(unsigned int i = 3; i <= 5; ++i)
        j3_5Indicesv.push_back(allDqIndicesV[i]);
    solutionPlottable.second = OpenSoT::Indices(j3_5Indicesv);
    logger.plotter->subplot(2,2,4);
    logger.plotter->plot_t(solutionPlottable);
    logger.plotter->title("Solution (joints 3-5)");
    logger.plotter->autoLegend(solutionPlottable);
    logger.plotter->xlabel("t [s]");
    logger.plotter->ylabel("solution [rad/s]");
    logger.plotter->savefig();
    logger.plotter->show();

    logger.close();  // the final file should have 4 rows of data, 33 columns
}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

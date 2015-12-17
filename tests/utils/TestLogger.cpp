#include <idynutils/idynutils.h>
#include <idynutils/tests_utils.h>
#include <OpenSoT/utils/logger/L.h>
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

    logger.udpate(t, dq);

    v[0] = 1.0;
    dq[3] = 2.0;

    logger.udpate(t, dq);

    v[0] = 0.0;
    dq[3] = 0.0;

    v[1] = 1.0;
    dq[4] = 2.0;

    logger.udpate(t, dq);

    v[1] = 0.0;
    dq[4] = 0.0;

    v[2] = 1.0;
    dq[5] = 2.0;

    logger.udpate(t, dq);
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

    logger.udpate(t, dq);

    v[0] = 1.0;
    dq[3] = 2.0;

    logger.udpate(t, dq);

    logger.close(); // the first file should have 2 rows of data, 33 columns
    logger.open("test_cw_data2");
    dataFlusher = logger.add(v.data(),3);

    v[0] = 0.0;
    dq[3] = 0.0;

    v[1] = 1.0;
    dq[4] = 2.0;

    logger.udpate(t, dq);

    v[1] = 0.0;
    dq[4] = 0.0;

    v[2] = 1.0;
    dq[5] = 2.0;

    logger.udpate(t, dq);
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

    logger.udpate(t, dq);

    v[0] = 1.0;
    dq[3] = 2.0;

    logger.udpate(t, dq);

    logger.close();
    logger.open("test_aw_data2");
    dataFlusher = logger.add(v.data(),3);

    v[0] = 0.0;
    dq[3] = 0.0;

    v[1] = 1.0;
    dq[4] = 2.0;

    logger.udpate(t, dq);

    v[1] = 0.0;
    dq[4] = 0.0;

    v[2] = 1.0;
    dq[5] = 2.0;

    logger.udpate(t, dq);
    logger.close();  // the second file should have 2 rows of data, 33 columns

    logger.open("test_aw_data1");
    dataFlusher = logger.add(v.data(),3);

    v[2] = 0.0;
    dq[5] = 0.0;

    v[1] = 1.0;
    dq[4] = 2.0;

    logger.udpate(t, dq);

    v[1] = 0.0;
    dq[4] = 0.0;

    v[2] = 1.0;
    dq[5] = 2.0;

    logger.udpate(t, dq);
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

    logger.udpate(t, dq);

    v[0] = 1.0;
    dq[3] = 2.0;

    logger.udpate(t, dq);

    v[0] = 0.0;
    dq[3] = 0.0;

    v[1] = 1.0;
    dq[4] = 2.0;

    logger.udpate(t, dq);

    v[1] = 0.0;
    dq[4] = 0.0;

    v[2] = 1.0;
    dq[5] = 2.0;

    logger.udpate(t, dq);

    OpenSoT::plotters::Plottable dataPlottable =
        dataFlusher->i(OpenSoT::flushers::DataFlusher<double>::ALL);
    logger.plotter->figure(10.24,7.68,"estimated torques for torso vs real torques");
    logger.plotter->subplot(1,3,1);
    logger.plotter->plot_t(dataPlottable);
    logger.plotter->title("Data Flusher");
    logger.plotter->autoLegend(dataPlottable);
    logger.plotter->xlabel("t [s]");
    logger.plotter->ylabel("data");
    logger.plotter->savefig();

    OpenSoT::plotters::Plottable solutionPlottable =
        logger.dq_opt();
    logger.plotter->subplot(1,3,2);
    logger.plotter->plot_t(solutionPlottable);
    logger.plotter->title("Solution");
    logger.plotter->autoLegend(solutionPlottable);
    logger.plotter->xlabel("t [s]");
    logger.plotter->ylabel("solution [rad/s]");
    logger.plotter->savefig();

    OpenSoT::Indices allDqIndices = solutionPlottable.second;
    std::vector<unsigned int> allDqIndicesV = allDqIndices.getRowsVector();
    std::vector<unsigned int> j3_5Indicesv;
    for(unsigned int i = 3; i <= 5; ++i)
        j3_5Indicesv.push_back(allDqIndicesV[i]);
    solutionPlottable.second = OpenSoT::Indices(j3_5Indicesv);
    logger.plotter->subplot(1,3,3);
    logger.plotter->plot_t(solutionPlottable);
    logger.plotter->title("Solution (joints 3-5");
    logger.plotter->autoLegend(solutionPlottable);
    logger.plotter->xlabel("t [s]");
    logger.plotter->ylabel("solution [rad/s]");
    logger.plotter->savefig();

    logger.close();  // the final file should have 4 rows of data, 33 columns
}

TEST_F(testLogger, testDynamicsFlusherWork)
{
}


}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

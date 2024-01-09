#include <gtest/gtest.h>
#include <OpenSoT/Task.h>
#include <OpenSoT/tasks/velocity/Postural.h>
#include <chrono>

#include "../common.h"

namespace {
class fooTask: public OpenSoT::Task <Eigen::MatrixXd, Eigen::VectorXd>
{
public:
    typedef std::shared_ptr<fooTask> Ptr;

    fooTask(const Eigen::MatrixXd& A, const Eigen::VectorXd& b):Task("foo", 20)
    {
        _A = A;
        _b = b;
    }

    void setA(const Eigen::MatrixXd& A){
        _A = A;
    }
    void setb(const Eigen::VectorXd& b){
        _b = b;
    }
    void setW(const Eigen::MatrixXd& W){
        _W = W;
    }

    ~fooTask(){}
    void _update(const Eigen::VectorXd &x){}
};

class testTask: public TestBase
{
protected:
    testTask(): TestBase("coman")
    {

    }

    virtual ~testTask()
    {

    }

    virtual void SetUp()
    {

    }

    virtual void TearDown()
    {

    }
};

TEST_F(testTask, testCheckConsistency)
{
    OpenSoT::tasks::velocity::Postural::Ptr postural;
    auto q = _model_ptr->generateRandomQ();
    postural.reset(new OpenSoT::tasks::velocity::Postural(*_model_ptr, q));
    postural->update(q);

    EXPECT_TRUE(postural->checkConsistency());

    Eigen::MatrixXd fooA;
    Eigen::VectorXd foob;

    fooTask::Ptr footask;
    footask.reset(new fooTask(fooA, foob));
    footask->update(Eigen::VectorXd::Ones(10));

    EXPECT_FALSE(footask->checkConsistency());

    fooA.setIdentity(13,0);
    footask->setA(fooA);
    EXPECT_FALSE(footask->checkConsistency());

    fooA.setIdentity(13,13);
    footask->setA(fooA);
    EXPECT_FALSE(footask->checkConsistency());

    foob.setOnes(3);
    footask->setb(foob);
    EXPECT_FALSE(footask->checkConsistency());


    Eigen::MatrixXd W(2,3);
    W.setIdentity(2,3);
    footask->setW(W);
    EXPECT_FALSE(footask->checkConsistency());

    W.setIdentity(3,3);
    footask->setW(W);
    EXPECT_FALSE(footask->checkConsistency());

    fooA.setIdentity(13,15);
    foob.setOnes(13);
    footask->setA(fooA);
    footask->setb(foob);
    EXPECT_FALSE(footask->checkConsistency());

    W.setIdentity(13,13);
    footask->setW(W);
    EXPECT_FALSE(footask->checkConsistency());

    fooA.setIdentity(13,20);
    footask->setA(fooA);
    EXPECT_TRUE(footask->checkConsistency());
}

TEST_F(testTask, testComputeCost)
{
    Eigen::VectorXd q = _model_ptr->generateRandomQ();

    OpenSoT::tasks::velocity::Postural::Ptr postural =
            std::make_shared<OpenSoT::tasks::velocity::Postural>(*_model_ptr, q);

    postural->setWeight(10.);
    postural->setLambda(1.);

    postural->update(q);

    Eigen::VectorXd x = _model_ptr->generateRandomQ();

    double cost = (postural->getA()*x - postural->getb()).transpose()*postural->getWeight()*(postural->getA()*x - postural->getb());
    std::cout<<"cost: "<<cost<<std::endl;
    std::cout<<"task cost: "<<postural->computeCost(x)<<std::endl;

    EXPECT_DOUBLE_EQ(cost, postural->computeCost(x));

}

TEST_F(testTask, testDiagonalWeight)
{
    Eigen::VectorXd q = _model_ptr->generateRandomQ();
    OpenSoT::tasks::velocity::Postural::Ptr postural(
        new OpenSoT::tasks::velocity::Postural(*_model_ptr, q));

    EXPECT_FALSE(postural->getWeightIsDiagonalFlag());

    auto start = std::chrono::steady_clock::now();
    postural->getWA();
    auto stop = std::chrono::steady_clock::now();
    auto time_for_WA = std::chrono::duration_cast<std::chrono::microseconds>(stop-start).count();
    std::cout<<"Time elapsed for getWA with NOT diagonal weight: "<<time_for_WA<<" [micro s]"<<std::endl;

    start = std::chrono::steady_clock::now();
    postural->getWb();
    stop = std::chrono::steady_clock::now();
    auto time_for_Wb = std::chrono::duration_cast<std::chrono::microseconds>(stop-start).count();
    std::cout<<"Time elapsed for getWb with NOT diagonal weight: "<<time_for_Wb<<" [micro s]"<<std::endl;

    postural->setWeightIsDiagonalFlag(true);
    EXPECT_TRUE(postural->getWeightIsDiagonalFlag());

    start = std::chrono::steady_clock::now();
    postural->getWA();
     stop = std::chrono::steady_clock::now();
     time_for_WA = std::chrono::duration_cast<std::chrono::microseconds>(stop-start).count();
    std::cout<<"Time elapsed for getWA with diagonal weight: "<<time_for_WA<<" [micro s]"<<std::endl;

    start = std::chrono::steady_clock::now();
    postural->getWb();
    stop = std::chrono::steady_clock::now();
    time_for_Wb = std::chrono::duration_cast<std::chrono::microseconds>(stop-start).count();
    std::cout<<"Time elapsed for getWb with diagonal weight: "<<time_for_Wb<<" [micro s]"<<std::endl;

}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

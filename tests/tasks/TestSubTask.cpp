#include <gtest/gtest.h>
#include <OpenSoT/SubTask.h>
#include <OpenSoT/tasks/velocity/Postural.h>
#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <OpenSoT/constraints/velocity/JointLimits.h>
#include <xbot2_interface/xbotinterface2.h>
#include <OpenSoT/utils/AutoStack.h>

#include "../common.h"


using namespace OpenSoT::tasks;


namespace {

class TestSubTaskMap: public TestBase
{
protected:

    TestSubTaskMap():TestBase("coman_floating_base")
    {

    }

    virtual ~TestSubTaskMap() {
    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }



};

class TestSubTask: public TestBase
{
protected:

    OpenSoT::tasks::velocity::Postural::Ptr _postural;
    OpenSoT::constraints::velocity::JointLimits::Ptr _joint_limits;

    TestSubTask():TestBase("coman_floating_base")

    {
        Eigen::VectorXd tmp;
        tmp = _model_ptr->getNeutralQ();
        _model_ptr->setJointPosition(tmp);
        _model_ptr->update();

        _postural = std::make_shared<OpenSoT::tasks::velocity::Postural>(*_model_ptr);

        Eigen::VectorXd qmin, qmax;
        _model_ptr->getJointLimits(qmin, qmax);


        _joint_limits = std::make_shared<OpenSoT::constraints::velocity::JointLimits>(*_model_ptr, qmax, qmin);

        _postural->update();
        _joint_limits->update();


    }

    virtual ~TestSubTask() {
    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }


};

TEST_F(TestSubTaskMap, testSubMapConstructor)
{
    std::list<unsigned int> indices;
    // first chunk of size 3
    indices.push_back(1);
    indices.push_back(2);
    indices.push_back(3);

    // second chunk of size 2
    indices.push_back(7);
    indices.push_back(8);

    // third chunk of size 2 (11 will get added later)
    indices.push_back(10);

    // fourth chunk of size 1
    indices.push_back(13);

    // fifth chunk of size 3
    indices.push_back(17);
    indices.push_back(18);
    indices.push_back(19);

    // goes into third chunk and increases it to size 2
    indices.push_back(11);

    OpenSoT::Indices subTaskMap(indices);
    ASSERT_EQ(subTaskMap.getChunks().size(), 5);

    OpenSoT::Indices::ChunkList::const_iterator i = subTaskMap.getChunks().begin();
    EXPECT_EQ(i->size(), 3);
    i++;

    ASSERT_TRUE(i != subTaskMap.getChunks().end());
    EXPECT_EQ(i->size(), 2);
    i++;

    ASSERT_TRUE(i != subTaskMap.getChunks().end());
    EXPECT_EQ(i->size(), 2);
    i++;

    ASSERT_TRUE(i != subTaskMap.getChunks().end());
    EXPECT_EQ(i->size(), 1);
    i++;

    ASSERT_TRUE(i != subTaskMap.getChunks().end());
    EXPECT_EQ(i->size(), 3);

    ASSERT_EQ(subTaskMap.asList().size(), 11);
    ASSERT_EQ(subTaskMap.asVector().size(), 11);
    ASSERT_EQ(subTaskMap.size(), 11);
}

TEST_F(TestSubTaskMap, testSubMapToString)
{
    std::list<unsigned int> indices;
    // first chunk of size 3
    indices.push_back(1);
    indices.push_back(2);
    indices.push_back(3);

    // second chunk of size 2
    indices.push_back(7);
    indices.push_back(8);

    // third chunk of size 2 (11 will get added later)
    indices.push_back(10);

    // fourth chunk of size 1
    indices.push_back(13);

    // fifth chunk of size 3
    indices.push_back(17);
    indices.push_back(18);
    indices.push_back(19);

    // goes into third chunk and increases it to size 2
    indices.push_back(11);

    OpenSoT::Indices subTaskMap(indices);
    EXPECT_EQ(std::string(subTaskMap), "1to3plus7to8plus10to11plus13plus17to19");
}

TEST_F(TestSubTaskMap, testSubMapOperatorPlus)
{
    std::list<unsigned int> indices;
    std::list<unsigned int> indices2;
    std::list<unsigned int> indices3;

    std::list<unsigned int> indices1_plus_3;


    indices.push_back(1);
    indices.push_back(2);
    indices.push_back(3);

    indices2.push_back(7);
    indices2.push_back(8);

    indices3.push_back(3);
    indices3.push_back(4);
    indices3.push_back(5);

    indices1_plus_3.insert(indices1_plus_3.end(), indices.begin(), indices.end());
    indices1_plus_3.insert(indices1_plus_3.end(), indices3.begin(), indices3.end());
    indices1_plus_3.sort(); indices1_plus_3.unique();

    OpenSoT::Indices subTaskMap(indices.begin(),indices.end());
    OpenSoT::Indices subTaskMap2(indices2);
    OpenSoT::Indices subTaskMap3(indices3);

    EXPECT_EQ(std::string(subTaskMap+subTaskMap2),"1to3plus7to8");
    EXPECT_EQ(std::list<unsigned int>(subTaskMap+subTaskMap3),indices1_plus_3);
    EXPECT_EQ(std::string(subTaskMap+subTaskMap3),"1to5");
    EXPECT_EQ((subTaskMap+subTaskMap3).getChunks().size(),1);
    EXPECT_EQ(std::string(subTaskMap2+subTaskMap3),"3to5plus7to8");
    EXPECT_EQ(std::string(subTaskMap2+subTaskMap3+9),"3to5plus7to9");
    EXPECT_EQ(std::string(subTaskMap2+10+subTaskMap3),"3to5plus7to8plus10");
    EXPECT_EQ(std::string(subTaskMap2+10+11+12+subTaskMap3),"3to5plus7to8plus10to12");
}

TEST_F(TestSubTaskMap, testSubMapOperatorMinus)
{
    std::list<unsigned int> indices;
    std::list<unsigned int> indices2;
    std::list<unsigned int> indices3;

    std::list<unsigned int> indices1_minus_3;


    indices.push_back(1);
    indices.push_back(2);
    indices.push_back(3);

    indices3.push_back(3);

    indices1_minus_3.push_back(1);
    indices1_minus_3.push_back(2);

    OpenSoT::Indices subTaskMap(indices.begin(),indices.end());
    OpenSoT::Indices subTaskMap3(indices3);

    EXPECT_EQ(std::string(subTaskMap-subTaskMap3),"1to2");
    EXPECT_EQ(std::list<unsigned int>(subTaskMap-subTaskMap3),indices1_minus_3);
    EXPECT_EQ(std::string(subTaskMap-1-2),"3");
}

TEST_F(TestSubTaskMap, testSubTaskOperatorToList)
{
    std::list<unsigned int> indices;
    // first chunk of size 3
    indices.push_back(1);
    indices.push_back(2);
    indices.push_back(3);

    // second chunk of size 2
    indices.push_back(7);
    indices.push_back(8);

    // third chunk of size 2 (11 will get added later)
    indices.push_back(10);

    // fourth chunk of size 1
    indices.push_back(13);

    // fifth chunk of size 3
    indices.push_back(17);
    indices.push_back(18);
    indices.push_back(19);

    // goes into third chunk and increases it to size 2
    indices.push_back(11);

    OpenSoT::Indices subTaskMap(indices);
    indices.sort(); indices.unique();
    EXPECT_EQ(std::list<unsigned int>(subTaskMap), indices);
}

TEST_F(TestSubTaskMap, testRange)
{
    std::list<unsigned int> indices;
    // first chunk of size 3
    indices.push_back(1);
    indices.push_back(2);
    indices.push_back(3);

    OpenSoT::Indices subTaskMap(indices);
    EXPECT_EQ("1to3", std::string(OpenSoT::Indices::range(1,3)));
    EXPECT_EQ(subTaskMap, OpenSoT::Indices::range(1,3));
}

static inline bool matrixAreEqual(const Eigen::MatrixXd& m0,
                                  const Eigen::MatrixXd& m1)
{
    bool sizeAreCompatible = (m0.rows() == m1.rows() &&
                              m0.cols() == m1.cols());
    EXPECT_TRUE(sizeAreCompatible) << "Size of compared matrices "
                                   << "are not compatible";
    if(!sizeAreCompatible)
        return false;

    bool areEqual = true;
    for(unsigned int r = 0; r < m0.rows(); ++r)
        for(unsigned int c = 0; c < m0.cols(); ++c) {
            EXPECT_DOUBLE_EQ(m0(r,c), m1(r,c)) << "Elements in ("
                                               << r << "," << c
                                               << ") are not equal";

            using namespace testing::internal;
            bool elementAreEqual;
            FloatingPoint<double> lhs(m0(r,c));
            FloatingPoint<double> rhs(m1(r,c));
            elementAreEqual = lhs.AlmostEquals(rhs);

            areEqual = areEqual & elementAreEqual;

        }
    return areEqual;
}

TEST_F(TestSubTask, testgetA)
{
    using namespace OpenSoT;

    SubTask::Ptr subTask = std::make_shared<SubTask>(_postural, Indices::range(0,2));
    ASSERT_EQ(subTask->getA().rows(), 3);
    ASSERT_EQ(subTask->getA().cols(), _postural->getXSize());
    Eigen::MatrixXd A(3,_postural->getXSize());
    A = _postural->getA().block(0,0,3,_postural->getXSize());
    EXPECT_TRUE(matrixAreEqual(subTask->getA(), A));

    subTask.reset();
    subTask= std::make_shared<SubTask>(_postural, Indices::range(0,2) +
                                                 Indices::range(5,6));
    ASSERT_EQ(subTask->getA().rows(), 5);
    ASSERT_EQ(subTask->getA().cols(), _postural->getXSize());
    A.resize(5, _postural->getXSize()); A.setZero(A.rows(), A.cols());
    A << _postural->getA().block(0,0,3,_postural->getXSize()),
         _postural->getA().block(5,0,2,_postural->getXSize());
    EXPECT_TRUE(matrixAreEqual(subTask->getA(),A));
}

TEST_F(TestSubTask, testGetHessianAType)
{
    using namespace OpenSoT;

    SubTask::Ptr subTask = std::make_shared<SubTask>(_postural, Indices::range(1,3));
    ASSERT_EQ(subTask->getHessianAtype(), HST_POSDEF);
}

static inline bool vectorAreEqual(const Eigen::VectorXd& v0,
                                  const Eigen::VectorXd& v1)
{
    bool sizeAreCompatible = (v0.size() == v1.size());
    EXPECT_TRUE(sizeAreCompatible) << "Size of compared vectors "
                                   << "are not equal";
    if(!sizeAreCompatible)
        return false;

    bool areEqual = true;
    for(unsigned int s = 0; s < v0.size(); ++s) {
        EXPECT_DOUBLE_EQ(v0(s), v1(s)) << "Elements in  ("
                                           << s
                                           << ") are not equal";

        using namespace testing::internal;
        bool elementAreEqual;
        FloatingPoint<double> lhs(v0(s));
        FloatingPoint<double> rhs(v1(s));
        elementAreEqual = lhs.AlmostEquals(rhs);

        areEqual = areEqual & elementAreEqual;

    }
    return areEqual;
}

TEST_F(TestSubTask, testgetb)
{
    using namespace OpenSoT;

    SubTask::Ptr subTask = std::make_shared<SubTask>(_postural, Indices::range(0,2));
    ASSERT_EQ(subTask->getb().size(), 3);
    Eigen::VectorXd b(3), bLambda(3);
    b = _postural->getb().segment(0,3);
    EXPECT_TRUE(vectorAreEqual(subTask->getb(), b));

    subTask.reset();
    subTask = std::make_shared<SubTask>(_postural, Indices::range(0,2) +
                                                 Indices::range(5,6));
    ASSERT_EQ(subTask->getb().size(), 5);
    b.resize(5);
    b << _postural->getb().segment(0,3), _postural->getb().segment(5,2);
    EXPECT_TRUE(vectorAreEqual(subTask->getb(),b));

    b = subTask->getb();
    subTask->setLambda(0.1);
    subTask->update();
    bLambda = subTask->getb();

    EXPECT_TRUE(vectorAreEqual(bLambda,b*0.1));
}

TEST_F(TestSubTask, testgetWeight)
{
    using namespace OpenSoT;

    Eigen::MatrixXd W(_postural->getWeight().rows(), _postural->getWeight().cols());
    W.setZero();
    for(unsigned int i = 0; i < W.rows(); ++i)
        W(i,i) = i;
    _postural->setWeight(W);

    std::cout<<"_postural->getWeight(): \n"<<_postural->getWeight()<<std::endl;


    SubTask::Ptr subTask = std::make_shared<SubTask>(_postural, Indices::range(0,2));
    ASSERT_EQ(subTask->getWeight().rows(), 3);
    ASSERT_EQ(subTask->getWeight().cols(), 3);

    subTask->update();

    std::cout<<"subTask->getWeight(): \n"<<subTask->getWeight()<<std::endl;

    EXPECT_TRUE(matrixAreEqual(subTask->getWeight(),_postural->getWeight().block(0,0,3,3)));


    auto tmp = 10.*subTask;
    std::cout<<"tmp->getWeight(): \n"<<tmp->getWeight()<<std::endl;
    std::cout<<"_postural->getWeight(): \n"<<_postural->getWeight()<<std::endl;
    EXPECT_TRUE(matrixAreEqual(subTask->getWeight(),_postural->getWeight().block(0,0,3,3)));
    EXPECT_TRUE(matrixAreEqual(subTask->getWeight(),tmp->getWeight()));

    W.setIdentity();
    _postural->setWeight(W);
    std::cout<<"_postural->getWeight(): \n"<<_postural->getWeight()<<std::endl;

    subTask->update();
    std::cout<<"subTask->getWeight(): \n"<<subTask->getWeight()<<std::endl;

    tmp->update();
    std::cout<<"tmp->getWeight(): \n"<<tmp->getWeight()<<std::endl;

    EXPECT_TRUE(matrixAreEqual(subTask->getWeight(),_postural->getWeight().block(0,0,3,3)));
    EXPECT_TRUE(matrixAreEqual(subTask->getWeight(),tmp->getWeight()));

    tmp->setWeight(2*W.block(0,0,3,3));
    subTask->update();
    tmp->update();
    std::cout<<"_postural->getWeight(): \n"<<_postural->getWeight()<<std::endl;
    std::cout<<"subTask->getWeight(): \n"<<subTask->getWeight()<<std::endl;
    std::cout<<"tmp->getWeight(): \n"<<tmp->getWeight()<<std::endl;

    EXPECT_TRUE(matrixAreEqual(subTask->getWeight(),_postural->getWeight().block(0,0,3,3)));
    EXPECT_TRUE(matrixAreEqual(subTask->getWeight(),tmp->getWeight()));


    OpenSoT::Indices id(1);
    auto tmp2 = 2*(tmp%id);


    std::cout<<"tmp2->getWeight(): "<<tmp2->getWeight()<<std::endl;

    EXPECT_EQ(tmp2->getWeight()(0,0), 4);
    EXPECT_TRUE(matrixAreEqual(tmp2->getWeight(),tmp->getWeight().block(1,1,1,1)));
    EXPECT_TRUE(matrixAreEqual(subTask->getWeight().block(1,1,1,1),tmp2->getWeight()));
    EXPECT_TRUE(matrixAreEqual(subTask->getWeight().block(1,1,1,1),_postural->getWeight().block(1,1,1,1)));


    std::cout<<"_postural->getWeight(): \n"<<_postural->getWeight()<<std::endl;
    std::cout<<"subTask->getWeight(): \n"<<subTask->getWeight()<<std::endl;
    std::cout<<"tmp->getWeight(): \n"<<tmp->getWeight()<<std::endl;
    std::cout<<"tmp2->getWeight(): \n"<<tmp2->getWeight()<<std::endl;


    subTask.reset();
    subTask= std::make_shared<SubTask>(_postural, Indices::range(0,2) +
                                                 Indices::range(5,6));
    ASSERT_EQ(subTask->getWeight().rows(), 5);
    ASSERT_EQ(subTask->getWeight().cols(), 5);
    W.resize(5, 5);
    unsigned int c_indices[] = {0, 1, 2, 5, 6};
    std::vector<unsigned int> indices(c_indices,c_indices+sizeof(c_indices)/sizeof(indices[0]));
    for(unsigned int r = 0; r < indices.size(); ++r)
        for(unsigned int c = 0; c < indices.size(); ++c)
            W(r,c) = _postural->getWeight()(indices[r], indices[c]);
    EXPECT_TRUE(matrixAreEqual(subTask->getWeight(),W));
}

TEST_F(TestSubTask, testsetWeight)
{
    using namespace OpenSoT;

    Eigen::MatrixXd fullW = _postural->getWeight();
    Eigen::MatrixXd W(3,3);
    W.setZero();

    for(unsigned int i = 0; i < 3; ++i)
    {
        W(i,i) = i+1;
        fullW(i,i) = i+1;
    }


    SubTask::Ptr subTask = std::make_shared<SubTask>(_postural, Indices::range(0,2));
    subTask->setWeight(W);

    ASSERT_EQ(subTask->getWeight().rows(), 3);
    ASSERT_EQ(subTask->getWeight().cols(), 3);

    EXPECT_TRUE(matrixAreEqual(_postural->getWeight(), fullW))  <<
        "_postural->getWeight() is:\n" << _postural->getWeight() <<
        "\nshould be:\n" << fullW;
    EXPECT_TRUE(matrixAreEqual(subTask->getWeight(),W));

    W.resize(5,5); W.setZero();
    for(unsigned int i = 0; i < 3; ++i) { W(i,i) = i+1; fullW(i,i) = i+1; }
    for(unsigned int i = 3; i < 5 ; ++i) { W(i,i) = i+1; fullW(i+2,i+2) = i+1; }

    subTask = std::make_shared<SubTask>(_postural, Indices::range(0,2) +
                                                  Indices::range(5,6));
    subTask->setWeight(W);
    ASSERT_EQ(subTask->getWeight().rows(), 5);
    ASSERT_EQ(subTask->getWeight().cols(), 5);

    EXPECT_TRUE(matrixAreEqual(_postural->getWeight(),fullW)) << "\n"    << _postural->getWeight()
                                                                           << "\nto\n" << fullW << "\n";
    EXPECT_TRUE(matrixAreEqual(subTask->getWeight(),W));

}


TEST_F(TestSubTask, testGetConstraints)
{
    using namespace OpenSoT;

    _postural->getConstraints().push_back(_joint_limits);

    SubTask::Ptr subTask = std::make_shared<SubTask>(_postural, Indices::range(1,3));

    Eigen::VectorXd lowerBounds = subTask->getConstraints().front()->getLowerBound();
    Eigen::VectorXd b = subTask->getb();

    Eigen::VectorXd q = _model_ptr->generateRandomQ();
    _model_ptr->setJointPosition(q);
    _model_ptr->update();


    subTask->update();

    EXPECT_FALSE( b == subTask->getb())<<"b: "<<b.transpose()<<"\n"<<"subTask->getb(): "<<subTask->getb()<<std::endl;
    EXPECT_FALSE( lowerBounds == subTask->getConstraints().front()->getLowerBound());

    lowerBounds = subTask->getConstraints().front()->getLowerBound();
    b = subTask->getb();

    q = _model_ptr->generateRandomQ();
    _model_ptr->setJointPosition(q);
    _model_ptr->update();

    subTask->update();

    EXPECT_FALSE( b == subTask->getb())<<"b: "<<b.transpose()<<"\n"<<"subTask->getb(): "<<subTask->getb()<<std::endl;
    EXPECT_FALSE( lowerBounds == subTask->getConstraints().front()->getLowerBound());
}

TEST_F(TestSubTask, testUpdate)
{
    using namespace OpenSoT;

    _postural->getConstraints().push_back(_joint_limits);
    SubTask::Ptr subTask = std::make_shared<SubTask>(_postural, Indices::range(1,3));


    ASSERT_EQ(_postural->getConstraints(), subTask->getConstraints());
}

TEST_F(TestSubTask, testGetTaskSize)
{
    using namespace OpenSoT;

    _postural->getConstraints().push_back(_joint_limits);
    SubTask::Ptr subTask = std::make_shared<SubTask>(_postural, Indices::range(1,3));
    ASSERT_EQ(subTask->getTaskSize(), 3);

    subTask = std::make_shared<SubTask>(_postural, Indices::range(1,3) +
                                                  Indices::range(6,7));
    ASSERT_EQ(subTask->getTaskSize(), 5);
}

void check_matrix_are_equal(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B)
{
    EXPECT_EQ(A.rows(), B.rows());
    EXPECT_EQ(A.cols(), B.cols());
    for(unsigned int i = 0; i < A.rows(); ++i)
    {
        for(unsigned int j = 0; j < B.rows(); ++j)
            EXPECT_NEAR(A(i,j),B(i,j),1e-10);
    }
}

void check_vector_are_equal(const Eigen::VectorXd& A, const Eigen::VectorXd& B)
{
    EXPECT_EQ(A.size(), B.size());
    for(unsigned int j = 0; j < B.size(); ++j)
        EXPECT_NEAR(A(j),B(j),1e-10);
}

TEST_F(TestSubTask, testWithCartesian)
{
    using namespace OpenSoT::tasks::velocity;

    // setting initial position with bent legs
    Eigen::VectorXd q_whole = _model_ptr->getNeutralQ();
    q_whole[_model_ptr->getDofIndex("RHipSag")+1 ] = -25.0*M_PI/180.0;
    q_whole[_model_ptr->getDofIndex("RKneeSag")+1 ] = 50.0*M_PI/180.0;
    q_whole[_model_ptr->getDofIndex("RAnkSag")+1 ] = -25.0*M_PI/180.0;
    q_whole[_model_ptr->getDofIndex("LHipSag")+1 ] = -25.0*M_PI/180.0;
    q_whole[_model_ptr->getDofIndex("LKneeSag")+1 ] = 50.0*M_PI/180.0;
    q_whole[_model_ptr->getDofIndex("LAnkSag")+1 ] = -25.0*M_PI/180.0;

    _model_ptr->setJointPosition(q_whole);
    _model_ptr->update();

    Cartesian::Ptr left_leg = std::make_shared<Cartesian>("lleg", *_model_ptr,
                                          "l_sole", "Waist");
    Eigen::MatrixXd ref(4,4); ref.setRandom(4,4);
    Eigen::VectorXd vref(6); vref.setRandom(6);
    left_leg->setReference(ref, vref);

    std::list<unsigned int> indices;
    indices.push_back(0);
    indices.push_back(1);
    indices.push_back(2);
    OpenSoT::SubTask::Ptr pose = std::make_shared<OpenSoT::SubTask>(left_leg, indices);

    left_leg->update();
    pose->update();


    check_matrix_are_equal(pose->getA(), left_leg->getA().block(0,0,3,_model_ptr->getNv()));
    std::cout<<"pose->getA(): "<<pose->getA()<<std::endl;
    std::cout<<"left_leg->getA().block(0,0,3,_model_ptr->getNv()): "<<left_leg->getA().block(0,0,3,_model_ptr->getNv())<<std::endl;
    check_vector_are_equal(pose->getb(), left_leg->getb().segment(0,3));
    std::cout<<"pose->getb(): "<<pose->getb()<<std::endl;
    std::cout<<"left_leg->getb().segment(0,3): "<<left_leg->getb().segment(0,3)<<std::endl;


    indices.clear();
    indices.push_back(3);
    indices.push_back(4);
    indices.push_back(5);
    OpenSoT::SubTask::Ptr pose2 = std::make_shared<OpenSoT::SubTask>(left_leg, indices);

    left_leg->update();
    pose2->update();


    check_matrix_are_equal(pose2->getA(), left_leg->getA().block(3,0,3,_model_ptr->getNv()));
    std::cout<<"pose2->getA(): "<<pose->getA()<<std::endl;
    std::cout<<"left_leg->getA().block(0,0,3,_model_ptr->getNv()): "<<left_leg->getA().block(3,0,3,_model_ptr->getNv())<<std::endl;
    check_vector_are_equal(pose2->getb(), left_leg->getb().segment(3,3));
    std::cout<<"pose2->getb(): "<<pose->getb()<<std::endl;
    std::cout<<"left_leg->getb().segment(0,3): "<<left_leg->getb().segment(3,3)<<std::endl;



    indices.clear();
    indices.push_back(1);
    indices.push_back(4);
    indices.push_back(5);
    OpenSoT::SubTask::Ptr subtask = std::make_shared<OpenSoT::SubTask>(left_leg, indices);
    subtask->update();

    EXPECT_EQ(subtask->getA().rows(), indices.size());
    EXPECT_EQ(subtask->getA().cols(), _model_ptr->getNv());
    EXPECT_EQ(subtask->getb().size(), indices.size());

    for(unsigned int i = 0; i < _model_ptr->getNv(); ++i)
        EXPECT_NEAR(subtask->getA().row(0)[i],left_leg->getA().row(1)[i],1e-10);
    for(unsigned int i = 0; i < _model_ptr->getNv(); ++i)
        EXPECT_NEAR(subtask->getA().row(1)[i],left_leg->getA().row(4)[i],1e-10);
    for(unsigned int i = 0; i < _model_ptr->getNv(); ++i)
        EXPECT_NEAR(subtask->getA().row(2)[i],left_leg->getA().row(5)[i],1e-10);

    EXPECT_NEAR(subtask->getb()[0],left_leg->getb()[1],1e-10);
    EXPECT_NEAR(subtask->getb()[1],left_leg->getb()[4],1e-10);
    EXPECT_NEAR(subtask->getb()[2],left_leg->getb()[5],1e-10);




    indices.clear();
    indices.push_back(0);
    indices.push_back(4);
    OpenSoT::SubTask::Ptr subtask2 = std::make_shared<OpenSoT::SubTask>(left_leg, indices);
    subtask2->update();

    EXPECT_EQ(subtask2->getA().rows(), indices.size());
    EXPECT_EQ(subtask2->getA().cols(), _model_ptr->getNv());
    EXPECT_EQ(subtask2->getb().size(), indices.size());

    for(unsigned int i = 0; i < _model_ptr->getNv(); ++i)
        EXPECT_NEAR(subtask2->getA().row(0)[i],left_leg->getA().row(0)[i],1e-10);
    for(unsigned int i = 0; i < _model_ptr->getNv(); ++i)
        EXPECT_NEAR(subtask2->getA().row(1)[i],left_leg->getA().row(4)[i],1e-10);

    EXPECT_NEAR(subtask2->getb()[0],left_leg->getb()[0],1e-10);
    EXPECT_NEAR(subtask2->getb()[1],left_leg->getb()[4],1e-10);

    std::cout<<"subtask2->getb()"<<subtask2->getb()<<std::endl;
    std::cout<<"left_leg->getb()"<<left_leg->getb()<<std::endl;

}

TEST_F(TestSubTask, testSubTaskCost)
{
    using namespace OpenSoT::tasks::velocity;

    // setting initial position with bent legs
    Eigen::VectorXd q_whole = _model_ptr->getNeutralQ();
    q_whole[_model_ptr->getDofIndex("RHipSag")+1 ] = -25.0*M_PI/180.0;
    q_whole[_model_ptr->getDofIndex("RKneeSag")+1 ] = 50.0*M_PI/180.0;
    q_whole[_model_ptr->getDofIndex("RAnkSag")+1 ] = -25.0*M_PI/180.0;
    q_whole[_model_ptr->getDofIndex("LHipSag")+1 ] = -25.0*M_PI/180.0;
    q_whole[_model_ptr->getDofIndex("LKneeSag")+1 ] = 50.0*M_PI/180.0;
    q_whole[_model_ptr->getDofIndex("LAnkSag")+1 ] = -25.0*M_PI/180.0;

    _model_ptr->setJointPosition(q_whole);
    _model_ptr->update();

    Cartesian::Ptr left_leg = std::make_shared<Cartesian>("lleg",  *_model_ptr,
                                          "l_sole", "Waist");
    Eigen::MatrixXd ref(4,4); ref.setRandom(4,4);
    Eigen::VectorXd vref(6); vref.setRandom(6);
    left_leg->setReference(ref, vref);

    Eigen::Matrix6d W = 200. * Eigen::Matrix6d::Identity();
    left_leg->setWeight(W);


    std::list<unsigned int> position_indices;
    position_indices.push_back(0);
    position_indices.push_back(1);
    position_indices.push_back(2);
    OpenSoT::SubTask::Ptr position = std::make_shared<OpenSoT::SubTask>(left_leg, position_indices);

    std::list<unsigned int> orientation_indices;
    orientation_indices.push_back(3);
    orientation_indices.push_back(4);
    orientation_indices.push_back(5);
    OpenSoT::SubTask::Ptr orientation = std::make_shared<OpenSoT::SubTask>(left_leg, orientation_indices);

    left_leg->update();


    Eigen::VectorXd dq(_model_ptr->getNv()); dq.setRandom();
    double left_leg_cost = left_leg->computeCost(dq);
    double position_cost = position->computeCost(dq);
    double orientation_cost = orientation->computeCost(dq);

    std::cout<<"left_leg cost: "<<left_leg->computeCost(dq)<<std::endl;
    std::cout<<"left_leg position cost: "<<position->computeCost(dq)<<std::endl;
    std::cout<<"left_leg orientation cost: "<<orientation->computeCost(dq)<<std::endl;

    EXPECT_DOUBLE_EQ(left_leg_cost, position_cost + orientation_cost);
}

TEST_F(TestSubTask, testWithPostural)
{
    Eigen::VectorXd q = _model_ptr->getNeutralQ();
    _model_ptr->setJointPosition(q);
    _model_ptr->update();
    Eigen::VectorXd q_ref = _model_ptr->generateRandomQ();

    using namespace OpenSoT::tasks;

    velocity::Postural::Ptr postural = std::make_shared<velocity::Postural>(*_model_ptr);
    postural->setReference(q_ref);
    postural->update();

    std::list<unsigned int> indices;
    // first chunk of size 3
    indices.push_back(1);
    indices.push_back(2);
    indices.push_back(3);
    // second chunk of size 2
    indices.push_back(7);
    indices.push_back(8);
    // third chunk of size 2 (11 will get added later)
    indices.push_back(10);
    // fourth chunk of size 1
    indices.push_back(13);
    // fifth chunk of size 3
    indices.push_back(17);
    indices.push_back(18);
    indices.push_back(19);
    // goes into third chunk and increases it to size 2
    indices.push_back(11);

    OpenSoT::SubTask::Ptr subtask = std::make_shared<OpenSoT::SubTask>(postural, indices);
    subtask->update();


    for(unsigned int j = 0; j < _model_ptr->getNv(); ++j)
        EXPECT_NEAR(subtask->getA().row(0)[j], postural->getA().row(1)[j], 1e-10);
    for(unsigned int j = 0; j < _model_ptr->getNv(); ++j)
        EXPECT_NEAR(subtask->getA().row(1)[j], postural->getA().row(2)[j], 1e-10);
    for(unsigned int j = 0; j < _model_ptr->getNv(); ++j)
        EXPECT_NEAR(subtask->getA().row(2)[j], postural->getA().row(3)[j], 1e-10);

    for(unsigned int j = 0; j < _model_ptr->getNv(); ++j)
        EXPECT_NEAR(subtask->getA().row(3)[j], postural->getA().row(7)[j], 1e-10);
    for(unsigned int j = 0; j < _model_ptr->getNv(); ++j)
        EXPECT_NEAR(subtask->getA().row(4)[j], postural->getA().row(8)[j], 1e-10);

    for(unsigned int j = 0; j < _model_ptr->getNv(); ++j)
        EXPECT_NEAR(subtask->getA().row(5)[j], postural->getA().row(10)[j], 1e-10);
    for(unsigned int j = 0; j < _model_ptr->getNv(); ++j)
        EXPECT_NEAR(subtask->getA().row(6)[j], postural->getA().row(11)[j], 1e-10);

    for(unsigned int j = 0; j < _model_ptr->getNv(); ++j)
        EXPECT_NEAR(subtask->getA().row(7)[j], postural->getA().row(13)[j], 1e-10);

    for(unsigned int j = 0; j < _model_ptr->getNv(); ++j)
        EXPECT_NEAR(subtask->getA().row(8)[j], postural->getA().row(17)[j], 1e-10);
    for(unsigned int j = 0; j < _model_ptr->getNv(); ++j)
        EXPECT_NEAR(subtask->getA().row(9)[j], postural->getA().row(18)[j], 1e-10);
    for(unsigned int j = 0; j < _model_ptr->getNv(); ++j)
        EXPECT_NEAR(subtask->getA().row(10)[j], postural->getA().row(19)[j], 1e-10);


}



}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

#include <idynutils/tests_utils.h>
#include <gtest/gtest.h>
#include <OpenSoT/constraints/velocity/JointLimits.h>
#include <yarp/math/Math.h>

using namespace yarp::math;

/*
 * Copied from Silvio Traversaro's iDynTree
 *
 * Copyright (C) 2013 RobotCub Consortium
 * Author: Silvio Traversaro
 * CopyPolicy: Released under the terms of the GNU LGPL v2.0 (or any later version)
 *
 */

#ifndef IDYNTREE_YARP_EIGEN_CONVERSIONS_H
#define IDYNTREE_YARP_EIGEN_CONVERSIONS_H

#include <Eigen/Dense>

namespace iDynTree
{

inline Eigen::Map<Eigen::VectorXd> toEigen(yarp::sig::Vector & yarpVector)
{
    return Eigen::Map<Eigen::VectorXd>(yarpVector.data(),yarpVector.size());
}

inline Eigen::Map< Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> > toEigen(yarp::sig::Matrix & yarpMatrix)
{
    return Eigen::Map< Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> >(yarpMatrix.data(),yarpMatrix.rows(),yarpMatrix.cols());
}

inline Eigen::Map<const Eigen::VectorXd> toEigen(const yarp::sig::Vector & yarpVector)
{
    return Eigen::Map<const Eigen::VectorXd>(yarpVector.data(),yarpVector.size());
}

inline Eigen::Map<const Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> > toEigen(const yarp::sig::Matrix & yarpMatrix)
{
    return Eigen::Map<const Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> >(yarpMatrix.data(),yarpMatrix.rows(),yarpMatrix.cols());
}

}

#endif

#ifndef _TEST_LEGACY_POSTURAL_
#define _TEST_LEGACY_POSTURAL_

#include <OpenSoT/tasks/velocity/Postural.h>

namespace OpenSoT{
namespace test{
namespace legacy{

class yarp_Postural : public OpenSoT::tasks::velocity::Postural {
            public:
                static yarp::sig::Matrix toYarp(const Eigen::MatrixXd& M)
                {
                    yarp::sig::Matrix tmp(M.rows(), M.cols());
                    for(unsigned int i = 0; i < M.rows(); ++i)
                        for(unsigned int j = 0; j < M.cols(); ++j)
                            tmp(i,j) = M(i,j);
                    return tmp;
                }

                static yarp::sig::Vector toYarp(const Eigen::VectorXd& v)
                {
                    yarp::sig::Vector tmp(v.rows(), 0.0);
                    for(unsigned int i = 0; i < v.rows(); ++i)
                        tmp(i) = v(i);
                    return tmp;
                }

            public:

                yarp_Postural(const yarp::sig::Vector& x):
                    Postural(iDynTree::toEigen(x)) {
                }


                void setReference(const yarp::sig::Vector& x_desired)
                {
                    Postural::setReference(iDynTree::toEigen(x_desired));
                }

                void setReference(const yarp::sig::Vector& x_desired,
                                  const yarp::sig::Vector& xdot_desired)
                {
                    Postural::setReference(iDynTree::toEigen(x_desired),
                                           iDynTree::toEigen(xdot_desired));
                }

                yarp::sig::Vector getReference(){
                    Eigen::VectorXd tmp = Postural::getReference();
                    return toYarp(tmp);
                }


                void getReference(yarp::sig::Vector& x_desired,
                                  yarp::sig::Vector& xdot_desired)
                {
                    Eigen::VectorXd tmp1;
                    Eigen::VectorXd tmp2;
                    Postural::getReference(tmp1, tmp2);
                    x_desired = toYarp(tmp1);
                    xdot_desired = toYarp(tmp2);
                }

                void setLambda(double lambda)
                {
                    Postural::setLambda(lambda);
                }

                yarp::sig::Vector getActualPositions()
                {
                    Eigen::VectorXd tmp = Postural::getActualPositions();
                    return toYarp(tmp);
                }

                yarp::sig::Vector getError()
                {
                    Eigen::VectorXd tmp = Postural::getError();
                    return toYarp(tmp);
                }

            };

}
}
}

#endif


namespace {

class testPosturalTask: public ::testing::Test
{
protected:

    testPosturalTask()
    {

    }

    virtual ~testPosturalTask() {

    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }

};

TEST_F(testPosturalTask, testPosturalTask_)
{
    yarp::sig::Vector q(6, 0.0);
    for(unsigned int i = 0; i < q.size(); ++i)
        q[i] = tests_utils::getRandomAngle();

    yarp::sig::Vector q_ref(q.size(), 0.0);

    OpenSoT::test::legacy::yarp_Postural postural(q);
    std::cout<<"Postural Task Inited"<<std::endl;
    EXPECT_TRUE(postural.getA() == iDynTree::toEigen(yarp::sig::Matrix(q.size(), q.size()).eye()));
    EXPECT_TRUE(postural.getWeight() == iDynTree::toEigen(yarp::sig::Matrix(q.size(), q.size()).eye()));
    EXPECT_TRUE(postural.getConstraints().size() == 0);

    double K = 0.1;
    postural.setLambda(K);
    EXPECT_DOUBLE_EQ(postural.getLambda(), K);

    postural.setReference(q_ref);
    postural.update(iDynTree::toEigen(q));
    EXPECT_TRUE(postural.getb() == iDynTree::toEigen(postural.getLambda()*(q_ref-q)));

    for(unsigned int i = 0; i < 100; ++i)
    {
        postural.update(iDynTree::toEigen(q));
        yarp::sig::Vector qq = OpenSoT::test::legacy::yarp_Postural::toYarp(postural.getb());
        q += qq;
    }

    for(unsigned int i = 0; i < q.size(); ++i)
        EXPECT_NEAR(q[i], q_ref[i], 1E-3);
}

//TEST_F(testPosturalTask, testPosturalTaskWithJointLimits_)
//{
//    iDynUtils idynutils("coman",
//                        std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.urdf",
//                        std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.srdf");
//    using namespace OpenSoT::tasks::velocity;
//    using namespace OpenSoT::constraints::velocity;

//    yarp::sig::Vector q(idynutils.iDyn3_model.getNrOfDOFs(), 0.0);
//    yarp::sig::Vector q_next(q);

//    for(unsigned int i = 0; i < q.size(); ++i) {
//        q[i] = tests_utils::getRandomAngle();
//        q_next[i] = tests_utils::getRandomAngle();
//        assert((q[i]!=q_next[i]));
//    }
//    idynutils.updateiDyn3Model(q);

//    Postural::TaskPtr postural( new OpenSoT::test::legacy::Posturall(q) );
//    Postural::ConstraintPtr bound(
//        new JointLimits(q,
//                        idynutils.iDyn3_model.getJointBoundMax(),
//                        idynutils.iDyn3_model.getJointBoundMin())
//    );

//    postural->getConstraints().push_back( bound );

//    yarp::sig::Vector old_b = postural->getb();
//    yarp::sig::Vector old_LowerBound = bound->getLowerBound();
//    yarp::sig::Vector old_UpperBound = bound->getUpperBound();
//    idynutils.updateiDyn3Model(q_next);
//    postural->update(q_next);
//    yarp::sig::Vector new_b = postural->getb();
//    yarp::sig::Vector new_LowerBound = bound->getLowerBound();
//    yarp::sig::Vector new_UpperBound = bound->getUpperBound();

//    EXPECT_FALSE(old_b == new_b);
//    EXPECT_FALSE(old_LowerBound == new_LowerBound);
//    EXPECT_FALSE(old_UpperBound == new_UpperBound);

//}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

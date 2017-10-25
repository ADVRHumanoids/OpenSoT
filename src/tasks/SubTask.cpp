#include "OpenSoT/SubTask.h"

OpenSoT::SubTask::SubTask(OpenSoT::SubTask::TaskPtr taskPtr, const std::list<unsigned int> rowIndices) :
    Task(taskPtr->getTaskID() + "_" + std::string(Indices(rowIndices)),
         taskPtr->getXSize()),
    _subTaskMap(rowIndices),
    _taskPtr(taskPtr)
{
    this->_A.resize(rowIndices.size(), _x_size);
    this->_b.resize(rowIndices.size());
    this->_W.resize(rowIndices.size(), rowIndices.size());

    this->generateA();
    this->generateb();
    this->generateHessianAtype();
    this->generateWeight();
}

void OpenSoT::SubTask::generateA()
{
    unsigned int chunk_size = 0;
    unsigned int j = 0;
    for(Indices::ChunkList::const_iterator i = _subTaskMap.getChunks().begin();
        i != _subTaskMap.getChunks().end(); ++i)
    {
        chunk_size = i->size();
        this->_A.block(j,0,chunk_size,_x_size) = _taskPtr->getA().block(i->front(),0,i->back()-i->front()+1, _x_size);
        j+=chunk_size;
    }
}

void OpenSoT::SubTask::generateHessianAtype()
{
    OpenSoT::HessianType fatherHessianType = _taskPtr->getHessianAtype();

    if ( fatherHessianType == HST_IDENTITY)
        this->_hessianType = HST_POSDEF;
    else this->_hessianType = fatherHessianType;
}

void OpenSoT::SubTask::generateb()
{
    unsigned int chunk_size = 0;
    unsigned int j = 0;
    for(Indices::ChunkList::const_iterator i = _subTaskMap.getChunks().begin();
        i != _subTaskMap.getChunks().end();++i)
    {
        chunk_size = i->size();
        this->_b.segment(j,chunk_size) = _taskPtr->getb().segment(i->front(),i->back()-i->front()+1);
        j+=chunk_size;
    }

    if(this->_b.size() > 0)
        this->_b = this->_b * this->_lambda;

}

void OpenSoT::SubTask::generateWeight()
{
        this->_W.setZero(_W.rows(), _W.cols());

        for(unsigned int r = 0; r < this->getTaskSize(); ++r)
            for(unsigned int c = 0; c < this->getTaskSize(); ++c)
                this->_W(r,c) = _taskPtr->getWeight()(this->_subTaskMap.asVector()[r],
                                                      this->_subTaskMap.asVector()[c]);
}

void OpenSoT::SubTask::setWeight(const Eigen::MatrixXd &W)
{
    assert(W.rows() == this->getTaskSize());
    assert(W.cols() == W.rows());

    this->_W = W;
    Eigen::MatrixXd fullW = _taskPtr->getWeight();
    for(unsigned int r = 0; r < this->getTaskSize(); ++r)
        for(unsigned int c = 0; c < this->getTaskSize(); ++c)
            fullW(this->_subTaskMap.asVector()[r],
                  this->_subTaskMap.asVector()[c]) = this->_W(r,c);

    _taskPtr->setWeight(fullW);
}

std::list<OpenSoT::SubTask::ConstraintPtr> &OpenSoT::SubTask::getConstraints()
{
    return _taskPtr->getConstraints();
}

const unsigned int OpenSoT::SubTask::getTaskSize() const
{
    unsigned int size = 0;
    for(Indices::ChunkList::const_iterator i = _subTaskMap.getChunks().begin();
        i != _subTaskMap.getChunks().end();
        ++i)
    {

        if(_taskPtr->getTaskSize() > i->back()) {
            size += i->size();
        }
    }
    return size;
}

void OpenSoT::SubTask::_update(const Eigen::VectorXd &x)
{
    _taskPtr->update(x);
    this->generateA();
    this->generateb();
    this->generateHessianAtype();
    this->generateWeight();
}

std::vector<bool> OpenSoT::SubTask::getActiveJointsMask()
{
    return _taskPtr->getActiveJointsMask();
}

bool OpenSoT::SubTask::setActiveJointsMask(const std::vector<bool> &active_joints_mask)
{
    return _taskPtr->setActiveJointsMask(active_joints_mask);
}

void OpenSoT::SubTask::_log(XBot::MatLogger::Ptr logger)
{
    _taskPtr->log(logger);
}

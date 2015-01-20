#include "OpenSoT/SubTask.h"

/******************
 *  SUBTASKMAP    *
 ******************/

std::list<unsigned int>::iterator OpenSoT::SubTask::SubTaskMap::getNextAdjacentChunk(std::list<unsigned int>::iterator searchBegin)
{
    if(searchBegin != _rowsList.end())
    {
        for(std::list<unsigned int>::iterator i = searchBegin;
            i != _rowsList.end();
            ++i)
        {
            std::list<unsigned int>::iterator next = i; next++;
            std::list<unsigned int>::iterator prev = i;

            while(next != _rowsList.end() &&
                  *next == *prev + 1) {
                next++;
                prev++;
            }
            return prev;
        }
    }
    return _rowsList.end();
}

void OpenSoT::SubTask::SubTaskMap::generateChunks()
{
    _rowsList.sort(); _rowsList.unique();

    this->_rowsVector.clear();
    this->_rowsVector.insert(this->_rowsVector.end(),
                             this->_rowsList.begin(),
                             this->_rowsList.end());
    this->_contiguousChunks.clear();

    for(std::list<unsigned int>::iterator i = _rowsList.begin();
        i!= _rowsList.end();
        ++i)
    {
        std::list<unsigned int>::iterator chunkEnd =
                this->getNextAdjacentChunk(i);
        /* insert doesn't include the last extreme,
         * so we need to increment chunkEnd */
        if(chunkEnd != _rowsList.end())
        {
            std::list<unsigned int>::iterator chunkAfterEnd = chunkEnd;
            ++chunkAfterEnd;
            RowsChunk chunk; chunk.insert(chunk.end(), i, chunkAfterEnd);
            this->_contiguousChunks.push_back(chunk);
        }
        i = chunkEnd;
    }
}

OpenSoT::SubTask::SubTaskMap::SubTaskMap(unsigned int i)
{
    _rowsList.push_back(i);
    this->generateChunks();
}

OpenSoT::SubTask::SubTaskMap::SubTaskMap(const std::list<unsigned int> &rowsList)
    : _rowsList(rowsList)
{
    this->generateChunks();
}

OpenSoT::SubTask::SubTaskMap::SubTaskMap(const OpenSoT::SubTask::SubTaskMap &subTaskMap)
    : _rowsList(subTaskMap.getRowsList())
{
    this->generateChunks();
}

const OpenSoT::SubTask::SubTaskMap::ChunkList& OpenSoT::SubTask::SubTaskMap::getChunks() const
{
    return _contiguousChunks;
}

const std::list<unsigned int>& OpenSoT::SubTask::SubTaskMap::getRowsList() const
{
    return _rowsList;
}

const std::vector<unsigned int> &OpenSoT::SubTask::SubTaskMap::getRowsVector() const
{
    return _rowsVector;
}

bool OpenSoT::SubTask::SubTaskMap::isContiguous() const
{
    return _contiguousChunks.size() == 1;
}

OpenSoT::SubTask::SubTaskMap OpenSoT::SubTask::SubTaskMap::range(unsigned int from, unsigned int to)
{
    std::list<unsigned int> rows;
    assert(from<=to && "from must be lower or equal to to");
    for(unsigned int row = from; row <= to; ++row)
        rows.push_back(row);
    return SubTaskMap(rows);
}

OpenSoT::SubTask::SubTaskMap::operator std::list<unsigned int>() const
{
    return this->getRowsList();
}

OpenSoT::SubTask::SubTaskMap::operator std::string() const
{
    std::stringstream subTaskIdSuffix;
    if(_rowsList.size() == 0)
        subTaskIdSuffix<<"empty";
    else
    {
        // for each chunk..
        for(ChunkList::const_iterator i = _contiguousChunks.begin();
            i != _contiguousChunks.end();
            ++i)
        {
            assert(i->size() != 0);

            if(i->size() == 1) {
                subTaskIdSuffix << i->front();
            } else {
                subTaskIdSuffix << i->front() << "-" << i->back();
            }

            if(i != _contiguousChunks.end() && i != --_contiguousChunks.end())
                subTaskIdSuffix << "+";
        }
    }
    subTaskIdSuffix.sync();

    return subTaskIdSuffix.str();
}

OpenSoT::SubTask::SubTaskMap OpenSoT::SubTask::SubTaskMap::operator+(const OpenSoT::SubTask::SubTaskMap &b) const
{
    std::list<unsigned int> rows = this->_rowsList;
    rows.insert(rows.end(), b.getRowsList().begin(), b.getRowsList().end());
    return SubTaskMap(rows);
}

OpenSoT::SubTask::SubTaskMap OpenSoT::SubTask::SubTaskMap::operator+(const unsigned int r) const
{
    std::list<unsigned int> rows = this->_rowsList;
    rows.push_back(r);
    return SubTaskMap(rows);
}

bool OpenSoT::SubTask::SubTaskMap::operator==(const OpenSoT::SubTask::SubTaskMap &b) const
{
    return this->_rowsList == b.getRowsList();
}

int OpenSoT::SubTask::SubTaskMap::size() const
{
    return this->_rowsList.size();
}







/***************
 *  SUBTASK    *
 ***************/

OpenSoT::SubTask::SubTask(OpenSoT::SubTask::TaskPtr taskPtr, const std::list<unsigned int> rowIndices) :
    Task(taskPtr->getTaskID() + std::string(SubTaskMap(rowIndices)),
         taskPtr->getXSize()),
    _subTaskMap(rowIndices),
    _taskPtr(taskPtr)
{
}

const yarp::sig::Matrix &OpenSoT::SubTask::getA()
{
    this->_A.resize(0, this->getXSize());

    for(SubTaskMap::ChunkList::const_iterator i = _subTaskMap.getChunks().begin();
        i != _subTaskMap.getChunks().end();
        ++i) {
        using namespace yarp::math;
        if(_taskPtr->getA().rows() > i->back())
            this->_A = pile(this->_A, _taskPtr->getA().submatrix(i->front(),
                                                                 i->back(),
                                                                 0, _x_size-1));
    }

    return this->_A;
}

const OpenSoT::HessianType OpenSoT::SubTask::getHessianAtype()
{
    OpenSoT::HessianType fatherHessianType = _taskPtr->getHessianAtype();

    if ( fatherHessianType == HST_IDENTITY)
        return HST_POSDEF;
    else return fatherHessianType;
}

const yarp::sig::Vector &OpenSoT::SubTask::getb()
{
    this->_b.resize(0);

    for(SubTaskMap::ChunkList::const_iterator i = _subTaskMap.getChunks().begin();
        i != _subTaskMap.getChunks().end();
        ++i) {
        using namespace yarp::math;
        if(_taskPtr->getb().size() > i->back())
            this->_b = cat(this->_b, _taskPtr->getb().subVector(i->front(),
                                                                i->back()));
    }

    return this->_b;
}

const yarp::sig::Matrix &OpenSoT::SubTask::getWeight() {
    this->_W.resize(this->getTaskSize(), this->getTaskSize());
    this->_W.zero();

    for(unsigned int r = 0; r < this->getTaskSize(); ++r)
        for(unsigned int c = 0; c < this->getTaskSize(); ++c)
            this->_W(r,c) = _taskPtr->getWeight()(this->_subTaskMap.getRowsVector()[r],
                                                  this->_subTaskMap.getRowsVector()[c]);

    return this->_W;
}

void OpenSoT::SubTask::setWeight(const yarp::sig::Matrix &W)
{
    assert(W.rows() == this->getTaskSize());
    assert(W.cols() == W.rows());

    this->_W = W;
    yarp::sig::Matrix fullW = _taskPtr->getWeight();
    for(unsigned int r = 0; r < this->getTaskSize(); ++r)
        for(unsigned int c = 0; c < this->getTaskSize(); ++c)
            fullW(this->_subTaskMap.getRowsVector()[r],
                  this->_subTaskMap.getRowsVector()[c]) = this->_W(r,c);

    _taskPtr->setWeight(fullW);
}

std::list<OpenSoT::SubTask::ConstraintPtr> &OpenSoT::SubTask::getConstraints()
{
    return _taskPtr->getConstraints();
}

const unsigned int OpenSoT::SubTask::getTaskSize() const
{
    unsigned int size = 0;
    for(SubTaskMap::ChunkList::const_iterator i = _subTaskMap.getChunks().begin();
        i != _subTaskMap.getChunks().end();
        ++i)
    {
        using namespace yarp::math;
        if(_taskPtr->getTaskSize() > i->back()) {
            size += i->size();
        }
    }
    return size;
}

void OpenSoT::SubTask::_update(const yarp::sig::Vector &x)
{
    _taskPtr->update(x);
}

std::vector<bool> OpenSoT::SubTask::getActiveJointsMask()
{
    return _taskPtr->getActiveJointsMask();
}

bool OpenSoT::SubTask::setActiveJointsMask(const std::vector<bool> &active_joints_mask)
{
    return _taskPtr->setActiveJointsMask(active_joints_mask);
}

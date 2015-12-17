#include <OpenSoT/utils/Indices.h>

std::list<unsigned int>::iterator OpenSoT::Indices::getNextAdjacentChunk(std::list<unsigned int>::iterator searchBegin)
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

void OpenSoT::Indices::generateChunks()
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

OpenSoT::Indices::Indices(unsigned int i)
{
    _rowsList.push_back(i);
    this->generateChunks();
}

OpenSoT::Indices::Indices(const std::list<unsigned int> &rowsList)
    : _rowsList(rowsList)
{
    this->generateChunks();
}

OpenSoT::Indices::Indices(const std::vector<unsigned int> &rowsVector)
{
    for(std::vector<unsigned int>::const_iterator it = rowsVector.begin();
        it != rowsVector.end();
        ++it)
    {
        _rowsList.push_back(*it);
    }

    this->generateChunks();
}

OpenSoT::Indices::Indices(const OpenSoT::Indices &subTaskMap)
    : _rowsList(subTaskMap.asList())
{
    this->generateChunks();
}

const OpenSoT::Indices::ChunkList& OpenSoT::Indices::getChunks() const
{
    return _contiguousChunks;
}

const std::list<unsigned int>& OpenSoT::Indices::asList() const
{
    return _rowsList;
}

const std::vector<unsigned int> &OpenSoT::Indices::asVector() const
{
    return _rowsVector;
}

bool OpenSoT::Indices::isContiguous() const
{
    return _contiguousChunks.size() == 1;
}

OpenSoT::Indices OpenSoT::Indices::range(unsigned int from, unsigned int to)
{
    std::list<unsigned int> rows;
    assert(from<=to && "from must be lower or equal to to");
    for(unsigned int row = from; row <= to; ++row)
        rows.push_back(row);
    return Indices(rows);
}

OpenSoT::Indices::operator std::list<unsigned int>() const
{
    return this->asList();
}

OpenSoT::Indices::operator std::string() const
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

OpenSoT::Indices OpenSoT::Indices::operator+(const OpenSoT::Indices &b) const
{
    std::list<unsigned int> rows = this->_rowsList;
    rows.insert(rows.end(), b.asList().begin(), b.asList().end());
    return Indices(rows);
}

OpenSoT::Indices OpenSoT::Indices::operator+(const unsigned int r) const
{
    std::list<unsigned int> rows = this->_rowsList;
    rows.push_back(r);
    return Indices(rows);
}

bool OpenSoT::Indices::operator==(const OpenSoT::Indices &b) const
{
    return this->_rowsList == b.asList();
}

int OpenSoT::Indices::size() const
{
    return this->_rowsList.size();
}

#include <OpenSoT/utils/logger/flushers/FakeFlusher.h>

OpenSoT::flushers::FakeFlusher::FakeFlusher(unsigned int size, unsigned int indicesOffset)
    : _size(size), _indicesOffset(indicesOffset)
{
    _descriptions.resize(_size, std::string());
}

OpenSoT::flushers::FakeFlusher::~FakeFlusher()
{

}

std::string OpenSoT::flushers::FakeFlusher::toString() const
{
    return std::string("");
}

int OpenSoT::flushers::FakeFlusher::getSize() const
{
    return _size;
}

OpenSoT::Indices OpenSoT::flushers::FakeFlusher::getIndices(int label) const
{
    return Indices::range(0, _size - 1);
}

unsigned int OpenSoT::flushers::FakeFlusher::getIndicesOffset() const
{
    return _indicesOffset;
}

void OpenSoT::flushers::FakeFlusher::setIndicesOffset(unsigned int offset)
{
    _indicesOffset = offset;
}


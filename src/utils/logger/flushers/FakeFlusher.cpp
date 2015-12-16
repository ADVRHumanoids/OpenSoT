#include <OpenSoT/utils/logger/flushers/FakeFlusher.h>

OpenSoT::flushers::FakeFlusher::FakeFlusher(unsigned int size)
    : _size(size)
{

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

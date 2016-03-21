#include <OpenSoT/utils/logger/plotters/Plotter.h>
#include <OpenSoT/utils/logger/flushers/ScriptFlusher.h>
#include <sstream>

using namespace OpenSoT::flushers;

ScriptFlusher::ScriptFlusher(std::string script_name,
                             std::list<unsigned int> cols,
                             unsigned int size,
                             unsigned int indicesOffset,
                             unsigned int rows)
    : FakeFlusher(size, indicesOffset), _script_name(script_name), _cols(cols)
{
    _descriptions.resize(_size, std::string());
}

ScriptFlusher::~ScriptFlusher()
{

}

std::string ScriptFlusher::toString() const
{
    using namespace OpenSoT::plotters;
    if(this->_rows < std::numeric_limits<unsigned int>::infinity())
        return std::string("import " + this->_script_name + "\n" +
                            "data = " + this->_script_name +
                           "(data, " + Plotter::getIndicesString(this->_cols) + ")");
    else
    {
        std::stringstream rows_str;
        rows_str << this->_rows;
        return std::string("import " + this->_script_name + "\n" +
                           "data = " + this->_script_name +
                           "(data, " + Plotter::getIndicesString(this->_cols) +
                            ", " + rows_str.str() + ")");
    }

}

/*
int ScriptFlusher::getSize() const
{
    return _size;
}

OpenSoT::Indices ScriptFlusher::getIndices(int label) const
{
    return Indices::range(0, _size - 1);
}
*/

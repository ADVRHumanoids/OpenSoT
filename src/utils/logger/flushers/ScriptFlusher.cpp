#include <OpenSoT/utils/logger/plotters/Plotter.h>
#include <OpenSoT/utils/logger/flushers/ScriptFlusher.h>
#include <sstream>

using namespace OpenSoT::flushers;

ScriptFlusher::ScriptFlusher(std::string script_name,
                             std::list<unsigned int> cols,
                             unsigned int size,
                             unsigned int indicesOffset,
                             std::list<boost::any> args)
    : FakeFlusher(size, indicesOffset), _script_name(script_name), _cols(cols),
      _args(args)
{
    _descriptions.resize(_size, std::string());
}

ScriptFlusher::~ScriptFlusher()
{

}

std::string ScriptFlusher::toString() const
{
    using namespace OpenSoT::plotters;

    std::stringstream args_str;
    for(std::list< boost::any >::const_iterator i = this->_args.begin();
        i != this->_args.end();
        ++i)
    {
        std::list<boost::any>::const_iterator next = i; ++next;

        if(const unsigned int* arg_uint = boost::any_cast<const unsigned int>(&(*i)))
            args_str << *arg_uint;
        else if(const int* arg_int = boost::any_cast<const int>(&(*i)))
            args_str << *arg_int;
        else if(const double* arg_double = boost::any_cast<const double>(&(*i)))
            args_str << *arg_double;
        else if(const std::string* arg_string = boost::any_cast<const std::string>(&(*i)))
            args_str << *arg_string;
        if(next != _args.end())
            args_str << ", ";
    }

    std::stringstream size_str;
    size_str << this->_size;

    std::string retval("import " + this->_script_name + "\n" +
                       "data = " + this->_script_name +
                       "(data, " + Plotter::getIndicesString(this->_cols) +
                       size_str.str());    // we add _size to check at the script side

    if(this->_args.size() > 0)
       retval += ", " + args_str.str() + ")";
    else
        retval += ")";

    return retval;

}

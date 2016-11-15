#include <OpenSoT/utils/logger/flushers/Flusher.h>
#include <OpenSoT/utils/logger/plotters/Plotter.h>
#include <stdexcept>

std::vector<std::string> OpenSoT::flushers::Flusher::getDescription(OpenSoT::Indices indices)
{
    std::vector<std::string> descriptions_v;
    std::vector<unsigned int> indices_v = indices.asVector();
    for(unsigned int i = 0; i < indices_v.size(); ++i)
    {
        if(indices_v[i] > _descriptions.size())
        {
            std::stringstream ss;
            ss << "Error: trying to access index " << indices_v[i]
               << " but _descriptions has " << _descriptions.size() << " elements";
            throw new std::range_error(ss.str());
        } else descriptions_v.push_back(_descriptions[indices_v[i]]);
    }
    return descriptions_v;
}

bool OpenSoT::flushers::Flusher::setDescription(const std::vector<std::string> descriptions,
                                                OpenSoT::Indices indices)
{
    std::vector<unsigned int> indices_v = indices.asVector();
    if(*std::max_element(indices_v.begin(), indices_v.end()) > this->getSize())
        return false;
    for(unsigned int i = 0; i < indices_v.size(); ++i)
        _descriptions[indices_v[i]]= descriptions[i];
    return true;
}

void OpenSoT::flushers::Flusher::updateSolution(const Eigen::VectorXd &q_dot)
{
    _q_dot = q_dot;
}

OpenSoT::plotters::Plottable OpenSoT::flushers::Flusher::i(OpenSoT::Indices indices)
{
    return std::make_pair(this, indices);
}

OpenSoT::plotters::Plottable OpenSoT::flushers::Flusher::i(int label)
{
    return this->i(this->getIndices(label));
}

OpenSoT::plotters::Plottable OpenSoT::flushers::Flusher::operator()(OpenSoT::Indices indices)
{
    return this->i(indices);
}

OpenSoT::plotters::Plottable OpenSoT::flushers::Flusher::operator()(int label)
{
    return this->i(this->getIndices(label));
}

bool OpenSoT::flushers::Flusher::setDescription(const std::vector<std::string> descriptions)
{
    _descriptions.clear();
    _descriptions.insert(_descriptions.end(),
                         descriptions.begin(),
                         descriptions.end());
    return true;
}

std::vector<std::string> OpenSoT::flushers::Flusher::getDescription()
{
    return _descriptions;
}

std::ostream &operator<<(std::ostream &out, const OpenSoT::flushers::Flusher::Ptr &flusher)
{
    if(flusher)
        out << flusher;
    return out;
}


std::ostream &operator<<(std::ostream &out, const OpenSoT::flushers::Flusher &flusher)
{
    out << flusher.toString();
    return out;
}

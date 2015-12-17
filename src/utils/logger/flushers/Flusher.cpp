#include <OpenSoT/utils/logger/flushers/Flusher.h>

std::list<std::string> OpenSoT::flushers::Flusher::getDescription(OpenSoT::Indices indices)
{
    std::list<std::string> descriptions_l;
    std::vector<unsigned int> indices_v = indices.asVector();
    for(unsigned int i = 0; i < indices_v.size(); ++i)
        descriptions_l.push_back(_descriptions[indices_v[i]]);
    return descriptions_l;
}

bool OpenSoT::flushers::Flusher::setDescription(const std::list<std::string> descriptions,
                                                OpenSoT::Indices indices)
{
    std::vector<std::string> descriptions_v;
    descriptions_v.insert(descriptions_v.end(),
                          descriptions.begin(),
                          descriptions.end());
    std::vector<unsigned int> indices_v = indices.asVector();
    if(*std::max_element(indices_v.begin(), indices_v.end()) > this->getSize())
        return false;
    for(unsigned int i = 0; i < indices_v.size(); ++i)
        _descriptions[indices_v[i]]= descriptions_v[i];
    return true;
}

void OpenSoT::flushers::Flusher::updateSolution(const yarp::sig::Vector &q_dot)
{
    _q_dot = q_dot;
}

std::pair<OpenSoT::flushers::Flusher *, OpenSoT::Indices> OpenSoT::flushers::Flusher::i(OpenSoT::Indices indices)
{
    return std::make_pair(this, indices);
}

std::pair<OpenSoT::flushers::Flusher *, OpenSoT::Indices> OpenSoT::flushers::Flusher::i(int label)
{
    return this->i(this->getIndices(label));
}

std::pair<OpenSoT::flushers::Flusher*, OpenSoT::Indices> OpenSoT::flushers::Flusher::operator()(OpenSoT::Indices indices)
{
    return this->i(indices);
}

std::pair<OpenSoT::flushers::Flusher *, OpenSoT::Indices> OpenSoT::flushers::Flusher::operator()(int label)
{
    return this->i(this->getIndices(label));
}

bool OpenSoT::flushers::Flusher::setDescription(const std::list<std::string> descriptions)
{
    _descriptions.clear();
    _descriptions.insert(_descriptions.end(),
                         descriptions.begin(),
                         descriptions.end());
}

std::list<std::string> OpenSoT::flushers::Flusher::getDescription()
{
    std::list<std::string> descriptions_l;
    descriptions_l.insert(descriptions_l.end(),
                          _descriptions.begin(),
                          _descriptions.end());
    return descriptions_l;
}

std::ostream &operator<<(std::ostream &out, const OpenSoT::flushers::Flusher::Ptr &flusher)
{
    if(flusher)
        out << flusher->toString();
}


std::ostream &operator<<(std::ostream &out, const OpenSoT::flushers::Flusher &flusher)
{
    out << flusher.toString();
}

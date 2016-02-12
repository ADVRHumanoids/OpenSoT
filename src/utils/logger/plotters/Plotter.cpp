#include <OpenSoT/utils/logger/plotters/Plotter.h>
#include <OpenSoT/utils/logger/L.h>

std::list<unsigned int> OpenSoT::plotters::Plotter::getGlobalIndicesList(std::list<OpenSoT::plotters::Plottable> data)
{
    std::list<unsigned int> indices;
    for(std::list<OpenSoT::plotters::Plottable>::iterator it = data.begin();
        it != data.end();
        ++it)
    {
        Indices globalIndices = _logger->getGlobalIndices(*it);
        std::list<unsigned int> globalIndices_l = globalIndices.asList();
        indices.insert(indices.end(),
                       globalIndices_l.begin(),
                       globalIndices_l.end());
    }
    return indices;
}

std::string OpenSoT::plotters::Plotter::getIndicesString(std::list<unsigned int> indices)
{
    std::stringstream indices_str;
    for(std::list<unsigned int>::iterator it = indices.begin();
        it != indices.end();
        ++it)
        indices_str << *it << ", ";

    return indices_str.str();
}

OpenSoT::plotters::Plotter::Plotter(OpenSoT::L *logger)
    : _logger(logger), _n_fig(0)
{

}

void OpenSoT::plotters::Plotter::subplot(const unsigned int nRows, const unsigned int nCols, const unsigned int nSubPlot)
{
    _commands << "subplot(" << nRows << ","
                            << nCols << ","
                            << nSubPlot<< ");" << std::endl;
}

void OpenSoT::plotters::Plotter::figure(const unsigned int width_in,
                                        const unsigned int height_in,
                                        const std::string title)
{
    _n_fig++;
    if(_logger->getFormat() == OpenSoT::L::FORMAT_PYTHON)
        _commands << "fig = figure('" << title << "', "
                  << "figsize=(" << width_in << "," << height_in << "));" << std::endl;
}

void OpenSoT::plotters::Plotter::xlabel(const std::string& label)
{
    _commands << "xlabel(r'" << label << "');" << std::endl;
}

void OpenSoT::plotters::Plotter::ylabel(const std::string& label)
{
    _commands << "ylabel(r'" << label << "');" << std::endl;
}

OpenSoT::plotters::Plottable OpenSoT::plotters::Plotter::norm(OpenSoT::plotters::Plottable data)
{
    std::list<OpenSoT::plotters::Plottable> data_l;
    data_l.push_back(data);
    return norm(data_l);
}

OpenSoT::plotters::Plottable OpenSoT::plotters::Plotter::norm(std::list<OpenSoT::plotters::Plottable> data)
{
    std::list<unsigned int> globalIndices = getGlobalIndicesList(data);
    unsigned int minFreeIndex = _logger->getDataSize();
    for(std::list<flushers::Flusher::Ptr>::iterator it = _fakeFlushers.begin();
        it != _fakeFlushers.end(); ++it)
        minFreeIndex += (*it)->getSize();

    OpenSoT::flushers::Flusher::Ptr flusher(
        new OpenSoT::flushers::FakeFlusher(1, minFreeIndex));
    _fakeFlushers.push_back(flusher);

    if(_logger->getFormat() == OpenSoT::L::FORMAT_PYTHON)
        _commands << "data = np.vstack((data.transpose(), (data[:,("
                  << getIndicesString(globalIndices) << ")]**2).sum(1))).transpose();" << std::endl;
    /// @TODO add description
    return flusher->i(OpenSoT::flushers::FakeFlusher::ALL);
}

OpenSoT::plotters::Plottable OpenSoT::plotters::Plotter::times(OpenSoT::plotters::Plottable data1, OpenSoT::plotters::Plottable data2)
{
    std::list<OpenSoT::plotters::Plottable> data1_l, data2_l;
    data1_l.push_back(data1); data2_l.push_back(data2);
    std::list<unsigned int> globalIndices1 = getGlobalIndicesList(data1_l);
    std::list<unsigned int> globalIndices2 = getGlobalIndicesList(data2_l);
    unsigned int minFreeIndex = _logger->getDataSize();
    for(std::list<flushers::Flusher::Ptr>::iterator it = _fakeFlushers.begin();
        it != _fakeFlushers.end(); ++it)
        minFreeIndex += (*it)->getSize();

    OpenSoT::flushers::Flusher::Ptr flusher(
        new OpenSoT::flushers::FakeFlusher(globalIndices1.size(), minFreeIndex));
    _fakeFlushers.push_back(flusher);

    if(_logger->getFormat() == OpenSoT::L::FORMAT_PYTHON)
        _commands << "data = np.hstack((data, data[:,("
                  << getIndicesString(globalIndices1) << ")]* data[:,("
                  << getIndicesString(globalIndices2) << ")]));" << std::endl;

    return flusher->i(OpenSoT::flushers::FakeFlusher::ALL);
}

OpenSoT::plotters::Plottable OpenSoT::plotters::Plotter::medfilt(OpenSoT::plotters::Plottable data, int kernel_size)
{
    std::list<OpenSoT::plotters::Plottable> data_l;
    data_l.push_back(data);
    return medfilt(data_l, kernel_size);
}

OpenSoT::plotters::Plottable OpenSoT::plotters::Plotter::medfilt(std::list<OpenSoT::plotters::Plottable> data, int kernel_size)
{
    std::list<unsigned int> globalIndices = getGlobalIndicesList(data);
    unsigned int minFreeIndex = _logger->getDataSize();
    for(std::list<flushers::Flusher::Ptr>::iterator it = _fakeFlushers.begin();
        it != _fakeFlushers.end(); ++it)
        minFreeIndex += (*it)->getSize();

    OpenSoT::flushers::Flusher::Ptr flusher(
        new OpenSoT::flushers::FakeFlusher(globalIndices.size(), minFreeIndex));
    _fakeFlushers.push_back(flusher);

    if(_logger->getFormat() == OpenSoT::L::FORMAT_PYTHON)
        _commands << "data = np.hstack((data, scipy.signal.medfilt(data[:,("
                  << getIndicesString(globalIndices) << ")],("
                  << kernel_size << ",1))));" << std::endl;
    /// @TODO add description
    return flusher->i(OpenSoT::flushers::FakeFlusher::ALL);
}

OpenSoT::plotters::Plottable OpenSoT::plotters::Plotter::minus(OpenSoT::plotters::Plottable data)
{
    std::list<OpenSoT::plotters::Plottable> data_l;
    data_l.push_back(data);
    return minus(data_l);
}

OpenSoT::plotters::Plottable OpenSoT::plotters::Plotter::minus(std::list<OpenSoT::plotters::Plottable> data)
{
    std::list<unsigned int> globalIndices = getGlobalIndicesList(data);
    unsigned int minFreeIndex = _logger->getDataSize();
    for(std::list<flushers::Flusher::Ptr>::iterator it = _fakeFlushers.begin();
        it != _fakeFlushers.end(); ++it)
        minFreeIndex += (*it)->getSize();

    OpenSoT::flushers::Flusher::Ptr flusher(
        new OpenSoT::flushers::FakeFlusher(globalIndices.size(), minFreeIndex));
    _fakeFlushers.push_back(flusher);

    if(_logger->getFormat() == OpenSoT::L::FORMAT_PYTHON)
        _commands << "data = np.hstack((data, -1.0*data[:,("
                  << getIndicesString(globalIndices) << ")]));" << std::endl;
    /// @TODO add description
    return flusher->i(OpenSoT::flushers::FakeFlusher::ALL);
}

void OpenSoT::plotters::Plotter::legend(const std::list<std::string> labels, std::string options)
{
    if(_logger->getFormat() == OpenSoT::L::FORMAT_PYTHON)
    {
        _commands << "legend(p,(";
        for(std::list<std::string>::const_iterator it = labels.begin();
            it != labels.end();
            ++it)
            _commands << "r'" << *it << "', ";
        _commands << "), "<< options << ");" << std::endl;
    }
}

void OpenSoT::plotters::Plotter::figlegend(const std::list<std::string> labels, std::string options)
{
    if(_logger->getFormat() == OpenSoT::L::FORMAT_PYTHON)
    {
        _commands << "figlegend(p,(";
        for(std::list<std::string>::const_iterator it = labels.begin();
            it != labels.end();
            ++it)
            _commands << "r'" << *it << "', ";
        _commands << "), "<< options << ");" << std::endl;
    }
}

void OpenSoT::plotters::Plotter::autoLegend(std::list<OpenSoT::plotters::Plottable> plottables,
                                            const std::string options,
                                            const bool fig)
{
    if(fig)
        figlegend(autoGenerateLegend(plottables), options);
    else
        legend(autoGenerateLegend(plottables), options);
}

void OpenSoT::plotters::Plotter::autoLegend(OpenSoT::plotters::Plottable data,
                                            const std::string options,
                                            const bool fig)
{
    std::list<OpenSoT::plotters::Plottable> data_l;
    data_l.push_back(data);
    autoLegend(data_l, options, fig);
}

std::list<std::string> OpenSoT::plotters::Plotter::autoGenerateLegend(std::list<OpenSoT::plotters::Plottable> plottables)
{
    std::list<std::string> labels;
    for(std::list<OpenSoT::plotters::Plottable>::const_iterator it_p = plottables.begin();
        it_p != plottables.end();
        ++it_p)
    {
        std::vector<std::string> plottableLabels = it_p->first->getDescription(it_p->second);
        labels.insert(labels.end(),
                      plottableLabels.begin(),
                      plottableLabels.end());
    }
    return labels;
}

void OpenSoT::plotters::Plotter::plot_t(std::list<OpenSoT::plotters::Plottable> data)
{
    std::list<unsigned int> globalIndices = getGlobalIndicesList(data);
    if(_logger->getFormat() == OpenSoT::L::FORMAT_PYTHON)
        _commands << "p = plot(data[:,0], data[:,("<< getIndicesString(globalIndices) << ")]);" << std::endl;
}

void OpenSoT::plotters::Plotter::plot_t(OpenSoT::plotters::Plottable data)
{
    std::list<OpenSoT::plotters::Plottable> data_l;
    data_l.push_back(data);
    plot_t(data_l);
}

void OpenSoT::plotters::Plotter::savefig()
{
    if(_logger->getFormat() == OpenSoT::L::FORMAT_PYTHON)
        _commands << "fig.savefig('"
                  << _logger->getName()
                  << "_fig_"
                  << _logger->isAppending()
                  << "_" << _n_fig << ".eps', format='eps', transparent=True);" << std::endl;
}

void OpenSoT::plotters::Plotter::show()
{
    _commands << std::endl;
    _commands << "show(block=True);" << std::endl;
}

void OpenSoT::plotters::Plotter::tight_layout()
{
    _commands << std::endl;
    _commands << "tight_layout();" << std::endl;
}

void OpenSoT::plotters::Plotter::title(const std::string &title)
{
    _commands << "title(r'" << title << "');" << std::endl;
}

std::string OpenSoT::plotters::Plotter::getCommands()
{
    _n_fig = 0;
    std::string commands = _commands.str();
    _commands.str(std::string());
    _commands.clear();
    _fakeFlushers.clear();
    return commands;
}

std::list<OpenSoT::plotters::Plottable> OpenSoT::plotters::operator+(const OpenSoT::plotters::Plottable &p1, const OpenSoT::plotters::Plottable &p2)
{
    std::list<OpenSoT::plotters::Plottable> plottablesList;
    plottablesList.push_back(p1);
    plottablesList.push_back(p2);
    return plottablesList;
}


std::list<OpenSoT::plotters::Plottable> OpenSoT::plotters::operator+(const OpenSoT::plotters::Plottable &p1, std::list<OpenSoT::plotters::Plottable> &pl2)
{
    std::list<OpenSoT::plotters::Plottable> plottablesList;
    plottablesList.push_back(p1);
    plottablesList.insert(plottablesList.end(),
                          pl2.begin(),
                          pl2.end());
    return plottablesList;
}


std::list<OpenSoT::plotters::Plottable> OpenSoT::plotters::operator+(std::list<OpenSoT::plotters::Plottable> &pl1, const OpenSoT::plotters::Plottable &p2)
{
    std::list<OpenSoT::plotters::Plottable> plottablesList;
    plottablesList.insert(plottablesList.end(),
                          pl1.begin(),
                          pl1.end());
    plottablesList.push_back(p2);
    return plottablesList;
}


std::list<OpenSoT::plotters::Plottable> OpenSoT::plotters::operator+(std::list<OpenSoT::plotters::Plottable> &pl1, std::list<OpenSoT::plotters::Plottable> &pl2)
{
    std::list<OpenSoT::plotters::Plottable> plottablesList;
    plottablesList.insert(plottablesList.end(),
                          pl1.begin(),
                          pl1.end());
    plottablesList.insert(plottablesList.end(),
                          pl2.begin(),
                          pl2.end());
    return plottablesList;
}

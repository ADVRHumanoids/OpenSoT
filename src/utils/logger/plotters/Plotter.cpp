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
        std::list<unsigned int> globalIndices_l = globalIndices.getRowsList();
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
    _commands << "xlabel('" << label << "');" << std::endl;
}

void OpenSoT::plotters::Plotter::ylabel(const std::string& label)
{
    _commands << "ylabel('" << label << "');" << std::endl;
}

OpenSoT::plotters::Plottable OpenSoT::plotters::Plotter::norm(std::list<OpenSoT::plotters::Plottable> data)
{
    std::list<unsigned int> globalIndices = getGlobalIndicesList(data);
    OpenSoT::flushers::Flusher::Ptr flusher(new OpenSoT::flushers::FakeFlusher(globalIndices.size()));
    _fakeFlushers.push_back(flusher);

    Indices plottableIndices = OpenSoT::Indices::range(_logger->getMaximumIndex(),
                                                       _logger->getMaximumIndex()+globalIndices.size()-1);
    _commands << "data = np.hstack((data.transpose(), data[:,(" << getIndicesString(globalIndices) << ")]**2).sum(1))).transpose()";

    return std::make_pair(flusher.get(),plottableIndices);
}

void OpenSoT::plotters::Plotter::legend(const std::list<std::string> labels)
{
    if(_logger->getFormat() == OpenSoT::L::FORMAT_PYTHON)
    {
        _commands << "legend(p,(";
        for(std::list<std::string>::const_iterator it = labels.begin();
            it != labels.end();
            ++it)
            _commands << "'" << *it << "', ";
        _commands << "));" << std::endl;
    }
}

void OpenSoT::plotters::Plotter::autoLegend(std::list<OpenSoT::plotters::Plottable> plottables)
{
    legend(autoGenerateLegend(plottables));
}

void OpenSoT::plotters::Plotter::autoLegend(OpenSoT::plotters::Plottable data)
{
    std::list<OpenSoT::plotters::Plottable> data_l;
    data_l.push_back(data);
    autoLegend(data_l);
}

std::list<std::string> OpenSoT::plotters::Plotter::autoGenerateLegend(std::list<OpenSoT::plotters::Plottable> plottables)
{
    std::list<std::string> labels;
    for(std::list<OpenSoT::plotters::Plottable>::const_iterator it_p = plottables.begin();
        it_p != plottables.end();
        ++it_p)
    {
        std::list<std::string> plottableLabels = it_p->first->getDescription(it_p->second);
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
        _commands << "p = plot(data[:,0], data[:,"<< getIndicesString(globalIndices) << "]);" << std::endl;
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
                  << "_" << _n_fig << "', format='eps', transparent=True);" << std::endl;
}

void OpenSoT::plotters::Plotter::title(const std::string &title)
{
    _commands << "title('" << title << "');" << std::endl;
}

std::string OpenSoT::plotters::Plotter::getCommands()
{
    _commands << std::endl;
    _commands << "show(block=True);" << std::endl;
    std::string commands = _commands.str();
    _commands.str("");
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

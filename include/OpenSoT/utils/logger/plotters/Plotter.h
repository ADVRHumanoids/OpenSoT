/*
 * Copyright (C) 2014 Walkman
 * Author: Alessio Rocchi, Enrico Mingo
 * email:  alessio.rocchi@iit.it, enrico.mingo@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU Lesser General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#ifndef __PLOTTER_H__
#define __PLOTTER_H__

#include <OpenSoT/Constraint.h>
#include <OpenSoT/Task.h>
#include <OpenSoT/utils/logger/flushers/all.h>
#include <OpenSoT/utils/logger/flushers/ConstraintFlusher.h>
#include <OpenSoT/utils/logger/flushers/DataFlusher.h>
#include <OpenSoT/utils/logger/flushers/FakeFlusher.h>
#include <OpenSoT/utils/logger/flushers/TaskFlusher.h>
#include <map>
#include <fstream>
#include <list>
#include <sstream>
#include <string>
#include <utility>

namespace OpenSoT
{
    // forward declaration of L
    class L;

    namespace plotters
    {
        /**
         * @brief Plottable is a pair of a pointer to Flusher and an Indices set.
         * If the pointer to flusher is NULL, the indices will be considered as global.
         * This will happen e.g. when the indices make reference to a time series, or to the solution vector,
         * or to the norm vector.
         */
        typedef std::pair<OpenSoT::flushers::Flusher*, OpenSoT::Indices> Plottable;

        /**
         * @brief The Plotter class takes care of generating
         */
        class Plotter
        {
        protected:
            /**
             * @brief _logger is a reference to a logger.
             * It is used to transform local Plottable indices in global indices,
             * which depend on the order in which the plottables are flushed.
             * The _logger also defines which is the plotting format
             */
            L* _logger;

            /**
             * @brief logCommands a script with commands to generate the plot.
             * The commands will depend on the format of the logger.
             */
            std::stringstream _commands;

            int _n_fig;

            std::list<flushers::Flusher::Ptr> _fakeFlushers;

            std::list<unsigned int> getGlobalIndicesList(std::list<OpenSoT::plotters::Plottable> data);

            std::string getIndicesString(std::list<unsigned int> indices);

        public:
            typedef boost::shared_ptr<Plotter> Ptr;

            Plotter(L* logger);

            void subplot(const unsigned int nRows,
                         const unsigned int nCols,
                         const unsigned int nSubPlot);

            void figure(const unsigned int width_in,
                        const unsigned int height_in,
                        const std::string title);

            void xlabel(const std::string& label);

            void ylabel(const std::string& label);

            Plottable norm2(Plottable data);

            /**
             * @brief norm creates a new plottable which is the norm2 of a given plottable
             * @param data a list of plottables
             * @return
             */
            Plottable norm2(std::list<Plottable> data);

            /**
             * @brief times creates a new plottable which is the element wise product between two plottables
             * @param data1 the first plottable
             * @param data2 the second plottable
             * @return
             */
            Plottable times(Plottable data1, Plottable data2);

            Plottable times(std::list<OpenSoT::plotters::Plottable> data, double scalar);
            Plottable times(Plottable data, double scalar);

            Plottable medfilt(Plottable data, int kernel_size);

            Plottable pow(std::list<OpenSoT::plotters::Plottable> data, double exponent);
            Plottable pow(OpenSoT::plotters::Plottable data, double exponent);

            /**
             * @brief sum
             * @param data
             * @param direction "0" for rows, "1" for columns
             * @return
             */
            Plottable sum(std::list<OpenSoT::plotters::Plottable> data, int direction);
            Plottable sum(OpenSoT::plotters::Plottable data, int direction);

            /**
             * @brief medfilt applies a median filter
             * @param data
             * @return
             */
            Plottable medfilt(std::list<Plottable> data, int kernel_size);

            Plottable minus(Plottable data);

            /**
             * @brief minus creates a new plottable obtained by negating the data of a given plottable
             * @param data a list of plottables
             * @return
             */
            Plottable minus(std::list<Plottable> data);

            /**
             * @brief legend builds a legend from a list of labels, and applies it to a plot or subplot
             * @param labels a list of string, one for each plot line
             * @param options additional options to add to the legend command
             */
            void legend(const std::list<std::string> labels, std::string options = "loc='best'");

            /**
             * @brief figlegend builds a legend from a list of labels, and applies it to the main figure
             * @param labels a list of string, one for each plot line
             * @param options additional options to add to the legend command
             */
            void figlegend(const std::list<std::string> labels, const std::string options = "loc='best'");

            /**
             * @brief autoLegend build a legend from the flusher data description
             * @param options additional options to add to the legend command
             */
            void autoLegend(std::list<Plottable> data, const std::string options = "loc='best'", const bool fig = false);

            /**
             * @brief autoLegend build a legend from the flusher data description
             * @param options additional options to add to the legend command
             * @param fig if true, uses figlegend instead of legend
             */
            void autoLegend(Plottable data, const std::string options = "loc='best'", const bool fig = false);

            /**
             * @brief autoGenerateLegend builds a list of labels from the flusher data description
             */
            std::list<std::string> autoGenerateLegend(std::list<Plottable> data);

            /**
             * @brief plot_t plots data against time
             * @param data a list of plottables
             */
            void plot_t(std::list<Plottable> data);

            /**
             * @brief plot_t plots data against time
             * @param data a list of plottables
             */
            void plot_t(Plottable data);

            /**
             * @brief savefig saves the last defined figure
             */
            void savefig();

            /**
             * @brief show displays plots on screen
             */
            void show();

            /**
             * @brief tight_layout adjusts labels of plots to have a nice fitting plot
             */
            void tight_layout();

            void title(const std::string& title);

            /*bool plot(const Plottable& index, std::list<Plottable> data);*/

            std::string getCommands();
        };

        std::list<Plottable> operator+(const Plottable& p1, const Plottable& p2);
        std::list<Plottable> operator+(const Plottable& p1, std::list<Plottable>& pl2);
        std::list<Plottable> operator+(std::list<Plottable>& pl1, const Plottable& p2);
        std::list<Plottable> operator+(std::list<Plottable>& pl1, std::list<Plottable>& pl2);
    }
}

#endif

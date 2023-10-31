import mat73
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator

back_end_solvers=["OSQP", "qpSWIFT", "qpOASES", "proxQP", "eiQuadProg"]
front_end_solvers=["ihqp", "nhqp", "hcod"]

front_end_marker=dict()
front_end_marker["ihqp"] = 'o'
front_end_marker["hcod"] = "s"
front_end_marker["nhqp"] = 'X'

back_end_color=dict()
back_end_color["OSQP"]="C0"
back_end_color["qpSWIFT"]="C1"
back_end_color["qpOASES"]="C2"
back_end_color["proxQP"]="C3"
back_end_color["eiQuadProg"]="C4"
back_end_color["hcod"]="C5"

def filter_key(hcod_dict, prefix):
    data_dict = dict()
    data_dict['number_of_success'] = hcod_dict[prefix+'number_of_success']
    data_dict['solver_time_ms_mean'] = hcod_dict[prefix+'solver_time_ms_mean']
    data_dict['iterations'] = hcod_dict[prefix+'iterations']
    data_dict['total_runs'] = hcod_dict['total_runs']
    return data_dict

def load(file_name, prefix):
    tmp = mat73.loadmat(file_name)
    return filter_key(tmp, prefix)

def compute_success_rate(data):
    for level in data:
        for fe in data[level]:
            for be in data[level][fe]:
                data[level][fe][be]["success_rate"] = 100. * (data[level][fe][be]['number_of_success']/data[level][fe][be]['total_runs']).round(decimals=1)
    return data

def compute_quantiles(arr):
    median = np.quantile(arr, 0.5)
    Q1 = np.quantile(arr, 0.25)
    Q3 = np.quantile(arr, 0.75)
    return median, Q1, Q3

def plot_markers(ax, median, Q1, Q3, success_rate, marker, color, alpha):
    ax.scatter(median, success_rate, marker=marker, c=color, alpha=alpha, linewidths=0., s=70., zorder=10)
    X = [Q1, Q3]
    Y = [success_rate, success_rate]
    plt.plot(X, Y, color, linewidth=1.2, zorder=10, alpha=alpha)
    ax.scatter(Q1, success_rate, marker=marker, c=color, alpha=alpha, linewidths=0., s=30., zorder=10)
    ax.scatter(Q3, success_rate, marker=marker, c=color, alpha=alpha, linewidths=0., s=30., zorder=10)

def subplot_HQP(level, index, data, print_phantom=False):
    ax = plt.subplot(4, 1, index)

    for level_ in data:
        if level_ == level:
            for fe in data[level_]:
                if fe != "hcod":
                    for be in data[level_][fe]:
                        median, Q1, Q3 = compute_quantiles(data[level_][fe][be]["solver_time_ms_mean"])
                        plot_markers(ax, median, Q1, Q3, data[level_][fe][be]["success_rate"], front_end_marker[fe],
                                     back_end_color[be], 1.)
        else:
            if print_phantom:
                for fe in data[level_]:
                    if fe != "hcod":
                        for be in data[level_][fe]:
                            median, Q1, Q3 = compute_quantiles(data[level_][fe][be]["solver_time_ms_mean"])
                            plot_markers(ax, median, Q1, Q3, data[level_][fe][be]["success_rate"], front_end_marker[fe],
                                         back_end_color[be], .1)

    ll = dict()
    ll["1 LEVEL"] = "Stack 1"
    ll["2 LEVELS"] = "Stack 2"
    ll["3 LEVELS"] = "Stack 3"
    ll["4 LEVELS"] = "Stack 4"
    ax.text(5.4, 80, ll[level], rotation=-90)

    ax.spines['right'].set_visible(False)
    ax.spines['top'].set_visible(False)
    ax.xaxis.set_major_locator(MultipleLocator(1.0))
    ax.set_ylim([50, 110])
    ax.set_xlim([-0.2, 5.4])
    ax.spines['left'].set_position(('data',-0.1))
    ax.spines['bottom'].set_position(('data', 55))
    ax.scatter(-0.1, 109.1, marker="^", c="black", alpha=1., linewidths=0., s=50., zorder=10)
    ax.scatter(5.39, 55, marker=">", c="black", alpha=1., linewidths=0., s=50., zorder=10)

    #plt.plot([5.35, 5.5], [55, 55], color="white", linewidth=1., zorder=10, alpha=1.)
    #ax.scatter(5.35, 55, marker="$\wr$", c="black", alpha=1., linewidths=0., s=50., zorder=20)
    #ax.scatter(5.38, 55, marker="$\wr$", c="black", alpha=1., linewidths=0., s=50., zorder=20)


def subplot_HCOD(level, index, data, print_phantom=False):
    ax = plt.subplot(4, 2, index)

    for level_ in data:
        if level_ == level:
            median, Q1, Q3 = compute_quantiles(data[level_]["hcod"]["hcod"]["solver_time_ms_mean"])
            plot_markers(ax, median, Q1, Q3, data[level_]["hcod"]["hcod"]["success_rate"], front_end_marker["hcod"],
                         back_end_color["hcod"], 1.)
        else:
            if print_phantom:
                median, Q1, Q3 = compute_quantiles(data[level_]["hcod"]["hcod"]["solver_time_ms_mean"])
                plot_markers(ax, median, Q1, Q3, data[level_]["hcod"]["hcod"]["success_rate"], front_end_marker["hcod"],
                             back_end_color["hcod"], .1)


    ax.spines['right'].set_visible(False)
    ax.spines['top'].set_visible(False)
    ax.spines['left'].set_visible(False)
    ax.xaxis.set_major_locator(MultipleLocator(1.0))
    ax.set_ylim([50, 110])
    ax.set_xlim([12.8, 22.3])
    ax.yaxis.set_tick_params(labelbottom=False)
    ax.set_yticks([])
    ax.spines['bottom'].set_position(('data', 55))
    ax.scatter(22.25, 55, marker=">", c="black", alpha=1., linewidths=0., s=50., zorder=10)

coman = dict()

levels_string = ["1 LEVEL", "2 LEVELS", "3 LEVELS", "4 LEVELS"]
levels = [1, 2, 3, 4]
lev = 1
for level in levels_string:
    level_string = "LEVEL"
    if lev > 1:
        level_string = "LEVELS"
    coman[level] = dict()
    coman[level]["hcod"] = dict()
    coman[level]["hcod"]["hcod"] = load("coman_ik_stats_" + str(lev) + "_" + level_string + "_hcod.mat", "HCOD")
    coman[level]["ihqp"] = dict()
    coman[level]["nhqp"] = dict()
    for be in back_end_solvers:
        coman[level]["ihqp"][be] = load("coman_ik_stats_" + str(lev) + "_" + level_string + "_iHQP.mat", be)
        coman[level]["nhqp"][be] = load("coman_ik_stats_" + str(lev) + "_" + level_string + "_nHQP.mat", be)
    lev = lev + 1


compute_success_rate(coman)


fig = plt.figure(figsize=(16, 9), dpi=150)

plt.rcParams.update({
    "text.usetex": True,
    "font.family": "Helvetica",
    "font.size": 15
})

subplot_HQP("1 LEVEL", 1, coman)
#subplot_HCOD("1 LEVEL", 2, coman)
subplot_HQP("2 LEVELS", 2, coman)
#subplot_HCOD("2 LEVELS", 4, coman)
subplot_HQP("3 LEVELS", 3, coman)
#subplot_HCOD("3 LEVELS", 6, coman)
subplot_HQP("4 LEVELS", 4, coman)
#subplot_HCOD("4 LEVELS", 8, coman)


plt.subplots_adjust(top=0.88, bottom=0.11, left=0.06, right=0.97, wspace=0.0)
plt.gcf().text(0.45, 0.05, "Solver Time [ms]", fontsize=20)
plt.gcf().text(0.35, 0.96, "35-DOFs Inverse Kinematics Solvers Comparison", fontsize=20)
plt.gcf().text(0.02, 0.4, "Success Rate [\%]", fontsize=20, rotation=90)

from matplotlib.lines import Line2D
legend_elements=[]
legend_labels=["iHQP", "nHQP"]#, "HCOD"]
i = 0
for fe in ["ihqp", "nhqp"]:#, "hcod"]:
    legend_elements.append(Line2D([0], [0], marker=front_end_marker[fe], color='w', label=legend_labels[i], markerfacecolor='black',
                        markersize=10))
    i = i + 1
legend_elements.append(Line2D([0], [0], marker="", color='w', label='', markerfacecolor='', markersize=15))

import matplotlib.patches as mpatches
for key in back_end_color:
    if key != "hcod":
        legend_elements.append(mpatches.Patch(color=back_end_color[key], label=key))

plt.legend(handles=legend_elements, bbox_to_anchor=(0.5, 4.9), loc='upper center', borderaxespad=0, fontsize=15, frameon=True, ncol=len(legend_elements))



#plt.subplot_tool()
plt.savefig("coman_plot_ik.pdf")
#plt.show()

import mat73
import matplotlib.pyplot as plt
import sys
import numpy as np

data_dicts = dict()
for i in range(1, len(sys.argv)):
    tmp = mat73.loadmat(sys.argv[i])
    ID = ''.join(chr(int(i)) for i in tmp["ID"])
    ID = ID.replace("_", " ")
    data_dicts[ID] = tmp

data_dict = dict()
data_dict['number_of_success'] = dict()
data_dict['solver_time_ms_mean'] = dict()
data_dict['iterations'] = dict()
data_dict['total_runs'] = dict()
for key in data_dicts:
    data_dict['number_of_success'][key] = data_dicts[key]['HCODnumber_of_success']
    data_dict['solver_time_ms_mean'][key] = data_dicts[key]['HCODsolver_time_ms_mean']
    data_dict['iterations'][key] = data_dicts[key]['HCODiterations']
    data_dict['total_runs'][key] = data_dicts[key]['total_runs']

print(data_dict)

#computes oredered labels of solver based on number_of_success/total runs
levels_label = []
success_ratio_list = []
data_dict['success_ratio'] = dict()
for key in data_dict['number_of_success']:
    data_dict['success_ratio'][key] = data_dict['number_of_success'][key]/data_dict['total_runs'][key]

for success_ratio in data_dict['success_ratio']:
    if len(levels_label) == 0:
        levels_label.append(success_ratio)
        success_ratio_list.append(data_dict['success_ratio'][success_ratio])
    else:
        if data_dict['success_ratio'][success_ratio] > success_ratio_list[-1]:
            levels_label.append(success_ratio)
            success_ratio_list.append(data_dict['success_ratio'][success_ratio])
        else:
            levels_label.insert(0, success_ratio)
            success_ratio_list.insert(0, data_dict['success_ratio'][success_ratio])
            for i in range(len(success_ratio_list)):
                if success_ratio_list[i] > success_ratio_list[i + 1]:
                    success_ratio_list[i], success_ratio_list[i + 1] = success_ratio_list[i + 1], success_ratio_list[i]
                    levels_label[i], levels_label[i + 1] = levels_label[i + 1], levels_label[i]
                else:
                    break

print(levels_label)
print(success_ratio_list)


plt.rcParams.update({
    "text.usetex": True,
    "font.family": "Helvetica",
    "font.size": 15
})

fig = plt.figure()
bpl = []
for label in levels_label:
    bpl.append(data_dict['solver_time_ms_mean'][label])

bplot = plt.boxplot(bpl,
                    notch=True,  # notch shape
                    vert=True,  # vertical box alignment
                    patch_artist=True,  # fill with color
                    showfliers=False, #remove outliers from plot
                    labels=(100.*np.array(success_ratio_list).round(decimals=1)),
                    medianprops = dict(color = "black"))  # will be used to label x-ticks


# fill with colors
cm = plt.get_cmap('tab20')
dict_colors = dict()
i = 0
for label in levels_label:
    dict_colors[label] = cm(i)
    i = i + 1
for patch, solver in zip(bplot['boxes'], levels_label):
    patch.set_facecolor(dict_colors[solver])
    patch.set_edgecolor(dict_colors[solver])

fig.get_axes()[0].yaxis.grid(True)
fig.get_axes()[0].xaxis.grid(True)
fig.get_axes()[0].set_xlabel('Success rate [$\%$]')
fig.get_axes()[0].set_ylabel('Solver time [ms]')
plt.ticklabel_format(axis="y", style="sci", scilimits=(0,0))

level_list = levels_label

leg = fig.get_axes()[0].legend(level_list)
hl_dict = {handle.get_label(): handle for handle in leg.legendHandles}
i = 0
for key in hl_dict:
    hl_dict[key].set_color(dict_colors[level_list[i]])
    i = i + 1


if len(sys.argv) == 2:
    plt.savefig(sys.argv[1] + ".pdf")
else:
    import os
    common = os.path.commonprefix([sys.argv[1], sys.argv[2]])
    print(common)
    plt.savefig(common + "hcod" + ".pdf")

plt.show()
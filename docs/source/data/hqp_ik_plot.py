import mat73
import matplotlib.pyplot as plt
import sys
import numpy as np

loaded_data_dict = mat73.loadmat(sys.argv[1])

print(loaded_data_dict)

data_dict = dict()
data_dict['number_of_success'] = {'OSQP': loaded_data_dict['OSQPnumber_of_success'],
                                  'eiQuadProg': loaded_data_dict['eiQuadPrognumber_of_success'],
                                  'proxQP': loaded_data_dict['proxQPnumber_of_success'],
                                  'qpOASES': loaded_data_dict['qpOASESnumber_of_success'],
                                  'qpSWIFT': loaded_data_dict['qpSWIFTnumber_of_success']}
data_dict['solver_time_ms_mean'] = {'OSQP': loaded_data_dict['OSQPsolver_time_ms_mean'],
                                  'eiQuadProg': loaded_data_dict['eiQuadProgsolver_time_ms_mean'],
                                  'proxQP': loaded_data_dict['proxQPsolver_time_ms_mean'],
                                  'qpOASES': loaded_data_dict['qpOASESsolver_time_ms_mean'],
                                  'qpSWIFT': loaded_data_dict['qpSWIFTsolver_time_ms_mean']}
data_dict['iterations'] = {'OSQP': loaded_data_dict['OSQPiterations'],
                                  'eiQuadProg': loaded_data_dict['eiQuadProgiterations'],
                                  'proxQP': loaded_data_dict['proxQPiterations'],
                                  'qpOASES': loaded_data_dict['qpOASESiterations'],
                                  'qpSWIFT': loaded_data_dict['qpSWIFTiterations']}
total_runs = loaded_data_dict['total_runs']

#computes oredered labels of solver based on number_of_success
solver_label = []
number_of_success_list = []
for number_of_success in data_dict['number_of_success']:
    if len(solver_label) == 0:
        solver_label.append(number_of_success)
        number_of_success_list.append(data_dict['number_of_success'][number_of_success])
    else:
        if data_dict['number_of_success'][number_of_success] > number_of_success_list[-1]:
            solver_label.append(number_of_success)
            number_of_success_list.append(data_dict['number_of_success'][number_of_success])
        else:
            solver_label.insert(0, number_of_success)
            number_of_success_list.insert(0, data_dict['number_of_success'][number_of_success])
            for i in range(len(number_of_success_list)):
                if number_of_success_list[i] > number_of_success_list[i+1]:
                    number_of_success_list[i], number_of_success_list[i+1] = number_of_success_list[i+1], number_of_success_list[i]
                    solver_label[i], solver_label[i + 1] = solver_label[i + 1], solver_label[i]
                else:
                    break

print(solver_label)
print(number_of_success_list)

plt.rcParams.update({
    "text.usetex": True,
    "font.family": "Helvetica",
    "font.size": 15
})

fig = plt.figure()
bplot = plt.boxplot([data_dict['solver_time_ms_mean'][solver_label[0]],
                     data_dict['solver_time_ms_mean'][solver_label[1]],
                     data_dict['solver_time_ms_mean'][solver_label[2]],
                     data_dict['solver_time_ms_mean'][solver_label[3]],
                     data_dict['solver_time_ms_mean'][solver_label[4]]],
                    notch=True,  # notch shape
                    vert=True,  # vertical box alignment
                    patch_artist=True,  # fill with color
                    showfliers=False, #remove outliers from plot
                    labels=(100.*np.array(number_of_success_list)/total_runs).round(decimals=1),
                    medianprops = dict(color = "black"))  # will be used to label x-ticks

# fill with colors
dict_colors = {'OSQP': 'tab:blue', 'eiQuadProg': 'tab:red', 'proxQP': 'tab:green', 'qpOASES': 'tab:orange', 'qpSWIFT': 'tab:purple'}
for patch, solver in zip(bplot['boxes'], solver_label):
    patch.set_facecolor(dict_colors[solver])
    patch.set_edgecolor(dict_colors[solver])

fig.get_axes()[0].yaxis.grid(True)
fig.get_axes()[0].xaxis.grid(True)
fig.get_axes()[0].set_xlabel('Success rate [$\%$]')
fig.get_axes()[0].set_ylabel('Solver time [ms]')
plt.ticklabel_format(axis="y", style="sci", scilimits=(0,0))



solver_list = ['OSQP', 'eiQuadProg', 'proxQP', 'qpOASES', 'qpSWIFT']

leg = fig.get_axes()[0].legend(solver_list)
hl_dict = {handle.get_label(): handle for handle in leg.legendHandles}
i = 0
for key in hl_dict:
    hl_dict[key].set_color(dict_colors[solver_list[i]])
    i = i + 1


plt.savefig(sys.argv[1] + ".pdf")

plt.show()
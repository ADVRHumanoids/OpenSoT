load_data;

t_SCAFoI_i = 3;
t_capsules_i = 4;
t_i = 2;

figure('Position',[0 0 800 600]);
plot(log_t(:,t_i),[log_t(:,t_SCAFoI_i),log_t(:,t_capsules_i)]);
xlabel('t [s]'); ylabel('t [s]');
legend('SCAFoI computation time', 'narrow-phase computation time');


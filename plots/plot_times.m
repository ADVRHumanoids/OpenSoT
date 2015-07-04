load_data;

t_SCAFoI_i = 3;
t_capsules_i = 4;
t_i = 2;
t_SCAFoI_active_i = [6, 8, 10, 12, 14, 16, 18];

figure('Position',[0 0 800 600]);

subplot(2,1,1);
plot(log_t(:,t_i),[log_t(:,t_SCAFoI_i),log_t(:,t_capsules_i)]);
xlabel('t [s]'); ylabel('t [s]');
legend('SCAFoI computation time', 'narrow-phase computation time');
title('computation times');

subplot(2,1,2);
plot(log_t(:,t_i),sum(log_t(:,t_SCAFoI_active_i),2));
xlabel('t [s]'); ylabel('no. of active SCAFoIs');
legend('number of active SCAFoI (out of 7)');
title('activation scheme');


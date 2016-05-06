t = dT*[1:1:length(tau_l_leg)]';

tau_max = tau_legs_max(:,1);
tau_min = -tau_max;

subplot(3,2,1);
plot(t,filtered_ft_left_right(:,3),'b'); 
title('Filtered measured forces z left leg');
xlabel('[sec]')
ylabel('[N]')

subplot(3,2,3);
plot(t, tau_l_leg); hold on; plot(t, tau_max, 'k'); hold on; plot(t, tau_min,'k');
title('Left leg torques');
xlabel('[sec]')
ylabel('[Nm]')

subplot(3,2,5);
plot(t, computed_dq_l_leg);
title('Left leg computed velocities');
xlabel('[sec]')
ylabel('[rad/sec]')

subplot(3,2,2);
plot(t,filtered_ft_left_right(:,9),'b'); 
title('Filtered measured forces z right leg');
xlabel('[sec]')
ylabel('[N]')

subplot(3,2,4);
plot(t, tau_r_leg); hold on; plot(t, tau_max, 'k'); hold on; plot(t, tau_min,'k');
title('Left leg torques');
xlabel('[sec]')
ylabel('[Nm]')

subplot(3,2,6);
plot(t, computed_dq_r_leg);
title('Right leg computed velocities');
xlabel('[sec]')
ylabel('[rad/sec]')

%%
figure();
plot(t,cartesian_error)
title('CoM Cartesian error');
xlabel('[sec]')
ylabel('[m]')
legend('x','y')
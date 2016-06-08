t = (0:1:length(tau_left_leg0)-1)*dT;

%%
figure();
subplot(3,2,1)
plot(t, cartesian_error0(:,1), 'b'); hold on; plot(t, cartesian_error1(:,1), 'r'); 
title('Cartesian error x');
xlabel('[sec]')
ylabel('[m]')

subplot(3,2,3)
plot(t, cartesian_error0(:,2), 'b'); hold on; plot(t, cartesian_error1(:,2), 'r');
title('Cartesian error y');
xlabel('[sec]')
ylabel('[m]')

subplot(3,2,5)
plot(t, cartesian_error0(:,3), 'b'); hold on; plot(t, cartesian_error1(:,3), 'r');
title('Cartesian error z');
xlabel('[sec]')
ylabel('[m]')

subplot(3,2,2)
plot(t, cartesian_error0(:,4), 'b'); hold on; plot(t, cartesian_error1(:,4), 'r');
title('Cartesian error Roll');
xlabel('[sec]')
ylabel('[rad]')

subplot(3,2,4)
plot(t, cartesian_error0(:,5), 'b'); hold on; plot(t, cartesian_error1(:,5), 'r');
title('Cartesian error Pitch');
xlabel('[sec]')
ylabel('[rad]')

subplot(3,2,6)
plot(t, cartesian_error0(:,6), 'b'); hold on; plot(t, cartesian_error1(:,6), 'r');
title('Cartesian error Yaw');
xlabel('[sec]')
ylabel('[rad]')

%%
tau_max_left_arm = repmat(tau_max_left_arm0,size(tau_left_arm0,1),1);
tau_max_right_arm = repmat(tau_max_right_arm0,size(tau_right_arm0,1),1);
tau_max_torso = repmat(tau_max_torso0,size(tau_torso0,1),1);
tau_max_left_leg = repmat(tau_max_left_leg0,size(tau_left_leg0,1),1);
tau_max_right_leg = repmat(tau_max_right_leg0,size(tau_right_leg0,1),1);

figure();
subplot(3,2,1)
plot(t, tau_left_leg0(:,1), 'b'); hold on; plot(t, tau_left_leg1(:,1),'r'); 
title('Left Leg joint 0 torque');
xlabel('[sec]')
ylabel('[Nm]')

subplot(3,2,2)
plot(t, tau_left_leg0(:,2), 'b'); hold on; plot(t, tau_left_leg1(:,2),'r'); 
title('Left Leg joint 1 torque');
xlabel('[sec]')
ylabel('[Nm]')

subplot(3,2,3)
plot(t, tau_left_leg0(:,3), 'b'); hold on; plot(t, tau_left_leg1(:,3),'r'); 
title('Left Leg joint 2 torque');
xlabel('[sec]')
ylabel('[Nm]')

subplot(3,2,4)
plot(t, tau_left_leg0(:,4), 'b'); hold on; plot(t, tau_max_left_leg(:,4),'k'); hold on; plot(t, -tau_max_left_leg(:,4),'k'); hold on; plot(t, tau_left_leg1(:,4),'r'); 
title('Left Leg joint 3 torque');
xlabel('[sec]')
ylabel('[Nm]')

subplot(3,2,5)
plot(t, tau_left_leg0(:,5), 'b');  hold on; plot(t, tau_left_leg1(:,5),'r'); 
title('Left Leg joint 4 torque');
xlabel('[sec]')
ylabel('[Nm]')

subplot(3,2,6)
plot(t, tau_left_leg0(:,6), 'b'); hold on; plot(t, tau_left_leg1(:,6),'r'); 
title('Left Leg joint 5 torque');
xlabel('[sec]')
ylabel('[Nm]')

%%
figure();
subplot(3,2,1)
plot(t, computed_vel_left_leg0(:,1), 'b'); hold on; plot(t, computed_vel_left_leg1(:,1),'r'); 
title('Left Leg joint 0 computed velocities');
xlabel('[sec]')
ylabel('[rad/sec]')

subplot(3,2,2)
plot(t, computed_vel_left_leg0(:,2), 'b'); hold on; plot(t, computed_vel_left_leg1(:,2),'r'); 
title('Left Leg joint 1 computed velocities');
xlabel('[sec]')
ylabel('[rad/sec]')

subplot(3,2,3)
plot(t, computed_vel_left_leg0(:,3), 'b'); hold on; plot(t, computed_vel_left_leg1(:,3),'r'); 
title('Left Leg joint 2 computed velocities');
xlabel('[sec]')
ylabel('[rad/sec]')

subplot(3,2,4)
plot(t, computed_vel_left_leg0(:,4), 'b'); hold on; plot(t, computed_vel_left_leg1(:,4),'r'); 
title('Left Leg joint 3 computed velocities');
xlabel('[sec]')
ylabel('[rad/sec]')

subplot(3,2,5)
plot(t, computed_vel_left_leg0(:,5), 'b');  hold on; plot(t, computed_vel_left_leg1(:,5),'r'); 
title('Left Leg joint 4 computed velocities');
xlabel('[sec]')
ylabel('[rad/sec]')

subplot(3,2,6)
plot(t, computed_vel_left_leg0(:,6), 'b'); hold on; plot(t, computed_vel_left_leg1(:,6),'r'); 
title('Left Leg joint 5 computed velocities');
xlabel('[sec]')
ylabel('[rad/sec]')

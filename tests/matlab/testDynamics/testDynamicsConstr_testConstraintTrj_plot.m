
t = dT*[0:1:length(cartesian_error_0)-1]';

figure();
subplot(3,2,1);
plot(t,cartesian_error_0(:,1),'b'); hold on; plot(t,cartesian_error_1(:,1),'r');
title('Cartesian error x');
xlabel('[sec]')
ylabel('[m]')

subplot(3,2,3);
plot(t,cartesian_error_0(:,2),'b'); hold on; plot(t,cartesian_error_1(:,2),'r');
title('Cartesian error y');
xlabel('[sec]')
ylabel('[m]')

subplot(3,2,5);
plot(t,cartesian_error_0(:,3),'b'); hold on; plot(t,cartesian_error_1(:,3),'r');
title('Cartesian error z');
xlabel('[sec]')
ylabel('[m]')

subplot(3,2,2);
plot(t,cartesian_error_0(:,4),'b'); hold on; plot(t,cartesian_error_1(:,4),'r');
title('Cartesian error Roll');
xlabel('[sec]')
ylabel('[rad]')

subplot(3,2,4);
plot(t,cartesian_error_0(:,5),'b'); hold on; plot(t,cartesian_error_1(:,5),'r');
title('Cartesian error Pitch');
xlabel('[sec]')
ylabel('[rad]')

subplot(3,2,6);
plot(t,cartesian_error_0(:,6),'b'); hold on; plot(t,cartesian_error_1(:,6),'r');
title('Cartesian error Yaw');
xlabel('[sec]')
ylabel('[rad]')
%%
figure();
subplot(5,2,1);
plot(t,tau_0(:,1),'b'); hold on; plot(t,tau_1(:,1),'r'); hold on; plot(t, torque_limits1(:,1),'k'); hold on; plot(t, -torque_limits1(:,1),'k');
title('Torso joint 0 torque');
xlabel('[sec]')
ylabel('[Nm]')

subplot(5,2,3);
plot(t,tau_0(:,2),'b'); hold on; plot(t,tau_1(:,2),'r'); hold on; plot(t, torque_limits1(:,2),'k'); hold on; plot(t, -torque_limits1(:,2),'k');
title('Torso joint 1 torque');
xlabel('[sec]')
ylabel('[Nm]')

subplot(5,2,5);
plot(t,tau_0(:,3),'b'); hold on; plot(t,tau_1(:,3),'r');hold on; plot(t, torque_limits1(:,3),'k'); hold on; plot(t, -torque_limits1(:,3),'k');
title('Torso joint 2 torque');
xlabel('[sec]')
ylabel('[Nm]')

subplot(5,2,7);
plot(t,tau_0(:,4),'b'); hold on; plot(t,tau_1(:,4),'r');hold on; plot(t, torque_limits1(:,4),'k'); hold on; plot(t, -torque_limits1(:,4),'k');
title('Left Arm joint 0 torque');
xlabel('[sec]')
ylabel('[Nm]')

subplot(5,2,9);
plot(t,tau_0(:,5),'b'); hold on; plot(t,tau_1(:,5),'r');hold on; plot(t, torque_limits1(:,5),'k'); hold on; plot(t, -torque_limits1(:,5),'k');
title('Left Arm joint 1 torque');
xlabel('[sec]')
ylabel('[Nm]')

subplot(5,2,2);
plot(t,tau_0(:,6),'b'); hold on; plot(t,tau_1(:,6),'r');hold on; plot(t, torque_limits1(:,6),'k'); hold on; plot(t, -torque_limits1(:,6),'k');
title('Left Arm joint 2 torque');
xlabel('[sec]')
ylabel('[Nm]')

subplot(5,2,4);
plot(t,tau_0(:,7),'b'); hold on; plot(t,tau_1(:,7),'r');hold on; plot(t, torque_limits1(:,7),'k'); hold on; plot(t, -torque_limits1(:,7),'k');
title('Left Arm joint 3 torque');
xlabel('[sec]')
ylabel('[Nm]')

subplot(5,2,6);
plot(t,tau_0(:,8),'b'); hold on; plot(t,tau_1(:,8),'r');hold on; plot(t, torque_limits1(:,8),'k'); hold on; plot(t, -torque_limits1(:,8),'k');
title('Left Arm joint 4 torque');
xlabel('[sec]')
ylabel('[Nm]')

subplot(5,2,8);
plot(t,tau_0(:,9),'b'); hold on; plot(t,tau_1(:,9),'r');hold on; plot(t, torque_limits1(:,9),'k'); hold on; plot(t, -torque_limits1(:,9),'k');
title('Left Arm joint 5 torque');
xlabel('[sec]')
ylabel('[Nm]')

subplot(5,2,10);
plot(t,tau_0(:,10),'b'); hold on; plot(t,tau_1(:,10),'r');hold on; plot(t, torque_limits1(:,10),'k'); hold on; plot(t, -torque_limits1(:,10),'k');
title('Left Arm joint 6 torque');
xlabel('[sec]')
ylabel('[Nm]')

%%
figure();
subplot(5,2,1);
plot(t,computed_vel_0(:,1),'b'); hold on; plot(t,computed_vel_1(:,1),'r');
title('Torso joint 0 computed vel');
xlabel('[sec]')
ylabel('[rad/sec]')

subplot(5,2,3);
plot(t,computed_vel_0(:,2),'b'); hold on; plot(t,computed_vel_1(:,2),'r');
title('Torso joint 1 computed vel');
xlabel('[sec]')
ylabel('[rad/sec]')

subplot(5,2,5);
plot(t,computed_vel_0(:,3),'b'); hold on; plot(t,computed_vel_1(:,3),'r');
title('Torso joint 2 computed vel');
xlabel('[sec]')
ylabel('[rad/sec]')

subplot(5,2,7);
plot(t,computed_vel_0(:,4),'b'); hold on; plot(t,computed_vel_1(:,4),'r');
title('Left Arm joint 0 computed vel');
xlabel('[sec]')
ylabel('[rad/sec]')

subplot(5,2,9);
plot(t,computed_vel_0(:,5),'b'); hold on; plot(t,computed_vel_1(:,5),'r');
title('Left Arm joint 1 computed vel')
xlabel('[sec]')
ylabel('[rad/sec]')

subplot(5,2,2);
plot(t,computed_vel_0(:,6),'b'); hold on; plot(t,computed_vel_1(:,6),'r');
title('Left Arm joint 2 computed vel');
xlabel('[sec]')
ylabel('[rad/sec]')


subplot(5,2,4);
plot(t,computed_vel_0(:,7),'b'); hold on; plot(t,computed_vel_1(:,7),'r');
title('Left Arm joint 3 computed vel');
xlabel('[sec]')
ylabel('[rad/sec]')

subplot(5,2,6);
plot(t,computed_vel_0(:,8),'b'); hold on; plot(t,computed_vel_1(:,8),'r');
title('Left Arm joint 4 computed vel');
xlabel('[sec]')
ylabel('[rad/sec]')

subplot(5,2,8);
plot(t,computed_vel_0(:,9),'b'); hold on; plot(t,computed_vel_1(:,9),'r');
title('Left Arm joint 5 computed vel');
xlabel('[sec]')
ylabel('[rad/sec]')

subplot(5,2,10);
plot(t,computed_vel_0(:,10),'b'); hold on; plot(t,computed_vel_1(:,10),'r');
title('Left Arm joint 6 computed vel');
xlabel('[sec]')
ylabel('[rad/sec]')

%%
figure();
subplot(1,2,1);
plot(t(1:1:length(T_)),T_(:,1),'k');
title('Desired Cartesian Trj x');
xlabel('[sec]')
ylabel('[m]')

subplot(1,2,2);
plot(t(1:1:length(v_)),v_(:,1),'k');
title('Desired Cartesian Vel x');
xlabel('[sec]')
ylabel('[m/sec]')
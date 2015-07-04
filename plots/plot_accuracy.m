load_data;

t_SCAFoI_i = 3;
t_capsules_i = 4;
t_i = 2;
t_SCAFoI_distance_i = [5, 7, 9, 11, 13, 15, 17];
t_SCAFoI_active_i = [6, 8, 10, 12, 14, 16, 18];
t_SCAFoI_names = {'L_R_arm','L_Arm_Torso','R_Arm_Torso','L_Arm_L_Leg','R_Arm_R_Leg','L_Arm_R_Leg','R_Arm_L_Leg'}
d_threshold_lower = 0.15;

figure('Position',[0 0 1024 768]);
for i=1:size(t_SCAFoI_distance_i,2)
    subplot(4,2,i);
    plot(   log_t(:,t_i), [log_t(:,t_SCAFoI_distance_i(i)), d_threshold_lower*log_t(:,t_SCAFoI_active_i(i))]);
    hold on;
    plot(   log_t(:,t_i), d_threshold_lower*ones(1,size(log_t,1)),'r-');
    xlabel('t [s]'); ylabel('t [s]');
    title(t_SCAFoI_names(i));
end
legend('SCAFoI distance', 'is SCAFoI active?', 'activation threshold', 'Position', 'OutsideEast');

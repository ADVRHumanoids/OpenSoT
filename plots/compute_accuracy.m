load_data;

t_SCAFoI_i = 3;
t_capsules_i = 4;
t_i = 2;
t_SCAFoI_distance_i = [5, 7, 9, 11, 13, 15, 17];
t_SCAFoI_active_i = [6, 8, 10, 12, 14, 16, 18];
t_SCAFoI_names = {'L_R_arm','L_Arm_Torso','R_Arm_Torso','L_Arm_L_Leg','R_Arm_R_Leg','L_Arm_R_Leg','R_Arm_L_Leg'}
d_threshold_lower = 0.15;
accuracy = zeros(1,size(t_SCAFoI_distance_i,2));

for i=1:size(t_SCAFoI_distance_i,2)
    accuracy(i) = sum((log_t(:,t_SCAFoI_distance_i(i)) < d_threshold_lower) .* log_t(:,t_SCAFoI_active_i(i))) / sum(log_t(:,t_SCAFoI_distance_i(i)) < d_threshold_lower);
end
accuracy

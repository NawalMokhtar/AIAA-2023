function [E_max_M2, P_max_M2, Battery_results_M2, time_M2_min,TO_dist_M2,n_laps_M2,E_max_M3, P_max_M3, Battery_results_M3,time_M3_min,TO_dist_M3,Score_M2,...
     Score_M3,Overall_score,cap,Payload_weight_M2,v_avrg2,v_avrg3] = Mission_model_function(wing,cl_max,n,Cd0_wing,W_S,CL_max, M2,M3,pd,L_antenna) % bonus?

Payload_weight_M2 = pd;           
M2.P1 = -1 * M2.C1_M2;                        
M2.P2 = -1 * M2.C2_M2;                        
M2.P1_throttle = -1 * M2.C1_M2_Th; 
M2.P2_throttle = -1 * M2.C2_M2_Th;

M3.P1_max = -1 * M3.C1_M3_100;                     
M3.P2_max = -1 * M2.C2_M3_100;                    
M3.P1_throttle = -1 * M3.C1_M3_Th; 
M3.P2_throttle = -1 * M3.C2_M3_Th;

% MTOW_3 = M3.MTOW_3;
% time_M3_min = 0;
% total_cap_consumed_M3 = 0;

%% functions for missions :

[E_max_M3, P_max_M3, Battery_results_M3, lap_times_M3, time_M3_min, T_overall3, V_overall3, v_avrg3, TO_dist_M3, x_overall_M3, y_overall_M3, z_overall_M3] = M3_2023(wing,cl_max,n,Cd0_wing,CL_max,M3,M2);

[E_max_M2, P_max_M2, Battery_results_M2,n_laps_M2,lap_times_M2,time_M2_min,T_overall2,V_overall2,v_avrg2,TO_dist_M2,x_overall_M2,y_overall_M2,z_overall_M2] = M2_2023(wing,cl_max,n,Cd0_wing,CL_max,M2,M3);

%% SCORE
% max_M2 = score.max_score_M2 ;
% max_M3 = score.max_score_M3 ;
% % report_score = score.report_score*0.85+score.proposal_score*0.15 ;         %2025

%Score_M2 = 1 + payload_M2*N_laps./max_M2
%Score_M3 = 2 + (L_antenna/time_M3)./max_M3
Overall_score = 1;
cap = 67; %% To be changed
%% Remaining capacity

% remaining_cap_M2_mAh = M2.bat_cap-cap_consumed_M2;
% remaining_cap_M3_mAh = M3.bat_cap-cap_consumed_M3;
% remaining_cap_M2_percent = ((remaining_cap_M2_mAh)/(M2.bat_cap)) * 100;      % percentage
% remaining_cap_M3_percent = ((remaining_cap_M3_mAh)/(M3.bat_cap)) * 100;
% 
% cap.remaining_cap_M2_mAh = remaining_cap_M2_mAh;
% cap.remaining_cap_M3_mAh = remaining_cap_M3_mAh;
% cap.remaining_cap_M2_percent = remaining_cap_M2_percent;
% cap.remaining_cap_M3_percent = remaining_cap_M3_percent;

%% Display result

% Result_mission_model = table(time_M2_min,TO_dist_M2,Score_M2,...
%     no_of_laps_M3,fuel_tanks_weight_M2,n_tanks,time_M3_min,distance_takeoff_M3,Score_M3,Overall_score);  %2024
% 
%       % disp("  ")
%       % disp("Mission model results:   ")
%       % disp("  ")
%       % disp(Result_mission_model)
%       % disp("  ")
% 
%      Result_capacity = table(remaining_cap_M2_percent,remaining_cap_M2_mAh,...
%          remaining_cap_M3_percent,remaining_cap_M3_mAh);
% 
%       % disp("capacity results:   ")
%       % disp("  ")
%       % disp(Result_capacity)
% 
% 
      %%
      
      
% [no_of_laps_M3,time_M3_min,distance_takeoff_M3,cap_consumed_M3]= Mission_3(wing,cl_max,n,M3,no_of_boxes);
% m=1;
% %distance_takeoff_M3(m-1)=0;
% while_condition = no_of_boxes;
% distance_takeoff_M3_max = 0;
% 
% figure
% while m <= while_condition
% 
%      subplot(2,ceil(no_of_boxes/2),m)
%     [distance_takeoff_M3(m),time_lap_M3(m),cap_consumed_M3(m)]= Mission_3(wing,cl_max,n,M3,MTOW_3,m);
% 
%     MTOW_3 = MTOW_3 - box.weight;
%     time_M3_min = time_M3_min + time_lap_M3(m);
%     total_cap_consumed_M3 = total_cap_consumed_M3 + cap_consumed_M3(m);
% 
%     m = m+1;
% end
% 
% distance_takeoff_M3_max = max(distance_takeoff_M3);
% % while m <= while_condition
% %     time_M3_min = time_M3_min + time_lap_M3(m);
% % 
% % end
% 
% 
% no_of_laps_M3 = m-1;
% time_M3_min = time_M3_min/60;
% %[no_of_laps_M3,time_M3_min,distance_takeoff_M3,cap_consumed_M3]= Mission_3(wing,cl_max,n,M3,no_of_boxes);
% 
% figure;
% 
% plot(T_overall2,V_overall2)
% hold on 
% plot(T_overall3,V_overall3)
% xlabel('Time (s)')
% ylabel('Velocity (m/s)')
% title('Missions model')
% grid on
% legend('Mission 2','Mission 3')
% 
% figure
% plot3(x_overall_M2,y_overall_M2,z_overall_M2,'LineWidth',1.5)
% hold on
% plot3(x_overall_mission3,y_overall_mission3,z_overall_mission3,'LineWidth',1.5)
% title('Aircraft flight path')
% pbaspect([3 2 2])
% grid on
% axis([min([x_overall_M2,x_overall_mission3])-10 max([x_overall_M2,x_overall_mission3])+10 min([y_overall_M2,y_overall_mission3]) max([y_overall_M2,y_overall_mission3])+10 0 max([z_overall_M2,z_overall_mission3])])
% xlabel('x position (m)')
% ylabel('y position (m)')
% zlabel('z position (m)')
% legend('Mission 2 flight path','Mission 3 flight path')
end

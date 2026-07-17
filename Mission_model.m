% This code simulates the mission based on the UAVs combinations stored in
% the matrix 'mat', which is extracted from the DSE_trial_2 script 
% This script includes 1 function: Mission_model_v1
% This function also includes 2 other functions, one for each mission, all
% functions expect UAV sizing inputs extracted from the matrix mat saved as
% CSV file in the same folder 

%% code explanation 

%% main script
clc; 
clearvars;
close all; 

%% Constants
SG       = 18.288;
g        = 9.81;
u        = 0.04;
n        = 3; % load factor
rho      = 1.225;
S_runway = 152;
CD_antenna = 1.0767; 

%% Important equations
e  = @(AR) 1.78*(1-0.045*AR^0.68)-0.64;
%e_0 = \frac{2}{2 - AR + \sqrt{4 + AR^2(1 + \tan^2 \Lambda_{MT})}}
%@(AR) 2 ./ (2 - AR + sqrt(4 + AR.^2));
K  = @(AR) 1/pi/e(AR)/AR;
iter = 0;
%%

mat = readmatrix('DSE_finalResults.CSV');

h = waitbar(0,'Running...');
totalIterations = size(mat,1);
for i = 1:size(mat,1)
    iter = iter + 1;        % ← always increment
              if mod(iter, 50000) == 0  % ← update every 50k iterations
                waitbar(iter/totalIterations, h, ...
                  sprintf('%.1f%%  |  Valid: %d', ...
                          100*iter/totalIterations, i-1));
              end
    w = mat(i,1);
    S = mat(i,6);
    Vcr = mat(i,2);
    wing.s = mat(i,6);
    wing.CL_max = mat(i,9);
    M2.MTOW = mat(i,1); %kg
    pd = mat(i,13); %kg
    T = mat(i,11);
    T3 = mat(i,12);
    L_antenna = mat(i,14);
    M3.CD0_M3 = mat(i,10);
    M2.CD0_M2 = mat(i,15);
    M2.C1 = 0.5*rho*M2.CD0_M2*wing.s;
    wing.AR = mat(i,7);
    M2.C2 = 2*(w*g)^2 / (rho*S*pi*e(wing.AR)*wing.AR);
    M2.Ts_any = T*1.3; %why 1.3? FS?
    M2.C1_M2_Th = -31/130 * M2.Ts_any / 4; % check >> Assuming Vp = 2*Vcr
    M2.C2_M2_Th = -0.4534 * M2.Ts_any / 2; % check 
    M2.C1_M2 = - M2.C1_M2_Th / (100/70)^2; % check 
    M2.C2_M2 = - M2.C2_M2_Th / (100/70); % check 
    M2.Ts_100 = M2.Ts_any * 100/70; % check 
    M2.allowed_time = 600;
    rho_PVC  = 1450;
    area_pipe = (0.02134^2 - 0.01580^2) * pi;
    M_antenna = L_antenna * area_pipe * rho_PVC; 
    M3.MTOW_3      = w - pd + M_antenna; %kg
    M3.C1 = 0.5*rho*M3.CD0_M3*S;
    M3.C2 = 2*(M3.MTOW_3*g)^2 / (rho*S*pi*e(wing.AR)*wing.AR);
    M3.Ts_any = T3*1.3;
    Vcr = mat(i,3);
    M3.C1_M3_Th = -31/130 * M3.Ts_any / 4^2; % check
    M3.C2_M3_Th = -0.4534 * M3.Ts_any / 2;
    M3.C1_M3_100 = -M3.C1_M3_Th / (100/70)^2;
    M3.C2_M3_100 = -M3.C2_M3_Th / (100/70);
    M3.Ts_100 = M3.Ts_any * 100/70;
    M3.allowed_time = 300;
    M3.laps_req = 3;  

     try
     disp(i)
     [E_max_M2, P_max_M2, Battery_results_M2, time_M2_min,TO_dist_M2,n_laps_M2,E_max_M3, P_max_M3, Battery_results_M3,time_M3_min,TO_dist_M3,Score_M2,...
     Score_M3,Overall_score,cap,Payload_weight_M2,v_avrg2,v_avrg3] = Mission_model_function(wing,n, M2,M3,pd,L_antenna); % bonus?
     mat(i,16:22) = [time_M3_min,v_avrg2,v_avrg3, n_laps_M2, Score_M2, Score_M3,0];
      writematrix(mat,'MissionModel_finalResults.CSV')
        catch ME
        %if contains(ME.message,"M3 ")
            disp([ME.message,num2str(i)]);
        %end

        continue;
     end
  
end

close(h);
%%
% ids = find(mat(:,16) ~= 0)';
% mat = mat([ids],:);
% %%
% max_score2 = max(mat(:,20));
% max_score3 = max(mat(:,21));
% mat(:,20) = mat(:,20)/max_score2 + 1;
% mat(:,21) = mat(:,21)/max_score3 + 2;
% mat(:,22) = mat(:,20) + mat(:,21); % what is errr?
% 
% %%
% close all
% % % Define grid for interpolation for takeoff weight and stall speed
% [xq, yq] = meshgrid(linspace(min(mat(:,17)), max(mat(:,17)), 50), linspace(min(mat(:,18)), max(mat(:,18)), 50));
% 
% % Interpolate the wing area or thrust over the grid
% vq = griddata(mat(:,17), mat(:,18), mat(:,19) , xq, yq);
% 
% % Create a surface plot
% surf(xq, yq, vq,'EdgeColor', 'None', 'FaceColor', 'interp');
% colorbar;
% hold on;
% [V,indx]= max(mat(:,18));
% %plot3(mat(indx,17), mat(indx,18), mat(indx,19), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
% xlabel('M2 score');
% ylabel('M3 score');
% title('2D Surface Plot of SCORES');
% view(2)
% %%
% close all
% figure()
% required_thrust = mat(:, 8);
% 
% x = mat(:,4);
% y2 = mat(:,7);
% z=mat(:,2);
% v=mat(:,17);
% scatter3(x,y2 ,z,20,v , 'filled');
% %% 3D scatter plot: X (takeoff weight), Y (stall speed), Z (wing area)
% close all
% figure()
% % for i = linspace(4,9,6)
% %     try
%     ids = find((mat(:,9) < 50) & (mat(:,10) < 50) & mat(:,12) == 200)';
%     mat2 = mat(ids,:);
%     y = mat2(:,14);
%     z = mat2(:,19);
%     x=mat2(:,11);
%     v=mat2(:,19);
%     [xq, yq] = meshgrid(linspace(min(x), max(x), 20), linspace(min(y), max(y), 20));
%     vq = griddata(x,y,v,xq, yq);
%     zq = griddata(x,y,z,xq,yq);
%     surf(xq, yq,vq,'EdgeColor', 'None', 'FaceColor', 'interp');
% zlabel('total score');
% ylabel('Thrust');
% xlabel('Vcruise');
% grid on;
% 
% %     hold on
% %     catch continue; end
% % end
% figure()
% scatter3(x,y,v,20,v,'filled')
% zlabel('total score');
% ylabel('Thrust');
% xlabel('Vcruise');
% grid on;
% % Add labels and title
% 
% %end
% %%
% close all
% % % Define grid for interpolation for takeoff weight and stall speed
% ids = find((mat(:,9) < 50) & mat(:,12) == 200 & mat(:,11)>2.7 & mat(:,11)<3.5 & mat(:,2) > 30 & mat(:,2) < 31)';
% mat2 = mat(ids,:);
% %%
% 
% ids = find(mat(:,9) < 50 & mat(:,12) == 200 & mat(:,2) == 30 & mat(:,11)>2 & mat(:,11)<3.6)';
% mat2 = mat(ids,:);
% figure()
% [xq, yq] = meshgrid(linspace(min(mat2(:,6)), max(mat2(:,6)), 20), linspace(min(mat2(:,11)), max(mat2(:,11)), 20));
% vq = griddata(mat2(:,6), mat2(:,11), mat2(:,18) , xq, yq);
% zq = griddata(mat2(:,6), mat2(:,11), mat2(:,19) , xq, yq);
% surf(xq, yq, zq,zq,'EdgeColor', 'None', 'FaceColor', 'interp');
% xlabel('AR');
% ylabel('payload');
% zlabel('total score');
% title('2D Surface Plot of SCORES');
% %%
% ids = find((mat(:,8) <= 40))';
% mat2 = mat(ids,:);
% figure()
% plot(mat2(:,14),mat2(:,17))
% figure()
% plot(mat2(:,10),mat2(:,17))
% %%
% figure()
% scatter(mat(:,2),required_thrust./mat(:,1)/9.81)
% %%
% writematrix(mat,'matrix_short2.xlsx');
% ttt = table(mat(:,1),mat(:,2),mat(:,3),mat(:,4),mat(:,5),mat(:,6),mat(:,7),mat(:,8),mat(:,9),mat(:,10),mat(:,11),mat(:,12),mat(:,13),mat(:,14),mat(:,15),mat(:,16),mat(:,17),mat(:,18),mat(:,19),'VariableNames',{'MTOW';'Vcr';'Vcr3';'Vs2';'S';'AR';'b';'Clms';'T2';'T3'; 'payload weight';'glider weight';'Time 2'; 'M2_TO_distance'; 'M3_TO_distance'; 'M3_laps'; 'M2_score'; 'M3_score';'overall score'});
% writetable(ttt,'results_short2.csv');
% writetable(ttt,'results_short2.xlsx');

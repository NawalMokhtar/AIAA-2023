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
SG       = 60;
g        = 9.81;
u        = 0.04;
n        = 3; % load factor
rho      = 1.225;
S_runway = 152;
CD0_antenna = 1.0767; 

%% Important equations
e  = @(AR) 4.61*(1-0.045*AR^0.68)-3.1;
K  = @(AR) 1/pi/e(AR)/AR;

%%

mat = readmatrix("results_modified.csv")

for i = 1:size(mat,1)
    w = mat(i,1);
    S = mat(i,6);
    Vcr = mat(i,2);
    wing.s = mat(i,6);
    wing.cl_max = mat(i,10);
    M2.MTOW = mat(i,1);
    pd = mat(i,13);
    T = mat(i,11);
    T3 = mat(i,12);
    L_antenna = mat(i,14);
    CD0 = mat(i,10);
    M2.C1 = 0.5*rho*CD0*wing.s;
    wing.AR = mat(i,7);
    M2.C2 = 2*(w*g)^2 / (rho*S*pi*e(wing.AR)*wing.AR);
    M2.Ts_any = T*1.3; %why 1.3? FS?
    M2.C1_M2_Th = -31/130 * M2.Ts_any / Vcr^2; % check >> 
    M2.C2_M2_Th = -0.4534 * M2.Ts_any / Vcr; % check 
    M2.C1_M2 = - M2.C1_M2_Th / (100/70)^2; % check 
    M2.C2_M2 = - M2.C2_M2_Th / (100/70); % check 
    M2.Ts_100 = M2.Ts_any * 100/70; % check 
    M2.allowed_time = 600;
    rho_PVC  = 1450;
    area_pipe = (0.0889^2 - 0.0779272^2) * pi;
    M_antenna = L_antenna * area_pipe * rho_PVC; 
    M3.MTOW_3      = w - g*pd + M_antenna*g;
    M3.C1 = 0.5*rho*CD0*S;
    M3.C2 = 2*(M3.MTOW_3*g)^2 / (rho*S*pi*e(wing.AR)*wing.AR);
    M3.Ts_any = T3*1.3;
    Vcr = mat(i,3);
    M3.C1_M3_Th = -31/130 * M3.Ts_any / Vcr^2;
    M3.C2_M3_Th = -0.4534 * M3.Ts_any / Vcr;
    M3.C1_M3_100 = -M3.C1_M3_Th / (100/70)^2;
    M3.C2_M3_100 = -M3.C2_M3_Th / (100/70);
    M3.Ts_100 = M3.Ts_any * 100/70;
    M3.allowed_time = 300;
    M3.laps_req = 3;

    try
     disp(i)
     [Time_for_M2,distance_takeoff_M2,Time_for_M3,distance_takeoff_M3,no_of_laps_M2,Score_M2,...
     Score_M3,Overall_score,cap,Payload_weight_M2,errr,v_avrg2,v_avrg3] = Mission_model_function(wing,wing.cl_max,n,M2,M3,pd,L_antenna); % bonus?
     mat(i,14:20) = [Time_for_M3,v_avrg2,v_avrg3, no_of_laps_M2, Score_M2, Score_M3, errr];
    catch ME
        %if contains(ME.message,"M3 ")
            disp([ME.message,num2str(i)]);
        %end
        continue;
    end
end
clc;
clearvars;
close all;

%% Cl for airfoils
cls = [0.808,0.0,0.465;
0.908,0.0,0.569;
0.958,0.0,0.559;
1.008,0.073,0.617;
1.058,0.053,0.63;
1.108,0.125,0.673;
1.158,0.157,0.713;
1.208,0.196,0.766;
1.258,0.266,0.823;
1.308,0.313,0.865;
1.358,0.403,0.933;
1.408,0.436,0.978;
1.458,0.473,1.015;
1.508,0.495,1.029;
1.558,0.543,1.081;
1.608,0.572,1.106;
1.658,0.593,1.122;
1.708,0.616,1.138;
1.758,0.789,1.294;
1.808,0.624,1.178;
1.858,0.829,1.362;
1.908,0.964,1.505;
1.958,0.992,1.51;
2.008,1.036,1.542;
2.058,1.222,1.702;
2.108,1.115,1.64;
2.158,1.209,1.64];

clms = cls(:,1)';

%% Payload weight correlation
payload = @(x) 0.7735 * x + -5.7021;

%% Constants
SG       = 60;
g        = 9.81;
u        = 0.04;
n        = 3; % load factor
rho      = 1.225;
S_runway = 152;
CD0_antenna = 1.0767; 

%% Important equations
e  = @(AR) 1.78*(1-0.045*AR^0.68)-0.64;
K  = @(AR) 1/pi/e(AR)/AR;

%% Shipping box
SB_dimensions_sum = 1.5748;
SB_length_array   = linspace(0.5, 1.2, 60);

%% Design parameters' arrays
Gross_Weight  = linspace(5,25,50)*g;
Vcruise_array = linspace(10,45,70);
Vstall_array  = linspace(6,17,11);
% CD0_array     = linspace(0.03,0.07,20); %check 
CL_max_array  = linspace(1.1,2,7);
AR_array      = linspace(3,8,8); %check 

totalIterations = numel(Gross_Weight)*numel(CL_max_array)* ...
                  numel(Vstall_array)*numel(Vcruise_array)* ...
                  numel(AR_array)*numel(SB_length_array);
% h = waitbar(0,'Running...');
%% iterations
mat = zeros(10000000, 14);
i   = 1;

for w = Gross_Weight
  w_kg = w / g;                         
  pd   = payload(w_kg);                 

  for CL_max = CL_max_array

    col = find(abs(clms - CL_max) < 0.1, 1);
    if isempty(col), continue; end

    for Vstall = Vstall_array
      for Vcruise = Vcruise_array

        if Vstall >= 0.6 * Vcruise, continue; end

        W_S   = 0.5 * rho * Vstall^2 * CL_max;
        S     = w / W_S;
        CL_cr = 2 * W_S / (rho * Vcruise^2);
        CL_to = CL_max;

        if CL_cr * 0.9 > cls(col,3) || CL_cr < cls(col,2) * 0.9
          continue;
        end


          for AR = AR_array
            b = sqrt(AR * S);

            for L_antenna = SB_length_array % check if it's better if it is put diagonally 
              bmax = L_antenna * 2;
              if (b < bmax) && (pd > 0.3 * w_kg)
                [wingDrag, Cd0_wing] = WingDrag(CL_cr, Vcruise, S, AR);
                CD0 = Cd0_wing + CD0_antenna;
                q      = 0.5 * rho * (Vcruise * 1.2)^2;
                TW_ms  = q * CD0 / W_S + K(AR) / q * W_S;

                Vto    = Vstall * 1.1;
                q_to   = rho/2 * (Vto/sqrt(2))^2;
                CD_to  = CD0 + K(AR) * CL_to^2;
                TW_to  = Vto^2/(2*g*SG) + q_to*CD_to/W_S + u*(1 - q_to*CL_to/W_S);

                V_climb    = (Vstall + Vcruise) / 2;
                climb_dist = S_runway - SG;
                t_climb    = climb_dist / V_climb;
                ROC        = sqrt(2/rho * W_S * sqrt(K(AR)/3/CD0)); % Vertical distance is not constrained in our mission so check 
                q_cl       = rho/2 * V_climb^2;
                TW_ROC     = ROC/V_climb + q_cl/W_S*CD0 + K(AR)/q_cl*W_S;

               
                Vtrn   = 0.8 * Vcruise;
                R      = Vtrn^2 / g / sqrt(n^2-1); % Turn radius 
                phi    = acosd(1 / n);
                q_trn  = 0.5 * rho * Vtrn^2;
                TW_trn = q_trn*(CD0/W_S + K(AR)*(1/q_trn/cosd(phi))^2*W_S);

                TW       = max([TW_ms TW_to TW_ROC TW_trn]);
                Thrust_M2 = w * 1.1 * TW;

                %% Mission 3
                rho_PVC  = 1450;
                area_pipe = (0.0889^2 - 0.0779272^2) * pi;
                M_antenna = L_antenna * area_pipe * rho_PVC; % FIX: multiply by rho_PVC!
                w_M3      = w - g*pd + M_antenna*g;
                W_S_M3    = w_M3 / S;

                Vcruise_M3 = sqrt(w_M3 / (rho * CL_cr * S * 0.5));
                Vstall_M3  = 0.6 * Vcruise_M3;
                Vclimb_M3  = (Vstall_M3 + Vcruise_M3) / 2;
                t_climb_M3 = climb_dist / Vclimb_M3;
                ROC_M3     = sqrt(2/rho * W_S_M3 * sqrt(K(AR)/3/CD0));
                q_cl3      = rho/2 * Vclimb_M3^2;

                Vto3   = Vstall_M3 * 1.1;
                q_to3  = rho/2 * (Vto3/sqrt(2))^2;
                TW_to3 = Vto3^2/(2*g*SG) + q_to3*CD_to/W_S_M3 + u*(1 - q_to3*CL_to/W_S_M3);

                TW_ROC3 = ROC_M3/Vclimb_M3 + q_cl3/W_S_M3*CD0 + K(AR)/q_cl3*W_S_M3;

                Vtrn3   = 0.8 * Vcruise_M3;
                R       = Vtrn^2 / g / sqrt(n^2-1); % Turn radius 
                phi3    = acosd(1 / n);
                q_trn3  = 0.5 * rho * Vtrn3^2;
                TW_trn3 = q_trn3*(CD0/W_S_M3 + K(AR)*(1/q_trn3/cosd(phi3))^2*W_S_M3);

                q_ms3  = 0.5 * rho * (Vcruise_M3*1.2)^2;
                TW_ms3 = q_ms3*CD0/W_S_M3 + K(AR)/q_ms3*W_S_M3;

                TW3       = max([TW_ms3 TW_to3 TW_ROC3 TW_trn3]);
                Thrust_M3 = w_M3 * 1.1 * TW3;

                mat(i,1:14) = [w_kg Vcruise Vcruise_M3 Vstall Vstall_M3 ...
                                S AR b CL_max CD0 Thrust_M2 Thrust_M3 pd L_antenna];

                % if mod(i,1000)==0
                %   waitbar(i/totalIterations,h);
                % end
                i = i + 1;

              end 
            end 
          end 
        end 
      end 
    end 
  end 


% close(h)
mat = mat(1:i-1, :);
fprintf('Total valid designs found: %d\n', i-1);


%% Drage coefficient calculation 
function [wingDrag, Cd0_wing] = WingDrag(CL, Vcr, Sref, AR)
%% constants definition and variable initializing
flow = 'T';
TC_wing = 0.12;
XC_wing = 0.3; % How is the wing location known?
h=0;
dF = 0.12;
rho = 1.225;
viscosity = 1.789e-5;
TR = 1;
sweep = 0;
dTR = -0.375 + 0.45*exp(-0.0375*sweep);
b = sqrt(Sref*AR); % What is Sref?
MAC_wing = b/AR;
%% Induced Drag
%%%%%%% Corrective Factors
kFuselage = 1 - 2*(dF/b)^2;
kDrag0 = 0.8;
kWinglets = (1+2*h/b)^2;
cfactors = kFuselage*kDrag0*kWinglets;
%%%%%%% Induced Drag Calculations
f_shift = 0.0524.*(TR-dTR).^4 - 0.15.*(TR-dTR).^3 + 0.1659.*(TR-dTR).^2 - 0.0706.*(TR-dTR) + 0.0119;
e_shift = 1./(1+f_shift.*AR);
Cdi = CL^2/(pi*AR*e_shift*cfactors);

%% Wing Parasite Drag
Re = Vcr*MAC_wing*rho/viscosity;
if flow == 'T'
    Cf_wing = 0.074/(Re)^0.2;
else
    Cf_wing = 1.328/sqrt(Re);
end
FF = (1+0.6*TC_wing/XC_wing+100*TC_wing^4);
Swet = 2*(1+0.2*TC_wing)*Sref;
Cd0_wing = Cf_wing*FF*(Swet/Sref);

wingDrag = (Cdi+Cd0_wing)*0.5*rho*Vcr^2*Sref;
end
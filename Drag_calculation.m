function [wingDrag, Cd0_wing] = WingDrag(CL, Vcr, Sref, AR)
%% constants definition and variable initializing
flow = 'T';
TC_wing = 0.12;
XC_wing = 0.3;
h=0;
dF = 0.12;
rho = 1.225;
viscosity = 1.789e-5;
TR = 1;
sweep = 0;
dTR = -0.375 + 0.45*exp(-0.0375*sweep);
b = sqrt(Sref*AR);
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
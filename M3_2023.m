function[cap_consumed_M3, lap_times_M3, time_M3_min, T_overall3, V_overall3, v_avrg3, TO_dist_M3, x_overall_M3, y_overall_M3, z_overall_M3] = M3_2023(wing,cl_max,n,M3,~)
    
imax = 1000;
M3_allowed_time = M3.allowed_time;
M3_bat_cap = M3.bat_cap;
Ts = M3.Ts_100;
Ts_any = M3.Ts_any;
laps_req = M3.laps_req;
MTOW = M3.MTOW_3;

%c1,c2 are the coeifficients of Drag
c1 = M3.C1;
c2 = M3.C2;

%c3,c4 are positive coeifficients of thrust-propulsion at 100% throttle
c3 = M3.P1;
c4 = M3.P2;

%c3,c4 are positive coeifficients of thrust-propulsion at 70% throttle
c3_throttle = M3.P1_throttle;
c4_throttle = M3.P2_throttle;

% Ground-rolling-friction coefficient used during takeoff
mu = 0.6;


S_ref = wing.s;
AR = wing.AR;

% Runge Katta approach
h=.1;
i=1;
% Arrays Initialization
t1(i)=0;
v1(i)=0;
distance(i) = 0;
x0(i)=0;
y0(i)=0;
z0(i)=0;

%Induced-drag coefficient during ground roll and climb
c2_max = 0.5 *1.225*S_ref*cl_max^2/(pi*0.85*AR); %% How can the 'e' be set as a constant here while it is changing in M2??
L_takeoff = 0;

%The lift coefficient required at load factor n (during turn)
CL_turn = 2*n*MTOW*9.81/(1.225*S_ref);

%Induced-drag coefficient during turn
c2_turn = 0.5*1.225*S_ref*CL_turn^2/(pi*0.85*AR);

%% %%%%%%%%Lap 1%%%%%%%
v_stall = sqrt((2*MTOW*9.81)/(1.225*S_ref*cl_max));
%% takeoff
while (L_takeoff <= (MTOW*9.81))
    v_dot = @(t,v)((((Ts-c3*v^2-c4*v)*1)-c1*v^2-c2_max*v^2-mu*(0.5*1.225*v^2*S_ref*cl_max-(MTOW*9.81)))/MTOW);
    k1 = h*v_dot(t1(i),v1(i));
    k2 = h*v_dot(t1(i)+0.5*h,v1(i)+0.5*k1);
    k3 = h*v_dot(t1(i)+0.5*h,v1(i)+0.5*k2);
    k4 = h*v_dot(t1(i)+h,v1(i)+k3);
    t1(i+1) = t1(i)+h;
    v1(i+1) = v1(i)+1/6*(k1+2*k2+2*k3+k4);
    L_takeoff = 0.5*1.225*S_ref*(v1(i+1)^2)*cl_max ;
    distance(i+1) = distance(i)+(v1(i+1)+v1(i))*0.5*h;
    x0(i+1)=x0(i)+(v1(i+1)+v1(i))*0.5*h;
    y0(i+1)=0;
    z0(i+1)=0;
    if i >= imax         
        error("no convirgence at M3 takeoff");     
    else         
        i=i+1;     
    end
    
end
v_takeoff = v1(i);
t_takeoff = t1(i);
TO_dist_M3 = distance(i);
x_new=x0(i);
y_new=y0(i);
z_new=z0(i);
%% climb
i=1;
altitude(i)=0;
theta = 25;
t2(i)=t_takeoff;
v2(i)=v_takeoff;
ROC(i)=0;
x1(i)=x_new;
y1(i)=y_new;
z1(i)=z_new;
while (altitude <= 70)
    v_dot = @(t,v)((((Ts-c3*v^2-c4*v)*1)-c1*v^2-c2_max*v^2-MTOW*9.81*sind(theta))/MTOW);
    k1 = h*v_dot(t2(i),v2(i));
    k2 = h*v_dot(t2(i)+0.5*h,v2(i)+0.5*k1);
    k3 = h*v_dot(t2(i)+0.5*h,v2(i)+0.5*k2);
    k4 = h*v_dot(t2(i)+h,v2(i)+k3);
    t2(i+1) = t2(i)+h;
    v2(i+1) = v2(i)+1/6*(k1+2*k2+2*k3+k4);
    ROC(i+1) = v2(i+1)*sind(theta);
    altitude(i+1) = altitude(i)+(ROC(i+1)+ROC(i))*0.5*h;
    x1(i+1)=x1(i)+(v2(i)+v2(i+1))*0.5*cosd(theta)*h;
    y1(i+1)=0;
    z1(i+1)=z1(i)+(v2(i)+v2(i+1))*0.5*sind(theta)*h;    
    if i >= imax         
        error("no convirgence at M3 climb");     
    else         
        i=i+1;     
    end
end
v_climb =v2(i);
t_climb = t2(i);
altitude_climb = altitude(i);
distance_climb = altitude_climb/tand(theta);
x_new=x1(i);
y_new=y1(i);
z_new=z1(i);
%% cruise 0 (after climb)
i=1;
v0(i)=v_climb;
t0(i)=t_climb;
distance_cruise0(i)=0;
x2(i)=x_new;
y2(i)=y_new;
z2(i)=z_new;
while ( distance_cruise0 <( 150-distance_climb-TO_dist_M3 ))
    v_dot = @(t,v)((((Ts_any-c3_throttle*v^2-c4_throttle*v)*1)-c1*v^2-c2/v^2)/MTOW);
    k1 = h*v_dot(t0(i),v0(i));
    k2 = h*v_dot(t0(i)+0.5*h,v0(i)+0.5*k1);
    k3 = h*v_dot(t0(i)+0.5*h,v0(i)+0.5*k2);
    k4 = h*v_dot(t0(i)+h,v0(i)+k3);
    t0(i+1) = t0(i)+h;
    v0(i+1) = v0(i)+1/6*(k1+2*k2+2*k3+k4);
    distance_cruise0(i+1) = distance_cruise0(i)+(v0(i+1)+v0(i))*0.5*h;
    x2(i+1)=x2(i)+(v0(i)+v0(i+1))*0.5*h;
    y2(i+1)=0;     z2(i+1)=z_new;
    if i >= imax         
        error("no convirgence at M3 at cruise 0");     
    else         
        i=i+1;     
    end
end
v_cruise0=v0(i);
t_cruise0=t0(i);
x_new=x2(i);
y_new=y2(i);
z_new=z2(i);
%% first turn (180 deg)
i=1;
high_load_factor=0;
v3(i) = v_cruise0;
t3(i) = t_cruise0;
Turn_angle(i)=0;
x3(i)=x_new;
y3(i)=y_new;
z3(i)=z_new;
while Turn_angle < pi
    v_dot = @(t,v)((((Ts_any-c3_throttle*v^2-c4_throttle*v)*1)-c1*v^2-c2_turn/v^2)/MTOW);
    k1 = h*v_dot(t3(i),v3(i));
    k2 = h*v_dot(t3(i)+0.5*h,v3(i)+0.5*k1);
    k3 = h*v_dot(t3(i)+0.5*h,v3(i)+0.5*k2);
    k4 = h*v_dot(t3(i)+h,v3(i)+k3);
    v3(i+1) = v3(i)+1/6*(k1+2*k2+2*k3+k4);
    
    if v3(i+1) < v_stall
        
        v3(i+1) = v_stall;
        high_load_factor = 1;
    end
    
    angle_dot= @(t,Turn_angle)(9.81*sqrt((n^2)-1)/v3(i+1));
    k1 = h*angle_dot(t3(i),Turn_angle(i));
    k2 = h*angle_dot(t3(i)+0.5*h,Turn_angle(i)+0.5*k1);
    k3 = h*angle_dot(t3(i)+0.5*h,Turn_angle(i)+0.5*k2);
    k4 = h*angle_dot(t3(i)+h,Turn_angle(i)+k3);
    t3(i+1) = t3(i)+h;
    Turn_angle(i+1) = Turn_angle(i)+1/6*(k1+2*k2+2*k3+k4);
    x3(i+1)=x3(i)+(v3(i)+v3(i+1))*cos(Turn_angle(i+1))*0.5*h;
    y3(i+1)=y3(i)+(v3(i)+v3(i+1))*sin(Turn_angle(i+1))*0.5*h;
    z3(i+1)=z_new;    
    if i >= imax         
        error("no convirgence at M3 first turn (180 deg)");     
    else         
        i=i+1;     
    end
end
v_turn = v3(i);
t_turn = t3(i);
x_new=x3(i);
y_new=y3(i);
z_new=z3(i);
if high_load_factor == 1
    error(" Adjust the load factor, the velocity at turn is smaller than stall velocity at first turn (180 deg) M3")
end

%% first cruise

i = 1;
v4(i) = v_turn;
t4(i) = t_turn;
distance_cruise(i) = 0;
x4(i)=x_new;
y4(i)=y_new;
z4(i)=z_new;
while distance_cruise < 150
    v_dot = @(t,v)((((Ts_any-c3_throttle*v^2-c4_throttle*v)*1)-c1*v^2-c2/v^2)/MTOW);
    k1 = h*v_dot(t4(i),v4(i));
    k2 = h*v_dot(t4(i)+0.5*h,v4(i)+0.5*k1);
    k3 = h*v_dot(t4(i)+0.5*h,v4(i)+0.5*k2);
    k4 = h*v_dot(t4(i)+h,v4(i)+k3);
    t4(i+1) = t4(i)+h;
    v4(i+1) = v4(i)+1/6*(k1+2*k2+2*k3+k4);
    distance_cruise(i+1) = distance_cruise(i)+(v4(i+1)+v4(i))*0.5*h;
    x4(i+1)=x4(i)-(v4(i)+v4(i+1))*0.5*h;
    y4(i+1)=y_new;
    z4(i+1)=z_new;
    if i >= imax         
        error("no convirgence at M3 First Cruise");     
    else         
        i=i+1;     
    end
end
v_cruise = v4(i);
t_cruise = t4(i);
x_new=x4(i);
y_new=y4(i);
z_new=z4(i);
%% turn 360
i=1;
high_load_factor=0;
v5(i) = v_cruise;
t5(i) = t_cruise;
Turn_angle2(i)=0;
x5(i)=x_new;
y5(i)=y_new;
z5(i)=z_new;
while (Turn_angle2 < 2*pi)
    v_dot = @(t,v)((((Ts_any-c3_throttle*v^2-c4_throttle*v)*1)-c1*v^2-c2_turn/v^2)/MTOW);
    k1 = h*v_dot(t5(i),v5(i));
    k2 = h*v_dot(t5(i)+0.5*h,v5(i)+0.5*k1);
    k3 = h*v_dot(t5(i)+0.5*h,v5(i)+0.5*k2);
    k4 = h*v_dot(t5(i)+h,v5(i)+k3);
    v5(i+1) = v5(i)+1/6*(k1+2*k2+2*k3+k4);
    
    if v5(i+1) < v_stall
        
        v5(i+1) = v_stall;
        high_load_factor = 1;
        
    end
    
    
    angle_dot= @(t,Turn_angle2)(9.81*sqrt((n^2)-1)/v5(i+1));
    k1 = h*angle_dot(t5(i),Turn_angle2(i));
    k2 = h*angle_dot(t5(i)+0.5*h,Turn_angle2(i)+0.5*k1);
    k3 = h*angle_dot(t5(i)+0.5*h,Turn_angle2(i)+0.5*k2);
    k4 = h*angle_dot(t5(i)+h,Turn_angle2(i)+k3);
    t5(i+1) = t5(i)+h;
    Turn_angle2(i+1) = Turn_angle2(i)+1/6*(k1+2*k2+2*k3+k4);
    x5(i+1)=x5(i)-(v5(i)+v5(i+1))*cos(Turn_angle2(i+1))*0.5*h;
    y5(i+1)=y5(i)+(v5(i)+v5(i+1))*sin(Turn_angle2(i+1))*0.5*h;
    z5(i+1)=z_new;   
         if i >= imax         
             error("no convirgence at M3 360 turn");     
         else         
             i=i+1;     
         end
end
v_turn360 =v5(i);
t_turn360 = t5(i);

if high_load_factor == 1
    error(" Adjust the load factor, the velocity at turn is smaller than stall velocity at turn 360 M3 ")
end
x_new=x5(i);
y_new=y5(i);
z_new=z5(i);
%% crusie 2 after turn 360
i=1;
v6(i)=v_turn360;
t6(i)=t_turn360;
distance_cruise2(i)=0;
x6(i)=x_new;
y6(i)=y_new;
z6(i)=z_new;
while distance_cruise2 <150
    v_dot = @(t,v)((((Ts_any-c3_throttle*v^2-c4_throttle*v)*1)-c1*v^2-c2/v^2)/MTOW);
    k1 = h*v_dot(t6(i),v6(i));
    k2 = h*v_dot(t6(i)+0.5*h,v6(i)+0.5*k1);
    k3 = h*v_dot(t6(i)+0.5*h,v6(i)+0.5*k2);
    k4 = h*v_dot(t6(i)+h,v6(i)+k3);
    t6(i+1) = t6(i)+h;
    v6(i+1) = v6(i)+1/6*(k1+2*k2+2*k3+k4);
    distance_cruise2(i+1) = distance_cruise2(i)+(v6(i+1)+v6(i))*0.5*h;
    x6(i+1)=x6(i)-(v6(i)+v6(i+1))*0.5*h;
    y6(i+1)=y_new;
    z6(i+1)=z_new;  
    if i >= imax         
        error("no convirgence at M3 cruise 2");     
    else         
        i=i+1;     
    end
end
v_cruise2=v6(i);
t_cruise2=t6(i);
x_new=x6(i);
y_new=y6(i);
z_new=z6(i);

%% second turn (180 deg)
i=1;
high_load_factor=0;
v7(i) = v_cruise2;
t7(i) = t_cruise2;
Turn_angle3(i)=0;
x7(i)=x_new;
y7(i)=y_new;
z7(i)=z_new;
while Turn_angle3 < pi
    v_dot = @(t,v)((((Ts_any-c3_throttle*v^2-c4_throttle*v)*1)-c1*v^2-c2_turn/v^2)/MTOW);
    k1 = h*v_dot(t7(i),v7(i));
    k2 = h*v_dot(t7(i)+0.5*h,v7(i)+0.5*k1);
    k3 = h*v_dot(t7(i)+0.5*h,v7(i)+0.5*k2);
    k4 = h*v_dot(t7(i)+h,v7(i)+k3);
    v7(i+1) = v7(i)+1/6*(k1+2*k2+2*k3+k4);
    
    if v7(i+1) < v_stall
        
        v7(i+1) = v_stall;
        high_load_factor=1;
        
    end
    
    angle_dot= @(t,Turn_angle3)(9.81*sqrt((n^2)-1)/v7(i+1));
    k1 = h*angle_dot(t7(i),Turn_angle3(i));
    k2 = h*angle_dot(t7(i)+0.5*h,Turn_angle3(i)+0.5*k1);
    k3 = h*angle_dot(t7(i)+0.5*h,Turn_angle3(i)+0.5*k2);
    k4 = h*angle_dot(t7(i)+h,Turn_angle3(i)+k3);
    t7(i+1) = t7(i)+h;
    Turn_angle3(i+1) = Turn_angle3(i)+1/6*(k1+2*k2+2*k3+k4);
    x7(i+1)=x7(i)-(v7(i)+v7(i+1))*cos(Turn_angle3(i+1))*0.5*h;
    y7(i+1)=y7(i)-(v7(i)+v7(i+1))*sin(Turn_angle3(i+1))*0.5*h;
    z7(i+1)=z_new;   
         if i >= imax         
             error("no convirgence at M3 second 180 turn");     
         else         
             i=i+1;     
         end
end

v_turn2 =v7(i);
t_turn2 = t7(i);

x_new=x7(i);
y_new=y7(i);
z_new=z7(i);

if high_load_factor == 1
    error(" Adjust the load factor, the velocity at turn is smaller than stall velocity at second turn (180 deg) M3")
end

%% crusie 3 after turn (180 deg)
i=1;
v8(i)=v_turn2;
t8(i)=t_turn2;
distance_cruise3(i)=0;
x8(i)=x_new;
y8(i)=y_new;
z8(i)=z_new;
while distance_cruise3 <150
    v_dot = @(t,v)((((Ts_any-c3_throttle*v^2-c4_throttle*v)*1)-c1*v^2-c2/v^2)/MTOW);
    k1 = h*v_dot(t8(i),v8(i));
    k2 = h*v_dot(t8(i)+0.5*h,v8(i)+0.5*k1);
    k3 = h*v_dot(t8(i)+0.5*h,v8(i)+0.5*k2);
    k4 = h*v_dot(t8(i)+h,v8(i)+k3);
    t8(i+1) = t8(i)+h;
    v8(i+1) = v8(i)+1/6*(k1+2*k2+2*k3+k4);
    distance_cruise3(i+1) = distance_cruise3(i)+(v8(i+1)+v8(i))*0.5*h;
    x8(i+1)=x8(i)+(v8(i)+v8(i+1))*0.5*h;
    y8(i+1)=y_new;
    z8(i+1)=z_new; 
    if i >= imax         
        error("no convirgence at M3 cruise 3");     
    else         
        i=i+1;     
    end
end
v_cruise3=v8(i);
t_cruise3=t8(i);
x_new=x8(i);
y_new=y8(i);
z_new=z8(i);
time_lap_1=t_cruise3;

%% %%%%%Lap general%%%%%%%
%%%%%%%%%%Lap 2%%%%%%%%%%

%% 1st cruise in 2nd lap
i=1;
vG1(i)=v_cruise3;
tG1(i)=t_cruise3;
distanceG_cruise1(i)=0;
x9(i)=x_new;
y9(i)=y_new;
z9(i)=z_new;
while ( distanceG_cruise1 < 150)
    v_dot = @(t,v)((((Ts_any-c3_throttle*v^2-c4_throttle*v)*1)-c1*v^2-c2/v^2)/MTOW);
    k1 = h*v_dot(tG1(i),vG1(i));
    k2 = h*v_dot(tG1(i)+0.5*h,vG1(i)+0.5*k1);
    k3 = h*v_dot(tG1(i)+0.5*h,vG1(i)+0.5*k2);
    k4 = h*v_dot(tG1(i)+h,vG1(i)+k3);
    tG1(i+1) = tG1(i)+h;
    vG1(i+1) = vG1(i)+1/6*(k1+2*k2+2*k3+k4);
    distanceG_cruise1(i+1) = distanceG_cruise1(i)+(vG1(i+1)+vG1(i))*0.5*h;
    x9(i+1)=x9(i)+(vG1(i)+vG1(i+1))*0.5*h;  
    y9(i+1)=y_new;
    z9(i+1)=z_new;  
    if i >= imax         
        error("no convirgence at M3 ");     
    else         
        i=i+1;     
    end
end
vG_cruise1=vG1(i);
tG_cruise1=tG1(i);
x_new=x9(i);
y_new=y9(i);
z_new=z9(i);
%% first turn (180 deg)
i=1;
high_load_factor=0;
vG2(i) = vG_cruise1;
tG2(i) = tG_cruise1;
Turn_angle4(i)=0;
x10(i)=x_new;
y10(i)=y_new;
z10(i)=z_new;
while Turn_angle4 < pi
    v_dot = @(t,v)((((Ts_any-c3_throttle*v^2-c4_throttle*v)*1)-c1*v^2-c2_turn/v^2)/MTOW);
    k1 = h*v_dot(tG2(i),vG2(i));
    k2 = h*v_dot(tG2(i)+0.5*h,vG2(i)+0.5*k1);
    k3 = h*v_dot(tG2(i)+0.5*h,vG2(i)+0.5*k2);
    k4 = h*v_dot(tG2(i)+h,vG2(i)+k3);
    vG2(i+1) = vG2(i)+1/6*(k1+2*k2+2*k3+k4);
    
    if vG2(i+1) < v_stall
        vG2(i+1) = v_stall;
        high_load_factor=1;
    end
    
    angle_dot= @(t,Turn_angle4)(9.81*sqrt((n^2)-1)/vG2(i+1));
    k1 = h*angle_dot(tG2(i),Turn_angle4(i));
    k2 = h*angle_dot(tG2(i)+0.5*h,Turn_angle4(i)+0.5*k1);
    k3 = h*angle_dot(tG2(i)+0.5*h,Turn_angle4(i)+0.5*k2);
    k4 = h*angle_dot(tG2(i)+h,Turn_angle4(i)+k3);
    tG2(i+1) = tG2(i)+h;
    Turn_angle4(i+1) = Turn_angle4(i)+1/6*(k1+2*k2+2*k3+k4);
    x10(i+1)=x10(i)+(vG2(i)+vG2(i+1))*cos(Turn_angle4(i+1))*0.5*h;
    y10(i+1)=y10(i)+(vG2(i)+vG2(i+1))*sin(Turn_angle4(i+1))*0.5*h;
    z10(i+1)=z_new; 
         if i >= imax         
             error("no convirgence at M3 ");     
         else         
             i=i+1;     
         end
end
vG_turn1 =vG2(i);
tG_turn1 = tG2(i);

x_new=x10(i);
y_new=y10(i);
z_new=z10(i);

if high_load_factor == 1
    disp(" Adjust the load factor, the velocity at turn is smaller than stall velocity at first turn (180 deg) M3 lap 2 ")
end
%% 2nd cruise
i=1;
vG3(i)=vG_turn1;
tG3(i)=tG_turn1;
distanceG_cruise2(i)=0;
x11(i)=x_new;
y11(i)=y_new;
z11(i)=z_new;
while distanceG_cruise2 <150
    v_dot = @(t,v)((((Ts_any-c3_throttle*v^2-c4_throttle*v)*1)-c1*v^2-c2/v^2)/MTOW);
    k1 = h*v_dot(tG3(i),vG3(i));
    k2 = h*v_dot(tG3(i)+0.5*h,vG3(i)+0.5*k1);
    k3 = h*v_dot(tG3(i)+0.5*h,vG3(i)+0.5*k2);
    k4 = h*v_dot(tG3(i)+h,vG3(i)+k3);
    tG3(i+1) = tG3(i)+h;
    vG3(i+1) = vG3(i)+1/6*(k1+2*k2+2*k3+k4);
    distanceG_cruise2(i+1) = distanceG_cruise2(i)+(vG3(i+1)+vG3(i))*0.5*h;
    x11(i+1)=x11(i)-(vG3(i)+vG3(i+1))*0.5*h;  
    y11(i+1)=y_new;
    z11(i+1)=z_new;
    if i >= imax         
        error("no convirgence at M3 ");     
    else         
        i=i+1;     
    end
end
vG_cruise2=vG3(i);
tG_cruise2=tG3(i);
x_new=x11(i);
y_new=y11(i);
z_new=z11(i);
%% turn 360
i=1;
high_load_factor=0;
vG4(i) = vG_cruise2;
tG4(i) = tG_cruise2;
Turn_angle5(i)=0;
x12(i)=x_new;
y12(i)=y_new;
z12(i)=z_new;
while (Turn_angle5 < 2*pi)
    v_dot = @(t,v)((((Ts_any-c3_throttle*v^2-c4_throttle*v)*1)-c1*v^2-c2_turn/v^2)/MTOW);
    k1 = h*v_dot(tG4(i),vG4(i));
    k2 = h*v_dot(tG4(i)+0.5*h,vG4(i)+0.5*k1);
    k3 = h*v_dot(tG4(i)+0.5*h,vG4(i)+0.5*k2);
    k4 = h*v_dot(tG4(i)+h,vG4(i)+k3);
    vG4(i+1) = vG4(i)+1/6*(k1+2*k2+2*k3+k4);
    
    if vG4(i+1) < v_stall
        
        vG4(i+1) = v_stall;
        high_load_factor=1;
        
    end
    
    angle_dot= @(t,Turn_angle5)(9.81*sqrt((n^2)-1)/vG4(i+1));
    k1 = h*angle_dot(tG4(i),Turn_angle5(i));
    k2 = h*angle_dot(tG4(i)+0.5*h,Turn_angle5(i)+0.5*k1);
    k3 = h*angle_dot(tG4(i)+0.5*h,Turn_angle5(i)+0.5*k2);
    k4 = h*angle_dot(tG4(i)+h,Turn_angle5(i)+k3);
    tG4(i+1) = tG4(i)+h;
    Turn_angle5(i+1) = Turn_angle5(i)+1/6*(k1+2*k2+2*k3+k4);
    x12(i+1)=x12(i)-(vG4(i)+vG4(i+1))*cos(Turn_angle5(i+1))*0.5*h;
    y12(i+1)=y12(i)+(vG4(i)+vG4(i+1))*sin(Turn_angle5(i+1))*0.5*h;
    z12(i+1)=z_new; 
         if i >= imax         
             error("no convirgence at M3 ");     
         else         
             i=i+1;     
         end
end
vG_turn2 =vG4(i);
tG_turn2 = tG4(i);

x_new=x12(i);
y_new=y12(i);
z_new=z12(i);

if high_load_factor == 1
    error(" Adjust the load factor, the velocity at turn is smaller than stall velocity at turn 360 M3 lap 2 ")
end

%% crusie 3 after turn 360
i=1;
vG5(i)=vG_turn2;
tG5(i)=tG_turn2;
distanceG_cruise3(i)=0;
x13(i)=x_new;
y13(i)=y_new;
z13(i)=z_new;
while distanceG_cruise3 <150
    v_dot = @(t,v)((((Ts_any-c3_throttle*v^2-c4_throttle*v)*1)-c1*v^2-c2/v^2)/MTOW);
    k1 = h*v_dot(tG5(i),vG5(i));
    k2 = h*v_dot(tG5(i)+0.5*h,vG5(i)+0.5*k1);
    k3 = h*v_dot(tG5(i)+0.5*h,vG5(i)+0.5*k2);
    k4 = h*v_dot(tG5(i)+h,vG5(i)+k3);
    tG5(i+1) = tG5(i)+h;
    vG5(i+1) = vG5(i)+1/6*(k1+2*k2+2*k3+k4);
    distanceG_cruise3(i+1) = distanceG_cruise3(i)+(vG5(i+1)+vG5(i))*0.5*h;
    x13(i+1)=x13(i)-(vG5(i)+vG5(i+1))*0.5*h;  
    y13(i+1)=y_new;
    z13(i+1)=z_new; 
    if i >= imax         
        error("no convirgence at M3 ");     
    else         
        i=i+1;     
    end
end
vG_cruise3=vG5(i);
tG_cruise3=tG5(i);
x_new=x13(i);
y_new=y13(i);
z_new=z13(i);

%% second turn (180 deg)
i=1;
high_load_factor=0;
vG6(i) = vG_cruise3;
tG6(i) = tG_cruise3;
Turn_angle6(i)=0;
x14(i)=x_new;
y14(i)=y_new;
z14(i)=z_new;
while Turn_angle6 < pi
    v_dot = @(t,v)((((Ts_any-c3_throttle*v^2-c4_throttle*v)*1)-c1*v^2-c2_turn/v^2)/MTOW);
    k1 = h*v_dot(tG6(i),vG6(i));
    k2 = h*v_dot(tG6(i)+0.5*h,vG6(i)+0.5*k1);
    k3 = h*v_dot(tG6(i)+0.5*h,vG6(i)+0.5*k2);
    k4 = h*v_dot(tG6(i)+h,vG6(i)+k3);
    vG6(i+1) = vG6(i)+1/6*(k1+2*k2+2*k3+k4);
    
    if vG6(i+1) < v_stall
        
        vG6(i+1) = v_stall;
        high_load_factor=1;
    end
    
    angle_dot= @(t,Turn_angle6)(9.81*sqrt((n^2)-1)/vG6(i+1));
    k1 = h*angle_dot(tG6(i),Turn_angle6(i));
    k2 = h*angle_dot(tG6(i)+0.5*h,Turn_angle6(i)+0.5*k1);
    k3 = h*angle_dot(tG6(i)+0.5*h,Turn_angle6(i)+0.5*k2);
    k4 = h*angle_dot(tG6(i)+h,Turn_angle6(i)+k3);
    tG6(i+1) = tG6(i)+h;
    Turn_angle6(i+1) = Turn_angle6(i)+1/6*(k1+2*k2+2*k3+k4);
    x14(i+1)=x14(i)-(vG6(i)+vG6(i+1))*cos(Turn_angle6(i+1))*0.5*h;
    y14(i+1)=y14(i)-(vG6(i)+vG6(i+1))*sin(Turn_angle6(i+1))*0.5*h;
    z14(i+1)=z_new;      
         if i >= imax         
             error("no convirgence at M3 ");     
         else         
             i=i+1;     
         end
end

vG_turn3 =vG6(i);
tG_turn3 = tG6(i);

x_new=x14(i);
y_new=y14(i);
z_new=z14(i);

if high_load_factor == 1
    error(" Adjust the load factor, the velocity at turn is smaller than stall velocity at second turn (180 deg) M3 lap 2 ")
end
%% crusie 4 after turn (180 deg)
i=1;
vG7(i)=vG_turn3;
tG7(i)=tG_turn3;
distanceG_cruise4(i)=0;
x15(i)=x_new;
y15(i)=y_new;
z15(i)=z_new;
while distanceG_cruise4 <150
    v_dot = @(t,v)((((Ts_any-c3_throttle*v^2-c4_throttle*v)*1)-c1*v^2-c2/v^2)/MTOW);
    k1 = h*v_dot(tG7(i),vG7(i));
    k2 = h*v_dot(tG7(i)+0.5*h,vG7(i)+0.5*k1);
    k3 = h*v_dot(tG7(i)+0.5*h,vG7(i)+0.5*k2);
    k4 = h*v_dot(tG7(i)+h,vG7(i)+k3);
    tG7(i+1) = tG7(i)+h;
    vG7(i+1) = vG7(i)+1/6*(k1+2*k2+2*k3+k4);
    distanceG_cruise4(i+1) = distanceG_cruise4(i)+(vG7(i+1)+vG7(i))*0.5*h;
    x15(i+1)=x15(i)+(vG7(i)+vG7(i+1))*0.5*h;  
    y15(i+1)=y_new;
    z15(i+1)=z_new; 
    if i >= imax         
        error("no convirgence at M3 ");     
    else         
        i=i+1;     
    end
end
vG_cruise4=vG7(i);
tG_cruise4=tG7(i);
time_lap_2=tG_cruise4 - t_cruise3;
gen_lap = laps_req -1;
lap_times_M3 = [time_lap_1, repmat(time_lap_2, 1, gen_lap)];
%% Total time

if (time_lap_1 + (laps_req-1)*time_lap_2) > M3_allowed_time
    error('Design cannot complete the required 3 laps within the Mission 3 time window');
end
time_M3_min =(time_lap_1+(2*(time_lap_2)))/60; % in minutes
time_M3_hour = time_M3 / 60;
cap_consumed_M3= (I_max_100_M2 * 1000 * time_M3_hour)/0.85; %% Imax should be only for climb and takeoff 

%% overall Parameters 

V_overall3=[v1 v2 v0 v3 v4 v5 v6 v7 v8 vG1 vG2 vG3 vG4 vG5 vG6 vG7];
v_avrg3 = mean([vG_cruise1, vG_cruise2, vG_cruise3, vG_cruise4]);
T_overall3=[t1 t2 t0 t3 t4 t5 t6 t7 t8 tG1 tG2 tG3 tG4 tG5 tG6 tG7];
x_overall_M3=[x0 x1 x2 x3 x4 x5 x6 x7 x8 x9 x10 x11 x12 x13 x14 x15];
y_overall_M3=[y0 y1 y2 y3 y4 y5 y6 y7 y8 y9 y10 y11 y12 y13 y14 y15];
z_overall_M3=[z0 z1 z2 z3 z4 z5 z6 z7 z8 z9 z10 z11 z12 z13 z14 z15];

%% Plotting
% figure
%subplot(1,2,1);
% plot(T_overall,V_overall)
% xlabel('Time (s)')
% ylabel('Velocity (m/s)')
% title('Mission 2')
% grid on

% figure
% plot3(x_overall_mission2,y_overall_mission2,z_overall_mission2)
% title('Aircraft flight path')
% pbaspect([3 2 2])
% grid on
% axis([min(x_overall)-10 max(x_overall)+10 min(y_overall) max(y_overall)+10 0 max(z_overall)])
% xlabel('x position (m)')
% ylabel('y position (m)')
% zlabel('z position (m)')

end
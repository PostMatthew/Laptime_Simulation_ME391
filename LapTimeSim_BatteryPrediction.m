%% Laptime Simulation -- Power Consumption Script for Lafayette Motorsports
% By: Matt Post

% House Keeping Statements:
clc; close all; clear all;

% Build a simple track
approxMaxStraight = 200; %m, approximated from endurance track 
approxMinRad = 10; %m, approximated from endurance track
lengths = [approxMaxStraight, approxMinRad, approxMaxStraight, approxMinRad];
angles = [0,pi,0,pi]; % angles for our turns --> pi gives 180 degree turn
% Builds custom Track (see jupyter notebook to show this function was created/works)
track = buildTrack(lengths,angles); 

Car = buildCar();
[LapTimeStruct,FSM_struct] = GetLapTime(Car,track);
LapTimeStruct = GetEnergy(LapTimeStruct);
%%
EnergyConsumed_1L = 998.3/track.S(end)*LapTimeStruct.Energy(end)/(3600);
EnergyConsumed_FullTrack = 44*998.3/track.S(end)*LapTimeStruct.Energy(end)/(3600*1E3);
MaxEnergy = 6; %KWh
message_1 = ["Energy consumed on one lap:" EnergyConsumed_1L "Wh"]; % energy for one lap of track
message_2 = ["Energy consumed throughout endurance track:" EnergyConsumed_FullTrack "KWh"]; % energy for one lap of track
if EnergyConsumed_FullTrack >= MaxEnergy
    message_3 = ["You will not finish the Endurance Track. Need" EnergyConsumed_FullTrack-MaxEnergy "more KWh"];
else
    message_3 = ["You will finish the Endurance Track!"]
end
disp(message_1)
disp(message_2)
disp(message_3)
%%
figure()
hold on
plot(LapTimeStruct.S_car_vect,LapTimeStruct.Power)
xlabel("simulated time, seconds")
ylabel("power consumed, W");
hold off

figure()
hold on
plot(LapTimeStruct.t_sim_vect,LapTimeStruct.Energy)
xlabel("simulated time, seconds")
ylabel("energy consumed, W");
hold off

figure()
hold on
plot(LapTimeStruct.S_car_vect,LapTimeStruct.S_double_dot_car_vect)
plot(LapTimeStruct.S_car_vect,LapTimeStruct.S_double_dot_car_vect.*FSM_struct.vects.Turn,"*")
plot(LapTimeStruct.S_car_vect,LapTimeStruct.S_double_dot_car_vect.*FSM_struct.vects.Brake,".")
xlabel("Station, m")
ylabel("Velocity, m/s");
xlim([0 track.S(end)])
hold off
legend("Car Velocity", "Is this a turning point?","Are we going to fast?")
%% FUNCTIONS:

%% Build Car Functions
function engine_parameters_out = LoadEngine()
    engine_parameters_out.r = 10.05/ 39.37; % wheel radius inches to m 
    engine_parameters_out.ni = 14; % inner tooth count
    engine_parameters_out.no = 40;  % outer tooth count
    engine_parameters_out.J_m = 0.023; % kg m^2 motor inertia 
    engine_parameters_out.J_l = 0; % load inertia 
    engine_parameters_out.m_car = 1000/ 2.205; %pound to kg
    engine_parameters_out.Cd = 1; % assume coefficient of drag is 1
    engine_parameters_out.p = 1.21; %kg/m^3 density of air 
    engine_parameters_out.A = 8/10.764; %ft^2 to m^2
    engine_parameters_out.Cr = 0.03; % assume the rolling resistance of 4 wheels is 0.03 from published value of car wheels on Asphalt 
    engine_parameters_out.ag = 9.81; % m/s^2 gravity 
    engine_parameters_out.Tf = 0; % N*m torque due to internal system friction  
    engine_parameters_out.mu = 0.7; % assume friction coefficient of tire equal to 1 (need data from tire)
    engine_parameters_out.omega_maximum = 6000; %rpm, max angular velocity
    engine_parameters_out.Tmax = 114; %from data sheet, maximum torque 
    engine_parameters_out.P_max = 22500; % W  battery provides 300 Amps/ Open circuit voltage around 100V/ assume resistance
    engine_parameters_out.S_dot_max = 30.1; % m, maximum velocity of car
end

function suspension_out = LoadSuspension()
    suspension_out.a = .5;%meters, length from CG to front axle
    suspension_out.b = 1;%meters, length from CG to rear axle
    suspension_out.Krr = 1.5e4; %Nm/rad
    suspension_out.Krf = 1.5e4*suspension_out.b/(suspension_out.a)*1.5; %Nm/rad. The roll stiffnesses should be from suspension analysis, and include ARBs
    suspension_out.Hs = .2; %meters, height of unsprung weight above ROLL AXIS. Roll axis comes from suspension analysis.
    suspension_out.Zrf = .1; %meters, height of front roll center
    suspension_out.Zrr = .13; %meters, height of rear roll center
    suspension_out.ms = 400; %kg, sprung weight of car
    suspension_out.Wsf = suspension_out.ms*9.81*suspension_out.b/(suspension_out.a+suspension_out.b); %weight on front axle
    suspension_out.Wsr = suspension_out.ms*9.81*suspension_out.a/(suspension_out.a+suspension_out.b); %weight on rear axle
    suspension_out.Wuf = 20*9.81;%unsprung weight front
    suspension_out.Wur = 20*9.81;%unsprung weight rear
    suspension_out.Zuf = .15;%unsprung weight CG height, front
    suspension_out.Zur = .15;%unsprung weight CG height, rear
    suspension_out.Tf = 1.22;%meters, front track width
    suspension_out.Tr = 1.22;%meters, rear track width
    suspension_out.m = suspension_out.ms+suspension_out.Wuf/9.81+ suspension_out.Wur/9.81;%total mass
    suspension_out.Steer_Ratio_Approx = 12; %12 degrees HW / 1 degree RW
    suspension_out.mumax = 1.2;%this really is a function of the road surface-- what is the maximum friction? Used to scale tire.
end

function Car = buildCar()
    Car.sus = LoadSuspension();
    Car.engine = LoadEngine();
    if Car.sus.m ~= Car.engine.m_car
        Car.engine.m_car = Car.sus.m;
    end
end

%% Build Track Functions (GPS)
function [ecef_x, ecef_y, ecef_z,lat0,lon0,lat,lon] = LLA_to_ECEF(fname)
    data = load(fname);
    lat = data(:,1);
    lon = data(:,2);
    elev = data(:,3);
    earth_radius = 6371000; % Earth Radius, meters
    r = elev+earth_radius;
    ecef_x = r.*cosd(lat).*cosd(lon);
    ecef_y = r.*cosd(lat).*sind(lon);
    ecef_z = r.*sind(lat);
    lat0 = lat(1);
    lon0 = lon(1);
end

function [xEast, yNorth, zUp] = ecef2enu(ecef_x,ecef_y,ecef_z, lat0,lon0,ref_z)
    
    earth_radius = 6371000; % Earth Radius, meters
    r = ref_z+earth_radius;
    ref_x = r.*cosd(lat0).*cosd(lon0);
    ref_y = r.*cosd(lat0).*sind(lon0);
    ref_z = r.*sind(lat0);
    diff_ecef_x = ecef_x-ref_x;
    diff_ecef_y = ecef_y-ref_y;
    diff_ecef_z = ecef_z - ref_z;
    lambda = lat0+90;
    phi = lon0;
    %theta_z = lambda+pi/2; % converted to radians
    %theta_x = (pi/2-phi);
    theta_z = lambda*0.0174533; % converted to radians
    theta_x = (pi/2-phi*0.0174533);
    xEast = cos(theta_z)*diff_ecef_x+sin(theta_z)*diff_ecef_y;
    yNorth = diff_ecef_z*sin(theta_x)+diff_ecef_y*cos(theta_x)*cos(theta_z)-diff_ecef_x*cos(theta_x)*sin(theta_z);
    zUp = diff_ecef_z*cos(theta_x)-diff_ecef_y*cos(theta_z)*sin(theta_x)+diff_ecef_x*sin(theta_x)*sin(theta_z);
%              X*cz + Y*sz
% Z*sx + Y*cx*cz - X*cx*sz
% Z*cx - Y*cz*sx + X*sx*sz

%     Rz = [cos(theta_z),sin(theta_z),0; -sin(theta_z), cos(theta_z),0; 0,0,1;];
%     Rx_p = [1,0,0;0,cos(theta_x),sin(theta_x);0,-sin(theta_x),cos(theta_x);];
%     Enu = Rx_p*Rz*[ecef_x;ecef_y;ecef_z]
%     xEast = Enu(1);
%     yNorth = Enu(2);
%     zUp = Enu(3);
end

function [S,v,K] = GetStation(xEast,yNorth,zUp)
    S(1)=0;
    v(1)=0;
    K(1)=0;
    for i=2:length(xEast)
        delta_s(i) = sqrt((xEast(i)-xEast(i-1))^2+(yNorth(i)-yNorth(i-1))^2+(zUp(i)-zUp(i-1))^2); % diffrence in Station of track
        S(i) = delta_s(i)+S(i-1); % station of track
        v(i) = atan2((yNorth(i)-yNorth(i-1)),(xEast(i)-xEast(i-1))); % track heading
        K(i) = (v(i)-v(i-1))./(S(i)-S(i-1));
    end
end

function [S,v,K,xEast,yNorth] = BuildTrack(fname)
    [ecef_x, ecef_y, ecef_z,lat0,lon0] = LLA_to_ECEF(fname);
    [xEast, yNorth, zUp] = ecef2enu(ecef_x,ecef_y,ecef_z, lat0,lon0,ecef_z(1));
    xEast = xEast-xEast(1);
    yNorth = yNorth-yNorth(1);
    zUp = zUp - zUp(1);
    [S,v,K] = GetStation(xEast,yNorth,zUp);
end

function plts = pltTrackData(S,v,K,xEast,yNorth)
    figure(1)
    hold on
    plot(xEast,yNorth)
    xlabel("xEast (m)")
    ylabel("yNorth (m)")

    figure()
    hold on
    plot(S,v)
    xlabel("Station (m)")
    ylabel("Heading (radians)")

    figure()
    hold on
    xlabel("Station (m)")
    ylabel("Curviture (radians)")
    plot(S,K)
end

%% Dynamics / Track Info Functions
function K_here = K_hereFunc(S_track,K_track,S_here)
    K_here = interp1(S_track,K_track,S_here);
end

function bool_turn = AreWeInATurn(K_here)
    K_thresh = 1/25; %1/m
    if abs(K_here)>=K_thresh
        bool_turn = true;
    else
        bool_turn = false;
    end
end

function S_dot_crit = CritSpeed(K_next,fy_max)
    g = 9.81; % m/s^2, gravitational acceleration
    S_dot_crit = sqrt((fy_max*g)/abs(K_next));
end

function [K_next,dist_to_next_turn] = NextCurviture(S,K,S_here)
    indicies = 1:length(S);
    min_turn_radius = 6; %  minimum turning radius, Verfied by calculations done by G. Hartman
    turn_ahead_indicies = indicies(S>S_here & abs(K) > 1/min_turn_radius); %find indicies until the next turn
    if length(turn_ahead_indicies)>1
        next_turn_index = turn_ahead_indicies(1); % reports index of next turn
        K_next = K(next_turn_index); % reports the curviture of next turn [should be greater than abs(0.05)]
        %next_turn_indicies =  
    else 
        next_turn_index = indicies(end);
        K_next = K(end);
    end
    dist_to_next_turn = S(next_turn_index)-S_here;
end

function S_dot_crit_brake = CritBrake(S_crit_turn,ay_max,dist_to_next_turn)
    S_dot_crit_brake = sqrt(S_crit_turn^2+2*ay_max*dist_to_next_turn);
end

function bool_TooFast = AreWeTooFast(S_dot_car,S_dot_crit)
    if S_dot_car >= S_dot_crit
        bool_TooFast = 1;
    else
        bool_TooFast = 0;
    end
end

function integrated_thing = Integrator(integrated_last,deriv_current, step)
    integrated_thing = integrated_last+deriv_current*step;
end

function [dVdt,Tm,Omega_m] = EngineModel(Car,V_car)
    h = 0.01;
    % Calculations based on parameters or constants
    n_gear = Car.engine.ni/Car.engine.no; % gear ratio 
    p = 1.21; %kg/m^3 density of air 
    J_t = n_gear^2*Car.engine.J_l + Car.engine.J_m; % total inertia equation from website
    g = 9.81; % m/s^2 gravity 
    Tf = 0; % N*m torque due to internal system friction  
    mu = 0.7; % assume friction coefficient of tire equal to 1 (need data from tire)
    F_slip = mu*Car.engine.m_car*g; % mu*N
    T_slip = F_slip * Car.engine.r*n_gear; % maximum torque applied to wheels before tire slipping
    
    % identify T_limit (when the wheels start to slip)
    if T_slip <= Car.engine.Tmax 
        T_limit = T_slip; % maximum torque before wheel slipping
    end
    
    if T_slip > Car.engine.Tmax
        T_limit = Car.engine.Tmax; % maximum torque from motor 
    end
    
    omega_limit = 9.5488 * Car.engine.P_max /T_limit; % % Torque from the motor exceeds Torque limit until omega_motor reaches certain rpm 
    
    if V_car <= Car.engine.omega_maximum*2*pi/60*Car.engine.r*n_gear % before reaching maximum 6000 rpm
        
        if V_car <  omega_limit*2*pi/60*Car.engine.r*n_gear %m/s Torque from the motor exceeds maximum of 140 NM until reaches this speed 
            Tm = T_limit;
        end
        if V_car > omega_limit*2*pi/60*Car.engine.r*n_gear % m/s 
            Tm = Car.engine.P_max/(V_car/n_gear/Car.engine.r); %  N*m
        end
        
        dVdt = 1/(Car.engine.J_m/(n_gear*Car.engine.r)+n_gear*Car.engine.r*Car.engine.m_car)*Tm -Tf - n_gear*Car.engine.r*0.5*Car.engine.Cd*p*Car.engine.A/(Car.engine.J_m/(n_gear*Car.engine.r)+n_gear*Car.engine.r*Car.engine.m_car)*V_car.^2 - n_gear*Car.engine.r*Car.engine.Cr*Car.engine.m_car*g /(Car.engine.J_m/(n_gear*Car.engine.r)+n_gear*Car.engine.r*Car.engine.m_car);
    
    else 
        dVdt=0;
    end
    
    Omega_wheel = V_car/2/pi/Car.engine.r*60; % rpm
    Omega_m = V_car/n_gear/Car.engine.r*60/2/pi; %rpm
    V_car = V_car + dVdt*h;
    F_wheel = Car.engine.m_car*dVdt + 0.5*Car.engine.Cd*p*Car.engine.A*V_car.^2+Car.engine.Cr*Car.engine.m_car*g;
    
    
end

%% STEADY STATE CORNERING FUNCTION
function [TireData] = loadTireData(fname,mu_max)
    % load tire data
    tire_fy = load(fname);
    loads = tire_fy(1,2:end);
    slips = tire_fy(2:end,1);
    forces = tire_fy(2:end,2:end);
    %calculate the tires' nominal maximum friction (during test)
    mu_nominal = max(forces(:,1)/loads(1));
    %load these variables into a structure for ease of use.
    TireData.loads = loads;
    TireData.slips = slips*pi/180; % CONVERT TO RADIANS
    %scale entire tire forces according to the mu max parameter
    TireData.forces = forces*mu_max/mu_nominal;
end

% This function relies on a global TireData object (above).
function [Fy] = getTireFy(Fz,alpha,TireData)
    Fy = interp2(TireData.slips,TireData.loads,TireData.forces',alpha,Fz,'spline');
end

function [steer_inside,steer_outside] = loadSteerData(fname)
    %load raw data with columns [handwheel, roadwheel]
    steerdata = load(fname);
    %check to see if signs need to be flipped. Assume that data start with a LEFT handwheel angle
    if(sign(steerdata(end,1)/steerdata(end,2))==-1)
        steerdata(:,1) = -steerdata(:,1);
    end
    %now split the data. inside wheel should now be where the handwheel angle is >0
    steer_inside = steerdata(steerdata(:,1)>=0,:);
    steer_outside = abs(steerdata(steerdata(:,1)<=0,:));
end

function [delta_1, delta_2] = getSteerAngles(delta_HW,steer_left,steer_right)
    delta_1 = interp1(steer_left(:,1),steer_left(:,2),delta_HW,'extrap');
    delta_2 = interp1(steer_right(:,1),steer_right(:,2),delta_HW,'extrap');
end

function [alpha_f_left,alpha_f_right,alpha_r_left,alpha_r_right] = getSlipAngles(Car,U,R,V,HW)
    %assumes rear slip angles are the same.
    %call function to find our two roadwheel steer angles.
    [delta_left,delta_right] = getSteerAngles(HW,Car.sus.steer_left,Car.sus.steer_right);
    %compute yaw rate
    psidot = U/R;%U is fwd speed in m/s, R is turn radius in m.
    %now compute our slip angles
    alpha_f_left = atan((V+Car.sus.a*psidot)/(U-Car.sus.Tf/2*psidot))-delta_left;%from geometry
    alpha_f_right = atan((V+Car.sus.a*psidot)/(U+Car.sus.Tf/2*psidot))-delta_right;%from geometry
    alpha_r_left = atan((V-Car.sus.b*psidot)/(U-Car.sus.Tf/2*psidot));
    alpha_r_right = atan((V-Car.sus.b*psidot)/(U+Car.sus.Tf/2*psidot));
end

function Car = addSteering_and_Tire(Car,tirefile,steerfile)
    [TireData] = loadTireData(tirefile,1.2);
    Car.sus.TireData = TireData;%load car's tire data table using maximum mu
    [steer_left,steer_right] = loadSteerData(steerfile);% load car's steering system table
    [Car.sus.steer_left] = steer_left;
    [Car.sus.steer_right] = steer_right;
end

function [Fzf1,Fzf2,Fzr1,Fzr2] = getVerticalLoads(Car,U,R)
    %this function is based on Milliken pg 286
    ay = U^2/R;
    %we will assume LEFT HAND turns positive (symmetry), so positive ay will result in an increase in the RIGHT tire load (tire 2 on each axle)
    
    %compute front axle loads
    Pkf = Car.sus.Krf/(Car.sus.Krf+Car.sus.Krr-(Car.sus.Wsf+Car.sus.Wsr)*Car.sus.Hs);%fractional roll stiffness on this axle.
    dFzf = 2*ay/(Car.sus.Tf*9.81)*(Pkf*Car.sus.Hs*(Car.sus.Wsf+Car.sus.Wsr) + Car.sus.Wsf*Car.sus.Zrf + Car.sus.Wuf*Car.sus.Zuf);
    Fzf1 = (Car.sus.Wuf+Car.sus.Wsf)/2-dFzf;%left front wheel vertical load
    Fzf2 = (Car.sus.Wuf+Car.sus.Wsf)/2+dFzf;%right front wheel vertical load

    %compute rear axle loads
    Pkr = Car.sus.Krr/(Car.sus.Krf+Car.sus.Krr-(Car.sus.Wsf+Car.sus.Wsr)*Car.sus.Hs);%fractional roll stiffness on this axle.
    dFzr = 2*ay/(Car.sus.Tr*9.81)*(Pkr*Car.sus.Hs*(Car.sus.Wsf+Car.sus.Wsr) + Car.sus.Wsr*Car.sus.Zrr + Car.sus.Wur*Car.sus.Zur);
    Fzr1 = (Car.sus.Wur+Car.sus.Wsr)/2-dFzr;%left rear wheel vertical load
    Fzr2 = (Car.sus.Wur+Car.sus.Wsr)/2+dFzr;%right rear wheel vertical load
end

function phi = calcPhi(q,Car,U,R)
    
    %pull lateral velocty and HW angle out of the q vector
    V = q(1);
    HW = q(2);
    
    %calculate slip angles
    [alpha_f_left,alpha_f_right,alpha_r_left,alpha_r_right] = getSlipAngles(Car,U,R,V,HW,Car.sus.steer_left,Car.sus.steer_right);
    %calculate vertical tire force
    [Fzf1,Fzf2,Fzr1,Fzr2] = getVerticalLoads(Car,U,R);
    %get steer angles
    [delta_left,delta_right] = getSteerAngles(HW,Car.sus.steer_left,Car.sus.steer_right);
    %now calculate the lateral tire forces
    Fyf1 = -getTireFy(Fzf1,abs(alpha_f_left),Car.sus.TireData)/sign(alpha_f_left);
    Fyf2 = -getTireFy(Fzf2,abs(alpha_f_right),Car.sus.TireData)/sign(alpha_f_right);
    Fyr1 = -getTireFy(Fzr1,abs(alpha_r_left),Car.sus.TireData)/sign(alpha_r_left);
    Fyr2 = -getTireFy(Fzr2,abs(alpha_r_right),Car.sus.TireData)/sign(alpha_r_right);
   
    %now calculate values of each constraint
    phi(1,1) = (Fyf1*cos(delta_left)+Fyf2*cos(delta_right)+Fyr1+Fyr2) - Car.sus.m*U^2/R*sign(HW); %the sum of this should be 0!
    %phi(2,1) = Car.a*(Fyf1*cos(delta_left)+Fyf2*cos(delta_right)) -Car.b*(Fyr1+Fyr2) + Car.Tf/2*(Fyf1*sin(delta_left) - Fyf2*sin(delta_right)); ; %this is the sum of the moments in yaw. Should = 0!!
    phi(2,1) = Car.sus.a*(Fyf1*cos(delta_left)+Fyf2*cos(delta_right)) -Car.sus.b*(Fyr1+Fyr2) ; %this is the sum of the moments in yaw. Should = 0!!

end

function Jac = calcJac(q,Car,U,R)
    %calculate phi once.
    phi = calcPhi(q,Car,U,R);
    eps = .01;%value of perturbation
    for k = 1:length(q)
        %perturb q
        qlocal = q;
        qlocal(k) = qlocal(k)+eps;
        %calculate phi
        philocal = calcPhi(qlocal,Car,U,R);
        %now this column of jacobian is just the approx derivative
        Jac(:,k) = (philocal-phi)/eps;
    end
end

function [q,phi] = iterate(q,Car,U,R)
    Jac = calcJac(q,Car,U,R);
    phi = calcPhi(q,Car,U,R);
    q = q - pinv(Jac)*phi;
    phi = calcPhi(q,Car,U,R);
end

function [q,phi,niter] = NRSolve(q,Car,U,R,maxiter)
    thresh=.01;
    phi = calcPhi(q,Car,U,R);
    niter = 0;
    while (max(abs(phi))>thresh)&&(niter<maxiter)
        [q,phi] = iterate(q,Car,U,R);

        phi;
        niter=niter+1;
    end
end

function [Umax,aymax,delta,V] = findMaxSpeed(Car,mumax,R,thresh)
    g = 9.81;
    %find initial speed guess
    ayguess = mumax/2*g;
    ayguess/g;
    U = sqrt(ayguess*R);
    Ulast = 0;
    %keep track of how many iterations this solver used
    speediter = 0;

    %set max iterations for newton raphson
    maxiter = 20;

    done = false;
    %iterate on speed
    while ~done
        speediter = speediter+1
        %quesses for q based on ackermann angle
        steerangle_guess = (Car.sus.a+Car.sus.b)/R * Car.sus.Steer_Ratio_Approx *180/pi;
        %initial guess for parameters V and handwheel
        qguess = [0;steerangle_guess];
        [q,phi,niter] = NRSolve(qguess,Car,U,R,maxiter);
        hw = q(2);%keep track of handwheel solution
        if((abs(hw)<180) && (niter<maxiter)) %this is a good solution, so increment U
            Udiff = (U-Ulast);
            if abs(Udiff)<thresh
                done = true;
            else
                Ulast = U;
                U = U + abs(Udiff)/2;
%                 disp('increased U');
            end
        else
            Udiff = (U-Ulast);
            Ulast = U;
            U = U - abs(Udiff)/2;
%             disp('decreased U');
        end
    end

    Umax = U;
    aymax = U^2/R/g;
    delta = q(2);
    V = q(1);
    disp(['completed in: ' num2str(speediter) ' iterations'])
end
%% Test Track Functions
function [track] = fillTrack(S,K)
    nu(1) = 0;
    X(1) = 0;
    Y(1) = 0;
    %build a high resolution map
    Sq = 0:.001:max(S);
    Kq = interp1(S,K,Sq);

    for k = 2:length(Sq)
       nu(k) = nu(k-1)+Kq(k-1)*(Sq(k)-Sq(k-1));
       X(k) = X(k-1) + (Sq(k)-Sq(k-1))*cos(nu(k-1));
       Y(k) = Y(k-1) + (Sq(k)-Sq(k-1))*sin(nu(k-1));
    end

    track.heading = nu;
    track.X = X;
    track.Y = Y;
    track.S = Sq;
    track.K = Kq;
end

function [S,K] = initializeTrack()
    S = []; % initialize vectors
    K = [];
    S(1) = 0; % prepopulate with values of 0 for initial station point
    K(1) = 0;
end

function [S,K] = addStraight(S_current,K_current,lengthStraight,KStraight)
    N = length(S_current);
    fudge = 0.001;
    S_current(N+1) = S_current(N)+fudge;
    K_current(N+1) = KStraight;
    S_current(N+2) = S_current(N+1)+lengthStraight+fudge;
    K_current(N+2) = KStraight;
    S = S_current;
    K = K_current;
end

function [S,K] = addTurn(S_cur,K_cur,RTurn,thetaTurn)
    N = length(S_cur);
    fudge = 0.001;
    KTurn = 1/RTurn;
    STurn = abs(thetaTurn*RTurn); % arc length
    S_cur(N+1) = S_cur(N)+fudge;
    S_cur(N+2) = S_cur(N+1)+STurn+fudge;
    
    if thetaTurn <=0
        K_cur(N+1) = -KTurn;
        K_cur(N+2) = -KTurn;
    else
        K_cur(N+1) = KTurn;
        K_cur(N+2) = KTurn;  
    end
    S = S_cur;
    K = K_cur;
end

function [S,K] = StraightOrTurn(S_cur,K_cur,Length,angle)
    if angle ~= 0
        [S,K] = addTurn(S_cur, K_cur,Length,angle);
    else
        [S,K] = addStraight(S_cur, K_cur,Length,angle);
    end
end

function track = buildTrack(lengths,angles)
    [S,K] = initializeTrack();
    for i = 1:length(angles)
        [S,K] = StraightOrTurn(S,K,lengths(i),angles(i));
    end
    track = fillTrack(S,K);
end

function plts = pltTrack(track)
    figure(1)
    hold on
    title("Map of Track")
    plot(track.X,track.Y,"ko")
    xlabel('Track X (m)')
    ylabel('Track Y (m)')
    axis equal
    hold off
    
    figure(2)
    hold on
    title("Station v. Heading")
    plot(track.S,track.heading)
    xlabel('Track Station (m)')
    ylabel('Track Heading (radians)')
    hold off
    
    figure(3)
    hold on
    title("Station v. Curviture")
    plot(track.S,track.K)
    xlabel('Track Station (m)')
    ylabel('Track Curviture (radians)')
    hold off
end

%% Simulation Functions
function LapTimeStruct = initializeLapTimeSim()
    % Initialize needed scalars and vectors
    LapTimeStruct.S_car = 0; % initial station location of the car, m
    LapTimeStruct.S_dot_car = 0; % initial velocity of the car, m/s
    LapTimeStruct.S_double_dot_car = 0; % initial acceleration of car, m/s^2
    LapTimeStruct.S_dot_crit =0;
    LapTimeStruct.S_dot_crit_brake = 0;
    LapTimeStruct.t_sim_vect = [];
    LapTimeStruct.t_sim = 0;
    LapTimeStruct.S_car_vect = [];
    LapTimeStruct.S_dot_crit_vect =[];
    LapTimeStruct.S_dot_crit_brake_vect = [];
    LapTimeStruct.S_double_dot_car_vect = [];
    LapTimeStruct.S_dot_car_vect = [];
    LaptTimeStruct.Omega_m = 0;
    LapTimeStruct.Omega_m_vect = [];
    LapTimeStruct.TurnBoolVect = [];
    LapTimeStruct.TooFastBoolVect = [];
    LapTimeStruct.Tm = 0;
    LapTimeStruct.Tm_vect = [];
    LapTimeStruct.S_crit_vect = [];
    LapTimeStruct.K_next_vect = [];
    LapTimeStruct.dist_to_next_turn_vect = [];
    % Time stuff:
    LapTimeStruct.t_sim = 0; % Zero out simulated time
end

function FSM_Struct = FSM_initialize()
% Declare States 
   FSM_Struct.states.Throttle = true; % Assume we apply gas as soon as the track starts 
   FSM_Struct.states.Brake = false;
   FSM_Struct.states.Turn = false;
   
% Declare state vectors:
    FSM_Struct.vects.Throttle = [FSM_Struct.states.Throttle];
    FSM_Struct.vects.Brake = [FSM_Struct.states.Brake];
    FSM_Struct.vects.Turn = [FSM_Struct.states.Turn];
    
% Declare Transitions:
   
   % Throttle Transitions--
   FSM_Struct.transitions.Throttle_Latch = false;
   FSM_Struct.transitions.Throttle_To_Brake = false;
   FSM_Struct.transitions.Throttle_To_Turn = false;
   
   % Brake Transitions--
   FSM_Struct.transitions.Brake_Latch = false;
   FSM_Struct.transitions.Brake_To_Turn = false;
   FSM_Struct.transitions.Brake_To_Throttle = false;
   
   % Turn Transitions--
   FSM_Struct.transitions.Turn_Latch = false;
   FSM_Struct.transitions.Turn_To_Brake = false;
   FSM_Struct.transitions.Turn_To_Throttle = false;

% Declare Transition Vectors:

   % Throttle Transitions--
   FSM_Struct.vects.Throttle_Latch = [FSM_Struct.transitions.Throttle_Latch];
   FSM_Struct.vects.Throttle_To_Brake = [FSM_Struct.transitions.Throttle_To_Brake];
   FSM_Struct.vects.Throttle_To_Turn = [FSM_Struct.transitions.Throttle_To_Turn];
   
   % Brake Transitions--
   FSM_Struct.vects.Brake_Latch = [FSM_Struct.transitions.Brake_Latch];
   FSM_Struct.vects.Brake_To_Turn = [FSM_Struct.transitions.Brake_To_Turn];
   FSM_Struct.vects.Brake_To_Throttle = [FSM_Struct.transitions.Brake_To_Throttle];
   
   % Turn Transitions--
   FSM_Struct.vects.Turn_Latch = [FSM_Struct.transitions.Turn_Latch];
   FSM_Struct.vects.Turn_To_Brake = [FSM_Struct.transitions.Turn_To_Brake];
   FSM_Struct.vects.Turn_To_Throttle = [FSM_Struct.transitions.Turn_To_Throttle];
end

function [FSM_struct,LapTimeStruct] = FSM_Driver(LapTimeStruct,FSM_struct,Car)
    % Block #1: Process Inputs
        TooFast = AreWeTooFast(LapTimeStruct.S_dot_car,LapTimeStruct.S_dot_crit_brake);
        Turning = AreWeInATurn(abs(LapTimeStruct.K_here));
        
    % Block #2: Evaluate Logic
        % Throttle Transitions
        FSM_struct.transitions.Throttle_Latch = all(FSM_struct.states.Throttle && ~Turning && ~TooFast);
        FSM_struct.transitions.Throttle_To_Brake = FSM_struct.states.Throttle && TooFast;
        FSM_struct.transitions.Throttle_To_Turn = FSM_struct.states.Throttle && ~TooFast && Turning;
        
        % Brake Transitions
        FSM_struct.transitions.Brake_Latch = FSM_struct.states.Brake && TooFast;
        FSM_struct.transitions.Brake_To_Turn = FSM_struct.states.Brake && Turning && ~TooFast;
        FSM_struct.transitions.Brake_To_Throttle = FSM_struct.states.Brake && ~Turning && ~TooFast;
        
        % Turn Transitions
        FSM_struct.transitions.Turn_Latch = FSM_struct.states.Turn && ~TooFast && Turning;
        FSM_struct.transitions.Turn_To_Throttle = FSM_struct.states.Turn && ~TooFast && ~Turning;
        FSM_struct.transitions.Turn_To_Brake = FSM_struct.states.Turn && TooFast;
        
    % Block #3: Set States
        FSM_struct.states.Throttle = FSM_struct.transitions.Throttle_Latch || FSM_struct.transitions.Turn_To_Throttle || FSM_struct.transitions.Brake_To_Throttle;
        FSM_struct.states.Brake = FSM_struct.transitions.Brake_Latch || FSM_struct.transitions.Throttle_To_Brake || FSM_struct.transitions.Turn_To_Brake;
        FSM_struct.states.Turn = FSM_struct.transitions.Turn_Latch || FSM_struct.transitions.Throttle_To_Turn || FSM_struct.transitions.Brake_To_Turn;
    
    % Block #4: Updates Outputs
        if FSM_struct.states.Turn
            LapTimeStruct.S_double_dot_car = 0;
        end
        
        %LapTimeStruct.TurnBoolVect = [LapTimeStruct.TurnBoolVect; LapTimeStruct.bool_turn];
        %LapTimeStruct.TooFastBoolVect = [LapTimeStruct.TooFastBoolVect; LapTimeStruct.bool_to_fast]; 

        if FSM_struct.states.Throttle
            [LapTimeStruct.S_double_dot_car, LapTimeStruct.Tm, LapTimeStruct.Omega_m]= EngineModel(Car,LapTimeStruct.S_dot_car); % Engine Model Created by Harry Ma '22'
            %LapTimeStruct.S_double_dot_car = 0.3*g;
            LapTimeStruct.S_double_dot_car_vect = [LapTimeStruct.S_double_dot_car_vect; LapTimeStruct.S_double_dot_car];
            LapTimeStruct.Omega_m_vect = [LapTimeStruct.Omega_m_vect; LapTimeStruct.Omega_m];
            LapTimeStruct.Tm_vect = [LapTimeStruct.Tm_vect; LapTimeStruct.Tm];
        end
        
        if FSM_struct.states.Brake
            LapTimeStruct.S_double_dot_car = LapTimeStruct.a_brake_max;
            LapTimeStruct.S_double_dot_car_vect = [LapTimeStruct.S_double_dot_car_vect; LapTimeStruct.a_brake_max];
            LapTimeStruct.Tm_vect = [LapTimeStruct.Tm_vect; 0];
            LapTimeStruct.Omega_m_vect = [LapTimeStruct.Omega_m_vect; 0];
        end
        
        if FSM_struct.states.Turn
            LapTimeStruct.S_double_dot_car = 0;
            LapTimeStruct.S_double_dot_car_vect = [LapTimeStruct.S_double_dot_car_vect; LapTimeStruct.S_double_dot_car];
            LapTimeStruct.Tm_vect = [LapTimeStruct.Tm_vect; 0];
            LapTimeStruct.Omega_m_vect = [LapTimeStruct.Omega_m_vect; 0];
        end

        % Calculate current velocity of car and determine if we're going beyond maximum speed as determined by engine model
        LapTimeStruct.S_dot_car = Integrator(LapTimeStruct.S_dot_car,LapTimeStruct.S_double_dot_car,LapTimeStruct.dt);
        if LapTimeStruct.S_dot_car >= Car.engine.S_dot_max
            LapTimeStruct.S_dot_car = Car.engine.S_dot_max; % m/s
        end
        LapTimeStruct.S_dot_car_vect = [LapTimeStruct.S_dot_car_vect; LapTimeStruct.S_dot_car]; % append vector to include most recent vel
    % Super Block #4 (Update Transition Vectors):
    
        % Update our vectors of states
        FSM_struct.vects.Throttle = [FSM_struct.vects.Throttle; FSM_struct.states.Throttle];
        FSM_struct.vects.Brake = [FSM_struct.vects.Brake; FSM_struct.states.Brake];
        FSM_struct.vects.Turn = [FSM_struct.vects.Turn; FSM_struct.states.Turn];
%         % Throttle Transitions--
    
%         FSM_struct.vects.Throttle_Latch = [FSM_struct.vects.Throttle_Latch; FSM_struct.transitions.Throttle_Latch];
%         FSM_struct.vects.Throttle_To_Brake = [FSM_struct.vects.Throttle_To_Brake; FSM_struct.transitions.Throttle_To_Brake];
%         FSM_struct.vects.Throttle_To_Turn = [FSM_struct.vects.Throttle_To_Turn; FSM_struct.transitions.Throttle_To_Turn];

%         % Brake Transitions--
%         FSM_struct.vects.Brake_Latch = [FSM_struct.vects.Brake_Latch; FSM_struct.transitions.Brake_Latch];
%         FSM_struct.vects.Brake_To_Turn = [FSM_struct.vects.Brake_To_Turn; FSM_struct.transitions.Brake_To_Turn];
%         FSM_struct.vects.Brake_To_Throttle = [FSM_struct.vects.Brake_To_Throttle; FSM_struct.transitions.Brake_To_Throttle];

%         % Turn Transitions--
%         FSM_struct.vects.Turn_Latch = [FSM_struct.vects.Turn_Latch; FSM_struct.transitions.Turn_Latch];
%         FSM_struct.vects.Turn_To_Brake = [FSM_struct.vects.Turn_To_Brake; FSM_struct.transitions.Turn_To_Brake];
%         FSM_struct.vects.Turn_To_Throttle = [FSM_struct.vects.Turn_To_Throttle; FSM_struct.transitions.Turn_To_Throttle];

end

function [LapTimeStruct,FSM_struct] = GetLapTime(Car,track)
    % Initialize needed scalars and vectors
    LapTimeStruct = initializeLapTimeSim();
    FSM_struct = FSM_initialize();
    % Time stuff:
    LapTimeStruct.t_sim = 0; % Zero out simulated time
    LapTimeStruct.dt = 0.01; % time step for integrator
    Car.dt = LapTimeStruct.dt;
    g = 9.81; %m/s^2

    % Constant
    LapTimeStruct.a_y_max = 1.0*g;
    LapTimeStruct.a_turn_max = 1.3*g;
    LapTimeStruct.a_brake_max = -1.0*g;
    LapTimeStruct.a_engine_max = 0.3*g;
    LapTimeStruct.S_car_max = 30.1; %m/s 
    LapTimeStruct.fy_max = 1; % constant value for right now will be replaced by functionalized SS notebook

    while LapTimeStruct.S_car<=track.S(length(track.S))

        LapTimeStruct.K_here = K_hereFunc(track.S,track.K,LapTimeStruct.S_car);
        [LapTimeStruct.K_next,LapTimeStruct.dist_to_next_turn] = NextCurviture(track.S,track.K,LapTimeStruct.S_car);
        %dist_to_next_turn

        % SS Notekbook here
        %[Umax,aymax,delta,V] = findMaxSpeed(Car,mumax,R,Uthresh)
        LapTimeStruct.S_dot_crit = CritSpeed(LapTimeStruct.K_next,LapTimeStruct.fy_max);
        LapTimeStruct.S_dot_crit_vect = [LapTimeStruct.S_dot_crit_vect; LapTimeStruct.S_dot_crit];
        LapTimeStruct.S_dot_crit_brake = CritBrake(LapTimeStruct.S_dot_crit,LapTimeStruct.a_y_max,LapTimeStruct.dist_to_next_turn);
        LapTimeStruct.S_dot_crit_brake_vect = [LapTimeStruct.S_dot_crit_brake_vect; LapTimeStruct.S_dot_crit_brake];
%         LapTimeStruct.bool_turn = AreWeInATurn(abs(LapTimeStruct.K_here));
%         LapTimeStruct.bool_to_fast = AreWeTooFast(LapTimeStruct.S_dot_car,LapTimeStruct.S_dot_crit_brake);

        [FSM_struct,LapTimeStruct] = FSM_Driver(LapTimeStruct,FSM_struct,Car);
        % integrate to update time and position
        LapTimeStruct.S_car = Integrator(LapTimeStruct.S_car,LapTimeStruct.S_dot_car,LapTimeStruct.dt); 
        LapTimeStruct.S_car_vect = [LapTimeStruct.S_car_vect; LapTimeStruct.S_car];
        LapTimeStruct.t_sim = LapTimeStruct.t_sim+LapTimeStruct.dt;
        LapTimeStruct.t_sim_vect = [LapTimeStruct.t_sim_vect; LapTimeStruct.t_sim];
    end 
    FSM_struct.vects.Throttle=FSM_struct.vects.Throttle(2:end);
    FSM_struct.vects.Brake=FSM_struct.vects.Brake(2:end);
    FSM_struct.vects.Turn=FSM_struct.vects.Turn(2:end);
end

function LapTimeStruct = GetEnergy(LapTimeStruct)
    LapTimeStruct.Power = LapTimeStruct.Omega_m_vect.*0.10472.*LapTimeStruct.Tm_vect; 
    LapTimeStruct.Energy(1) = 0;
    for i = 2:length(LapTimeStruct.Omega_m_vect)
        LapTimeStruct.Energy(i) = Integrator(LapTimeStruct.Energy(i-1),LapTimeStruct.Power(i), LapTimeStruct.dt);
    end
end
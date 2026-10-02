function C = droneconfig()
%DRONECONFIG  Single source of truth for the MATLAB implementation.
%
%   Mirrors ../drone_sim/config.py value for value. The address book, the
%   marker geometry, the controller gains and the mission gates all live
%   here, so the world and the code that flies in it can never disagree.
%
%   Provenance tags:
%     [PAPER] Li, Chen, Lu, Wu, Cheng, "UAV Autonomous Landing Technology
%             Based on AprilTags Vision Positioning Algorithm", 38th CCC, 2019
%     [DJI]   DJI Matrice 100 class value
%     [MODEL] this project's modelling choice

% ------------------------------------------------------------ timing
C.DT        = 0.01;     % [MODEL] control step, s (100 Hz)
C.CAM_HZ    = 25;       % [MODEL] onboard camera frame rate
C.MAX_MISSION_TIME = 180;

% ------------------------------------------------------- airframe [DJI]
C.MASS    = 3.5;
C.GRAV    = 9.81;
C.HOVER   = C.MASS * C.GRAV;
C.MAXTHR  = 2.2 * C.HOVER;
C.MAXTILT = deg2rad(25);
C.J       = [0.045 0.045 0.080];

% ------------------------------------------------------------- camera
C.IMG_W = 640;  C.IMG_H = 480;
C.FOVY  = 60;
% The field of view is defined over the image HEIGHT, so the focal
% length in pixels is (h/2) / tan(fovy/2).
C.FOCAL   = (C.IMG_H/2) / tan(deg2rad(C.FOVY)/2);
C.CAMDROP = 0.12;       % camera sits this far below the body origin, m
C.MIN_TAG_PX = 20;      % [MODEL] smallest decodable tag edge, px
C.QUIET   = 1.25;       % tag + mandatory white border, as a factor on the edge

% ---------------------------------------------- dual-scale landing pad
C.TAG_LARGE  = 0.42;    % [PAPER-style] large tag edge, m
C.TAG_SMALL  = 0.12;    % [PAPER-style] small tag edge, m
C.TAG_OFFSET = 0.40;    % large tag centre offset from pad centre, m
C.PAD_SIZE   = 1.50;

% The modelled detector's error, calibrated against the Python build,
% which runs a REAL AprilTag detector on rendered frames and measures it
% against MuJoCo ground truth: 7.15 cm mean over 90 random poses, 9.02 cm
% on the large tag and 3.99 cm on the small one. Error grows with slant
% range because the same pixel of corner error subtends more ground.
C.VIS_SIGMA_A = 0.010;
C.VIS_SIGMA_B = 0.011;
C.HEAD_SIGMA  = deg2rad(2.7);   % pad-heading measurement noise
C.HEAD_SLERP  = 0.35;           % SLERP filter constant on that heading

% ------------------------------------------- guidance, eq. (9) [PAPER]
C.KP = 0.20;  C.KI = 0.03;  C.KD = 0.35;
C.I_CLAMP   = 1.2;
C.I_BAND    = 0.6;
C.D_TAU     = 0.12;
C.I_LEAK_TAU= 0.6;
C.GAIN_LOW  = 3.0;
C.GAIN_HI_ALT = 4.0;
C.GAIN_LO_ALT = 0.6;
C.AB_ALPHA  = 0.35;
C.AB_BETA   = 0.08;
C.AB_GAMMA  = 0.010;
C.LEAD_TIME = 0.25;
C.OMEGA_MAX       = 1.5;
C.OMEGA_TAU       = 0.35;
C.OMEGA_PERP_MIN  = 0.55;
C.OMEGA_MIN_SPEED = 0.30;

% --------------------------------------------------------- inner loops
C.KP_VEL = 1.6;  C.KI_VEL = 1.2;  C.VI_CLAMP = 0.6;
C.KP_VZ  = 2.4;
C.KP_ATT = 13.0; C.KD_RATE = 2.2;
C.KP_YAW = 3.0;  C.KD_YAW  = 1.2;
C.VSP_TAU = 0.08;

% ------------------------------------------------------ flight envelope
C.CRUISE_ALT   = 8.0;
C.V_MAX_CRUISE = 6.0;
C.V_MAX_TRACK  = 3.0;
C.V_MAX_VERT   = 1.4;

% -------------------------------------------------------- mission gates
C.ALIGN_TOL    = 0.45;
C.ALIGN_HOLD   = 0.7;
C.CONE_SLOPE   = 0.22;
C.DESCEND_ABORT= 1.00;
C.FINAL_ALT    = 0.45;
C.FINAL_SINK   = 0.35;
C.FINAL_TOL    = 0.08;
C.FINAL_TOL_K  = 0.14;
C.FINAL_TOL_MAX= 0.18;
C.LOST_TIMEOUT = 1.2;
C.COMMIT_HOLD  = 0.20;
C.MAX_GO_AROUNDS = 2;

% --------------------------------------------------------------- beacon
C.BEACON_SIGMA = 2.5;
C.BEACON_HZ    = 2.0;

% ----------------------------------------------------------------- wind
C.WIND_MEAN = [0.6 -0.4];
C.WIND_SIGMA= 0.5;
C.WIND_TAU  = 1.5;
C.DRAG      = 0.28;

C.HOME_XY = [0.0 14.5];

% ======================================================= address book
% Three kinds of landing site, all addressed the same way:
%   ground   - a pad on the street surface
%   vehicle  - a pad on the deck of a vehicle that keeps moving
%   roof     - a pad on a building roof, "height" metres up
%
% "height" is the height of the SURFACE the pad sits on. Everything
% downstream - the search altitude, the approach cone, the gain schedule,
% the commit height, the touchdown test - is measured against THIS and
% never against the street.
P = struct('id',{},'name',{},'kind',{},'home',{},'motion',{}, ...
           'speed',{},'heading',{},'half',{},'radius',{},'omega',{},'height',{});

P(1)  = mk(1,'A-01  Street drop point','ground', [-1.6  11.0],'static',0,0,0,0,0,0);
P(2)  = mk(2,'A-02  Loading bay',      'ground', [ 1.7   7.5],'static',0,0,0,0,0,0);
P(3)  = mk(3,'B-03  Courier van',      'vehicle',[-1.5   3.0],'line',0.9,deg2rad(90),3.0,0,0,0);
P(4)  = mk(4,'B-04  Clinic entrance',  'ground', [ 1.8   0.0],'static',0,0,0,0,0,0);
P(5)  = mk(5,'C-05  Service rover',    'vehicle',[-1.0  -3.5],'circle',0,0,0,0.9,0.60,0);
P(6)  = mk(6,'C-06  Apartment roof, 2F','roof',  [ 6.8  -6.0],'static',0,0,0,0,0,3.2);
P(7)  = mk(7,'D-07  Utility cart',     'vehicle',[-1.7 -10.5],'line',0.7,deg2rad(75),2.2,0,0,0);
P(8)  = mk(8,'D-08  Apartment roof, 3F','roof',  [-6.8 -13.0],'static',0,0,0,0,0,5.0);
P(9)  = mk(9,'E-09  Tower roof, 5F',   'roof',   [ 7.0 -18.5],'static',0,0,0,0,0,7.5);
P(10) = mk(10,'E-10  Kerbside locker', 'ground', [ 1.8 -20.0],'static',0,0,0,0,0,0);

for k = 1:numel(P)
    % a vehicle deck stands above the road; a roof stands at "height"
    if strcmp(P(k).kind,'vehicle'), P(k).deck = P(k).height + 0.34;
    else,                           P(k).deck = P(k).height + 0.022;
    end
end
C.PADS = P;
C.MOVING = [P(strcmp({P.motion},'static')==0).id];
end

function p = mk(id,name,kind,home,motion,speed,heading,half,radius,omega,height)
p = struct('id',id,'name',name,'kind',kind,'home',home,'motion',motion, ...
           'speed',speed,'heading',heading,'half',half,'radius',radius, ...
           'omega',omega,'height',height);
end

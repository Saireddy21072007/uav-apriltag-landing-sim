function [res, L] = simulate_landing(padId, seed, opts)
%SIMULATE_LANDING  Fly one mission to one AirTag address.
%
%   [res, L] = SIMULATE_LANDING(padId, seed, opts)
%
%   The mission:
%     TAKEOFF   climb to search altitude
%     TRANSIT   fly to the addressed pad's radio fix        (radio guidance)
%     SEARCH    expanding spiral when the radio fix was stale
%     ALIGN     hold altitude, drive the vision offset to zero   (vision)
%     DESCEND   descend only while inside the approach cone
%     FINAL     committed descent, motors cut on contact
%     LANDED / ABORTED
%
%   Guidance authority passes from radio to vision exactly once, at the
%   moment the addressed pad's marker first decodes. The drone will not
%   accept any other pad's marker: the ten pads carry ten different ids.
%
%   opts (all optional):
%     .feedforward  pad-velocity feed-forward from the tracker   [true]
%     .gainSchedule altitude gain scheduling                     [true]
%     .antiWindup   integral leak + unscaled trim                [true]
%     .tracker      'abg' constant acceleration | 'ct' coordinated turn ['ct']
%     .wind         steady wind plus Ornstein-Uhlenbeck gusts    [true]
%
%   PERCEPTION. MATLAB renders no camera, so the DETECTOR is modelled
%   rather than run: a tag decodes when its apparent edge exceeds
%   MIN_TAG_PX and its quiet zone fits inside the field of view - exactly
%   the two inequalities the Python build asserts in check_geometry() -
%   and the noise is calibrated to the error that build measures against
%   MuJoCo ground truth. Everything downstream of the measurement is the
%   same algorithm.

if nargin < 2 || isempty(seed), seed = 0; end
if nargin < 3, opts = struct(); end
opts = fill(opts, 'feedforward',  true);
opts = fill(opts, 'gainSchedule', true);
opts = fill(opts, 'antiWindup',   true);
opts = fill(opts, 'tracker',      'ct');
opts = fill(opts, 'wind',         true);

C = droneconfig();
rng(seed, 'twister');

pad0 = C.PADS([C.PADS.id] == padId);
deck = pad0.deck;

% ------------------------------------------------------------- state
s.pos = [C.HOME_XY 0.28];  s.vel = [0 0 0];
s.q = quatlib.identity();  s.omega = [0 0 0];
s.wind = C.WIND_MEAN;      s.armed = true;
s.vi = [0 0];              s.vspf = [0 0 0];

g = struct('int',[0 0],'lastErr',[0 0],'dState',[0 0],'started',false);
k = struct('have',false,'p',[0 0],'v',[0 0],'a',[0 0],'w',0,'since',0);

m = struct('state','TAKEOFF','t',0,'aligned',0,'sinceVis',99, ...
           'held',[0 0],'haveHeld',false,'switched',false,'goArounds',0, ...
           'searchT',0,'fix',[0 0],'haveFix',false, ...
           'headQ',quatlib.identity(),'camAcc',0,'beacon',0, ...
           'marker','none','cruise',deck + C.CRUISE_ALT);

nmax = ceil(C.MAX_MISSION_TIME / C.DT) + 10;
L = struct('t',zeros(nmax,1),'z',zeros(nmax,1),'agl',zeros(nmax,1), ...
           'err',zeros(nmax,1),'meas',nan(nmax,1),'seen',zeros(nmax,1), ...
           'x',zeros(nmax,1),'y',zeros(nmax,1),'padx',zeros(nmax,1), ...
           'pady',zeros(nmax,1),'tilt',zeros(nmax,1),'q',zeros(nmax,4), ...
           'deck',zeros(nmax,1));
L.state = cell(nmax,1);
n = 0;

while ~strcmp(m.state,'LANDED') && ~strcmp(m.state,'ABORTED')
    [pp, pv, pspeed] = padAt(pad0, m.t);

    k = trackPredict(C, k, opts);
    [m, k, live] = senseStep(C, s, m, k, pp, pv, pspeed, deck);
    [m, g, k, vsp] = missionStep(C, s, m, g, k, pp, deck, pspeed, live, opts);

    a = C.DT / (C.VSP_TAU + C.DT);
    s.vspf = s.vspf + a * (vsp - s.vspf);

    [thrust, alpha] = innerLoop(C, s, s.vspf, opts);
    s = integrateStep(C, s, thrust, alpha, opts, pad0, m.t);

    n = n + 1;
    bz = quatlib.bodyZ(s.q);
    L.t(n)=m.t; L.z(n)=s.pos(3); L.agl(n)=s.pos(3)-deck;
    L.err(n)=norm(pp - s.pos(1:2)); L.x(n)=s.pos(1); L.y(n)=s.pos(2);
    L.padx(n)=pp(1); L.pady(n)=pp(2); L.seen(n)=live;
    L.tilt(n)=rad2deg(acos(max(-1,min(1,bz(3)))));
    L.q(n,:)=s.q; L.deck(n)=deck;
    L.state{n}=m.state;
    if m.haveHeld, L.meas(n)=norm(m.held); end

    m.t = m.t + C.DT;
    if m.t > C.MAX_MISSION_TIME, m.state = 'ABORTED'; end
end

f = {'t','z','agl','err','meas','seen','x','y','padx','pady','tilt','deck'};
for i = 1:numel(f), L.(f{i}) = L.(f{i})(1:n); end
L.q = L.q(1:n,:);
L.state = L.state(1:n);

pp = padAt(pad0, m.t);
res = struct('outcome', m.state, 'padId', padId, 'name', pad0.name, ...
             'moving', ~strcmp(pad0.motion,'static'), 'time_s', m.t, ...
             'error_m', norm(pp - s.pos(1:2)), 'touchdown_vz', s.vel(3), ...
             'handover', m.switched, 'goArounds', m.goArounds);
end

function o = fill(o, name, val)
if ~isfield(o, name), o.(name) = val; end
end

% =====================================================================
%  PADS
% =====================================================================
function [p, v, speed] = padAt(pad, t)
switch pad.motion
    case 'static'
        p = pad.home; v = [0 0];
    case 'line'
        % back and forth along a fixed heading, so the vehicle stops and
        % REVERSES at each end - the case that breaks a naive turn model
        d  = [cos(pad.heading) sin(pad.heading)];
        p  = pad.home + pad.half * sin(pad.speed*t/pad.half) * d;
        v  = pad.speed * cos(pad.speed*t/pad.half) * d;
    case 'circle'
        w = pad.omega; r = pad.radius;
        p = pad.home + r*[cos(w*t)-1, sin(w*t)];
        v = r*w*[-sin(w*t), cos(w*t)];
    otherwise
        error('unknown motion %s', pad.motion);
end
speed = norm(v);
end

% =====================================================================
%  PERCEPTION - the detector, modelled
% =====================================================================
function [m, k, live] = senseStep(C, s, m, k, pp, pv, pspeed, deck)
m.camAcc = m.camAcc + C.DT;
if m.camAcc < 1/C.CAM_HZ
    m = deadReckon(C, m, k, s);
    live = m.sinceVis < 1.5/C.CAM_HZ;
    return
end
m.camAcc = 0;
fresh = false;

hc = s.pos(3) - deck - C.CAMDROP;
d  = pp - s.pos(1:2);
centreOff = norm(d);

if pspeed > 0.05, pyaw = atan2(pv(2), pv(1)); else, pyaw = 0; end
lp = pp + C.TAG_OFFSET*[cos(pyaw) sin(pyaw)];
largeOff = norm(lp - s.pos(1:2));

if tagVisible(C, C.TAG_SMALL, centreOff, hc)
    % The smaller tag wins whenever available: it is closer to the pad
    % centre and its pose is the more accurate one at the altitude where
    % it can be seen at all. The paper's multi-scale rule, made explicit.
    sg = C.VIS_SIGMA_A + C.VIS_SIGMA_B * hypot(centreOff, hc);
    m.held = d + sg*randn(1,2);
    m.haveHeld = true; m.sinceVis = 0; fresh = true;
    m.marker = 'small'; m.switched = true;

elseif tagVisible(C, C.TAG_LARGE, largeOff, hc)
    sg  = C.VIS_SIGMA_A + C.VIS_SIGMA_B * hypot(largeOff, hc);
    tag = (lp - s.pos(1:2)) + sg*randn(1,2);
    % The large tag gives the position of the TAG, not of the pad. The
    % correction is its known offset rotated by the MEASURED pad heading,
    % and a few degrees of heading noise becomes centimetres of position
    % error through that 0.40 m lever arm - which is what the SLERP
    % filter is for. Averaging the angle directly would smooth a turning
    % vehicle's heading the long way round every time it crossed +-180.
    yawMeas = pyaw + C.HEAD_SIGMA*randn();
    m.headQ = quatlib.slerp(m.headQ, quatlib.fromYaw(yawMeas), C.HEAD_SLERP);
    yf = quatlib.yawOf(m.headQ);
    m.held = tag - C.TAG_OFFSET*[cos(yf) sin(yf)];
    m.haveHeld = true; m.sinceVis = 0; fresh = true;
    m.marker = 'large';
else
    m = deadReckon(C, m, k, s);
    m.marker = 'none';
end

m.beacon = m.beacon + 1/C.CAM_HZ;
if m.beacon >= 1/C.BEACON_HZ
    m.beacon = 0;
    m.fix = pp + C.BEACON_SIGMA*randn(1,2);
    m.haveFix = true;
end

if fresh
    k = trackObserve(C, k, s.pos(1:2) + m.held);
    m.fix = s.pos(1:2) + m.held;
    m.haveFix = true;
end
live = m.sinceVis < 1.5/C.CAM_HZ;
end

function m = deadReckon(C, m, k, s)
m.sinceVis = m.sinceVis + C.DT;
if m.haveHeld
    m.held = m.held + (k.v - s.vel(1:2)) * C.DT;
end
end

function tf = tagVisible(C, sz, off, hc)
% Two questions, and they are the inequalities check_geometry() asserts:
%   1. is the tag big enough in the image?  f*size/range >= MIN_TAG_PX
%   2. does it FIT in the frame?
if hc <= 0.02, tf = false; return; end
if C.FOCAL * sz / hypot(off, hc) < C.MIN_TAG_PX, tf = false; return; end
tf = (off + sz*C.QUIET/2) <= fovHalfwidth(C, hc);
end

function h = fovHalfwidth(C, cameraHeight)
h = cameraHeight * (C.IMG_H/2) / C.FOCAL;
end

function h = blindAltitude(C, off)
% Height above the deck below which a marker seen from this far
% off-centre no longer fits in the frame. The exact inverse of
% fovHalfwidth, and the drone's decision height.
h = C.CAMDROP + (C.TAG_SMALL*C.QUIET/2 + abs(off)) / ((C.IMG_H/2)/C.FOCAL);
end

function tol = commitTol(C, padSpeed)
% A single number cannot serve both cases: demand 8 cm on a turning pad
% and the drone never commits at all; allow 16 cm on a stationary one and
% it lands three times less accurately than it could.
tol = min(C.FINAL_TOL + C.FINAL_TOL_K*abs(padSpeed), C.FINAL_TOL_MAX);
end

% =====================================================================
%  TARGET TRACKER - alpha-beta-gamma, with a coordinated-turn prediction
% =====================================================================
function k = trackPredict(C, k, opts)
if k.have
    if strcmp(opts.tracker,'ct')
        k.w = k.w + (C.DT/(C.OMEGA_TAU+C.DT)) * (turnRate(C,k) - k.w);
        ang = k.w * C.DT;
        if abs(ang) < 1e-4
            k.p = k.p + k.v*C.DT + 0.5*k.a*C.DT^2;
            k.v = k.v + k.a*C.DT;
        else
            % Exact integral of a velocity that rotates at w: the target
            % travels along the ARC, not along the tangent. A constant-
            % acceleration model can only extrapolate a straight line
            % plus a fixed bend, and a pad driving a steady circle curves
            % away from that continuously.
            c = cos(ang); sn = sin(ang);
            k.p = k.p + [(sn*k.v(1) - (1-c)*k.v(2))/k.w, ...
                         ((1-c)*k.v(1) + sn*k.v(2))/k.w];
            k.v = [c*k.v(1)-sn*k.v(2), sn*k.v(1)+c*k.v(2)];
            k.a = [c*k.a(1)-sn*k.a(2), sn*k.a(1)+c*k.a(2)];
        end
    else
        k.p = k.p + k.v*C.DT + 0.5*k.a*C.DT^2;
        k.v = k.v + k.a*C.DT;
    end
end
k.since = k.since + C.DT;
end

function w = turnRate(C, k)
% For ANY planar motion the part of the acceleration perpendicular to the
% velocity IS the turn:  omega = (v x a)_z / |v|^2 - so no new
% measurement is needed, it falls out of two states the filter already
% carries. It is believed only when the acceleration really is
% perpendicular: |sin| of the angle between them is 1 for a pad driving a
% steady circle and 0 for one braking in a straight line. Without that
% deadband the model is actively harmful on the pads that shuttle back
% and forth - at each end the vehicle stops and reverses, |v| passes
% through zero, and a cross product divided by |v|^2 reports a violent
% corner where there is only a straight stop.
s2 = k.v*k.v';
if s2 < C.OMEGA_MIN_SPEED^2, w = 0; return; end
cross = k.v(1)*k.a(2) - k.v(2)*k.a(1);
aMag = norm(k.a);
if aMag < 1e-6, w = 0; return; end
perp = abs(cross) / (sqrt(s2)*aMag);
if perp < C.OMEGA_PERP_MIN, w = 0; return; end
ramp = min(1, (perp - C.OMEGA_PERP_MIN)/(1 - C.OMEGA_PERP_MIN));
w = max(-C.OMEGA_MAX, min(C.OMEGA_MAX, cross/s2 * ramp));
end

function k = trackObserve(C, k, meas)
if ~k.have
    k.p = meas; k.v = [0 0]; k.a = [0 0]; k.w = 0; k.have = true;
else
    dt = max(k.since, 1e-3);
    resid = meas - k.p;
    k.p = k.p + C.AB_ALPHA*resid;
    k.v = max(-4, min(4, k.v + (C.AB_BETA/dt)*resid));
    aNew = k.a + (2*C.AB_GAMMA/dt^2)*resid;
    k.a = max(-2.5, min(2.5, 0.75*k.a + 0.25*aNew));
end
k.since = 0;
end

function v = leadVelocity(C, k, opts)
% On a turn the lead is a ROTATION of the velocity, not an addition to
% it - adding acceleration for a quarter of a second points the drone at
% the tangent, off the outside of the corner.
if strcmp(opts.tracker,'ct') && abs(k.w) > 1e-4
    ang = k.w * C.LEAD_TIME; c = cos(ang); sn = sin(ang);
    v = [c*k.v(1)-sn*k.v(2), sn*k.v(1)+c*k.v(2)];
else
    v = k.v + k.a*C.LEAD_TIME;
end
end

% =====================================================================
%  GUIDANCE - eq. (9) at the paper's published gains, with five changes
% =====================================================================
function gn = altitudeGain(C, agl, opts)
if ~opts.gainSchedule, gn = 1; return; end
if agl >= C.GAIN_HI_ALT, gn = 1; return; end
if agl <= C.GAIN_LO_ALT, gn = C.GAIN_LOW; return; end
f = (C.GAIN_HI_ALT - agl)/(C.GAIN_HI_ALT - C.GAIN_LO_ALT);
gn = 1 + f*(C.GAIN_LOW - 1);
end

function [g, v] = guidanceVel(C, g, k, off, agl, opts)
gn = altitudeGain(C, agl, opts);
v = [0 0];
for ax = 1:2
    err = off(ax);
    if ~g.started, g.lastErr(ax) = err; end

    % CONDITIONAL INTEGRATION. The integral exists to trim a steady wind
    % while the drone holds station over the pad; allowed to charge
    % during a twenty-metre transit it only buys overshoot.
    if abs(err) < C.I_BAND
        g.int(ax) = max(-C.I_CLAMP, min(C.I_CLAMP, g.int(ax) + err*C.DT));
    end
    % ...WITH A LEAK. A charge built while closing in from one side is
    % history, not trim. Left to unwind on its own it becomes the largest
    % term in the loop: the drone hovers over the pad, measures its ten
    % centimetre offset correctly, and flies the other way. On stationary
    % addresses this one rule is worth 12.5 cm -> 1.9 cm.
    if opts.antiWindup && g.int(ax)*err < 0
        g.int(ax) = g.int(ax) * max(0, 1 - C.DT/C.I_LEAK_TAU);
    end

    % RATE-NORMALISED, FILTERED DERIVATIVE. As printed the derivative has
    % no division by the sample period, so the same kd behaves completely
    % differently at every loop rate.
    raw = (err - g.lastErr(ax)) / max(C.DT, 1e-6);
    a = C.DT/(C.D_TAU + C.DT);
    g.dState(ax) = g.dState(ax) + a*(raw - g.dState(ax));
    g.lastErr(ax) = err;

    % The altitude schedule speeds up the RESPONSE as the camera's view
    % of the ground shrinks. It has no business rescaling the accumulated
    % trim, which is an estimate of a standing disturbance.
    if opts.antiWindup
        v(ax) = gn*(C.KP*err + C.KD*g.dState(ax)) + C.KI*g.int(ax);
    else
        v(ax) = gn*(C.KP*err + C.KI*g.int(ax) + C.KD*g.dState(ax));
    end
end
g.started = true;

% PAD-VELOCITY FEED-FORWARD. A PID chasing a moving pad is a PID chasing
% a ramp, and it keeps a steady-state lag - which is exactly the
% (-0.2, +0.5) m band the paper reports as its tracking result.
if opts.feedforward
    v = v + leadVelocity(C, k, opts);
end
nv = norm(v);
if nv > C.V_MAX_TRACK, v = v * C.V_MAX_TRACK/nv; end
end

function g = resetPid(g)
g.int = [0 0]; g.lastErr = [0 0]; g.dState = [0 0]; g.started = false;
end

% =====================================================================
%  MISSION STATE MACHINE
% =====================================================================
function [m, g, k, vsp] = missionStep(C, s, m, g, k, pp, deck, pspeed, live, opts)
vsp = [0 0 0];
agl = s.pos(3) - deck;
haveVision = (m.sinceVis < C.LOST_TIMEOUT) && m.haveHeld;
if m.haveHeld, off = m.held; else, off = [0 0]; end
err = norm(off);
climb = @(rate) min(rate, 1.1*(m.cruise - s.pos(3)));
holdAlt = @() max(-C.V_MAX_VERT, min(C.V_MAX_VERT, 1.1*(m.cruise - s.pos(3))));

switch m.state
case 'TAKEOFF'
    vsp(3) = holdAlt();
    if s.pos(3) > m.cruise - 0.4, m.state = 'TRANSIT'; end

case 'TRANSIT'
    vsp(3) = holdAlt();
    if live
        k.have = false; g = resetPid(g); m.state = 'ALIGN';
    elseif m.haveFix
        e = m.fix - s.pos(1:2);
        v = 0.8*e; nv = norm(v);
        if nv > C.V_MAX_CRUISE, v = v*C.V_MAX_CRUISE/nv; end
        vsp(1:2) = v;
        if norm(e) < 1.2, m.searchT = 0; m.state = 'SEARCH'; end
    end

case 'SEARCH'
    vsp(3) = holdAlt();
    m.searchT = m.searchT + C.DT;
    if live
        k.have = false; g = resetPid(g); m.state = 'ALIGN';
    else
        r = 0.5 + 0.30*m.searchT;
        want = m.fix + r*[cos(0.8*m.searchT) sin(0.8*m.searchT)];
        vsp(1:2) = max(-2.5, min(2.5, 1.1*(want - s.pos(1:2))));
        if m.searchT > 30, m.state = 'TRANSIT'; end
    end

case 'ALIGN'
    vsp(3) = holdAlt();
    if ~haveVision
        m.state = 'TRANSIT';
    else
        [g, v] = guidanceVel(C, g, k, off, agl, opts);
        vsp(1:2) = v;
        if err < C.ALIGN_TOL, m.aligned = m.aligned + C.DT; else, m.aligned = 0; end
        if m.aligned > C.ALIGN_HOLD, m.state = 'DESCEND'; end
    end

case 'DESCEND'
    tol = commitTol(C, pspeed);
    % DECISION HEIGHT. blindAltitude(err) is the height at which a marker
    % seen from this far off-centre stops fitting in the frame. Below it
    % the camera has nothing left to give, so the drone lands on the
    % solution it already holds. The trigger is the marker ACTUALLY going
    % away, held for a few frames - never a prediction that it is about
    % to. The geometry only answers the second question: was this loss
    % expected here? Reading it as a lost target instead is what produced
    % the go-around loop: climb eight metres, re-acquire, fly back down,
    % meet the identical geometry, until the mission times out.
    lostLow = (m.sinceVis > C.COMMIT_HOLD) && (agl <= blindAltitude(C, err)) ...
              && m.haveHeld;
    if lostLow
        m.state = 'FINAL';
    elseif ~haveVision
        % Losing the marker higher up is a real re-acquire, but it must
        % not be unbounded: an endless recovery is not a safety feature.
        m.goArounds = m.goArounds + 1;
        if m.goArounds > C.MAX_GO_AROUNDS && agl < 1.0 && m.haveHeld
            m.state = 'FINAL';
        else
            vsp(3) = climb(0.6); m.state = 'ALIGN';
        end
    else
        [g, v] = guidanceVel(C, g, k, off, agl, opts);
        vsp(1:2) = v;
        allowed = max(tol, C.CONE_SLOPE*agl);
        if ~live
            vsp(3) = 0;                       % never descend blind
        elseif err > C.DESCEND_ABORT
            vsp(3) = climb(0.5);
        elseif err > allowed
            vsp(3) = 0;                       % outside the cone: hold
        else
            q = max(0, 1 - err/allowed);
            vsp(3) = -max(0, min(C.V_MAX_VERT, 0.25 + 0.75*q));
            if agl < 1.2, vsp(3) = max(vsp(3), -0.45); end
        end
        if live && agl < C.FINAL_ALT && err < tol, m.state = 'FINAL'; end
    end

case 'FINAL'
    [g, v] = guidanceVel(C, g, k, off, agl, opts);
    vsp(1:2) = v;
    vsp(3) = -C.FINAL_SINK;
    % A go-around is only meaningful while the marker is still usable,
    % and only while the drone has any left to spend.
    if agl > blindAltitude(C, err) && m.goArounds <= C.MAX_GO_AROUNDS ...
            && err > 3*commitTol(C, pspeed)
        m.state = 'DESCEND';
    end
    if abs(agl - 0.21) < 0.09 && abs(s.vel(3)) < 0.25
        m.state = 'LANDED';
    end
end
end

% =====================================================================
%  INNER LOOP - velocity setpoint to thrust and torque, through a
%  desired ORIENTATION that is built, never decomposed
% =====================================================================
function [thrust, alpha] = innerLoop(C, s, vsp, opts)     %#ok<INUSD>
% A purely proportional velocity loop cannot hold station in a steady
% wind: holding against drag needs a standing lean, and the only way a P
% loop produces one is by keeping a standing velocity error of
% F_drag/(kp_vel*m) - about 3.6 cm/s here, the same order as the
% correction the vision loop asks for at touchdown height.
ev  = vsp(1:2) - s.vel(1:2);
acc = [C.KP_VEL*ev + C.KI_VEL*s.vi, C.KP_VZ*(vsp(3) - s.vel(3))];

aLat = norm(acc(1:2));
aMax = C.GRAV*tan(C.MAXTILT);
if aLat > aMax, acc(1:2) = acc(1:2)*aMax/aLat; end
acc(3) = max(-4, min(6, acc(3)));

f = C.MASS*[acc(1) acc(2) acc(3)+C.GRAV];
f(3) = max(f(3), 0.25*C.HOVER);
lean = norm(f(1:2)); maxLean = f(3)*tan(C.MAXTILT);
if lean > maxLean && maxLean > 0, f(1:2) = f(1:2)*maxLean/lean; end

% A quadrotor can only push along its own +z, so the desired body z axis
% IS the force direction. Three cross products and a normalise - no
% trigonometry, and nothing singular.
b3 = f(:)/max(norm(f),1e-9);
c1 = [cos(0); sin(0); 0];
b2 = cross(b3, c1); b2 = b2/max(norm(b2),1e-9);
b1 = cross(b2, b3);
qDes = quatlib.fromAxes(b1, b2, b3);

% The attitude error IS a rotation:  q_err = q^-1 * q_des
qErr = quatlib.mul(quatlib.conj(s.q), qDes);
e = quatlib.toRotvec(qErr);
alpha = 10*[C.KP_ATT*e(1) - C.KD_RATE*s.omega(1), ...
            C.KP_ATT*e(2) - C.KD_RATE*s.omega(2), ...
            C.KP_YAW*e(3) - C.KD_YAW*s.omega(3)];

% Thrust is the projection of the desired force onto the axis the drone
% is CURRENTLY pointing along, not the length of the desired force.
% Sizing it for an orientation the airframe has not reached yet leaves
% surplus lift, and the drone climbs away from its altitude setpoint
% whenever it manoeuvres hard.
bz = quatlib.bodyZ(s.q);
thrust = max(0, min(C.MAXTHR, dot(f(:), bz)));
end

% =====================================================================
%  PHYSICS
% =====================================================================
function s = integrateStep(C, s, thrust, alpha, opts, pad0, t)
% the velocity-loop integral, with anti-windup behind the tilt limiter
ev = s.vspf(1:2) - s.vel(1:2);
if norm([C.KP_VEL*ev + C.KI_VEL*s.vi]) < C.GRAV*tan(C.MAXTILT)
    s.vi = max(-C.VI_CLAMP, min(C.VI_CLAMP, s.vi + ev*C.DT));
end

if ~s.armed, thrust = 0; end

s.omega = s.omega + alpha*C.DT;
qd = quatlib.mul(s.q, [0 s.omega]);
s.q = quatlib.normalize(s.q + 0.5*qd*C.DT);

if opts.wind
    s.wind = s.wind + (C.DT/C.WIND_TAU)*(C.WIND_MEAN - s.wind) ...
             + C.WIND_SIGMA*sqrt(2*C.DT/C.WIND_TAU)*randn(1,2);
    w = s.wind;
else
    w = [0 0];
end

bz = quatlib.bodyZ(s.q)';
acc = (thrust/C.MASS)*bz - [0 0 C.GRAV] ...
      + [C.DRAG*(w - s.vel(1:2))/C.MASS, -C.DRAG*s.vel(3)/C.MASS];

s.vel = s.vel + acc*C.DT;
s.pos = s.pos + s.vel*C.DT;

floorH = surfaceUnder(C, s.pos(1:2), pad0, t);
if s.pos(3) <= floorH + 0.21
    s.pos(3) = floorH + 0.21;
    if s.vel(3) < 0, s.vel(3) = 0; end
    s.vel(1:2) = s.vel(1:2)*0.5;
end
end

function h = surfaceUnder(C, xy, pad0, t)      %#ok<INUSL>
% Height of the surface beneath a point: a roof if we are over one, a
% vehicle deck if we are over one, otherwise the road.
h = 0;
for i = 1:numel(C.PADS)
    p = C.PADS(i);
    pp = padAt(p, t);
    if strcmp(p.kind,'roof')
        if all(abs(xy - pp) < 2.2), h = max(h, p.deck); end
    elseif strcmp(p.kind,'vehicle')
        if all(abs(xy - pp) < 0.75), h = max(h, p.deck); end
    end
end
end

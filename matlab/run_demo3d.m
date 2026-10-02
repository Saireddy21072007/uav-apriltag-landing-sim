function run_demo3d(addresses, seed, gifFile)
%RUN_DEMO3D  Three-dimensional animation of the landing, MuJoCo style.
%
%   RUN_DEMO3D                            fly 6 -> 3 -> 9
%   RUN_DEMO3D([5], 0)                    just the circling rover
%   RUN_DEMO3D([6 3 9], 1, 'demo3d.gif')  ...and save the animation
%
%   What you are looking at, and why each piece is there:
%
%     * the drone is drawn as four rotor arms, rotated by the ACTUAL
%       attitude quaternion the controller is flying. When it tilts to
%       accelerate, you see it tilt - lateral motion is bought with
%       attitude, which is the whole reason the vehicle is underactuated.
%     * the yellow cone is the onboard camera's field of view, projected
%       from the camera's own axis onto the surface the pad sits on. It
%       swings when the drone tilts, because the camera is bolted to the
%       airframe. Watch it shrink on the descent: when the footprint
%       closes inside the marker, vision is finished and the drone
%       commits. That is the decision height.
%     * the black squares on each pad are the two markers, to scale:
%       0.42 m offset from the centre, 0.12 m at the centre.
%     * buildings are drawn from the address book's roof heights, so what
%       you see is what the mission code is flying against.

if nargin < 1 || isempty(addresses), addresses = [6 3 9]; end
if nargin < 2 || isempty(seed), seed = 1; end
if nargin < 3, gifFile = ''; end

C = droneconfig();
DRONE_SCALE = 4;    % see the note where the arms are drawn
fig = figure('Color',[0.09 0.10 0.12],'Position',[40 40 1180 720], ...
             'Visible', onoff(isempty(gifFile)));
ax = axes(fig,'Position',[0.04 0.06 0.60 0.86]); hold(ax,'on'); grid(ax,'on');
cax = axes(fig,'Position',[0.67 0.30 0.31 0.42]); hold(cax,'on');
set(cax,'Color',[0.06 0.07 0.09],'XColor',[0.5 0.5 0.55],'YColor',[0.5 0.5 0.55], ...
    'XTick',[],'YTick',[],'Box','on');
xlim(cax,[0 C.IMG_W]); ylim(cax,[0 C.IMG_H]); set(cax,'YDir','reverse');
daspect(cax,[1 1 1]);
camTtl = title(cax,'onboard camera','Color',[0.8 0.8 0.85],'FontSize',10);
% what the camera sees: the pad, then its two markers, then the crosshair
cPad = patch(cax,'XData',[0 0 0 0],'YData',[0 0 0 0],'FaceColor',[0.93 0.93 0.90], ...
             'EdgeColor',[0.1 0.6 0.9],'LineWidth',1.5);
cBig = patch(cax,'XData',[0 0 0 0],'YData',[0 0 0 0],'FaceColor','k','EdgeColor','none');
cSml = patch(cax,'XData',[0 0 0 0],'YData',[0 0 0 0],'FaceColor','k','EdgeColor','none');
plot(cax,[C.IMG_W/2-14 C.IMG_W/2+14],[C.IMG_H/2 C.IMG_H/2],'-','Color',[1 1 1 0.6]);
plot(cax,[C.IMG_W/2 C.IMG_W/2],[C.IMG_H/2-14 C.IMG_H/2+14],'-','Color',[1 1 1 0.6]);
cLock = text(cax,10,22,'','Color',[0.4 1 0.5],'FontSize',10,'FontWeight','bold');
set(ax,'Color',[0.12 0.13 0.16],'GridColor',[0.4 0.4 0.45], ...
       'XColor',[0.7 0.7 0.75],'YColor',[0.7 0.7 0.75],'ZColor',[0.7 0.7 0.75]);
xlabel(ax,'x (m)'); ylabel(ax,'y (m)'); zlabel(ax,'altitude (m)');
view(ax, -36, 22); daspect(ax,[1 1 1]);
camproj(ax,'perspective');
light(ax,'Position',[-8 20 25],'Style','infinite');

% ---- the street
surf(ax,[-3 3; -3 3],[-23 -23; 17 17],zeros(2),'FaceColor',[0.30 0.30 0.33], ...
     'EdgeColor','none');
surf(ax,[-11 -3; -11 -3],[-23 -23; 17 17],zeros(2),'FaceColor',[0.20 0.22 0.24], ...
     'EdgeColor','none');
surf(ax,[3 11; 3 11],[-23 -23; 17 17],zeros(2),'FaceColor',[0.20 0.22 0.24], ...
     'EdgeColor','none');

% ---- buildings, generated from the address book's heights
for i = 1:numel(C.PADS)
    p = C.PADS(i);
    if strcmp(p.kind,'roof')
        drawBox(ax, p.home(1), p.home(2), 0, 4.4, 4.4, p.height, ...
                [0.42 0.40 0.38]);
    end
end

% ---- pads, with their two markers drawn to scale
padH = gobjects(numel(C.PADS),1);
bigH = gobjects(numel(C.PADS),1);
smlH = gobjects(numel(C.PADS),1);
txtH = gobjects(numel(C.PADS),1);
for i = 1:numel(C.PADS)
    p = C.PADS(i);
    padH(i) = patch(ax,'XData',[0 0 0 0],'YData',[0 0 0 0],'ZData',[0 0 0 0], ...
                    'FaceColor',kindColour(p),'EdgeColor','k','LineWidth',0.5);
    bigH(i) = patch(ax,'XData',[0 0 0 0],'YData',[0 0 0 0],'ZData',[0 0 0 0], ...
                    'FaceColor','k','EdgeColor','none');
    smlH(i) = patch(ax,'XData',[0 0 0 0],'YData',[0 0 0 0],'ZData',[0 0 0 0], ...
                    'FaceColor','k','EdgeColor','none');
    txtH(i) = text(ax,0,0,0,sprintf('%d',p.id),'Color','w', ...
                   'FontWeight','bold','FontSize',10);
end

trailH = plot3(ax,NaN,NaN,NaN,'-','Color',[0.35 0.65 1.0],'LineWidth',1.3);
armH   = gobjects(2,1);
for a = 1:2
    armH(a) = plot3(ax,NaN,NaN,NaN,'-','Color','w','LineWidth',2.5);
end
rotorH = plot3(ax,NaN,NaN,NaN,'o','MarkerSize',9,'MarkerFaceColor',[1 0.85 0.3], ...
               'MarkerEdgeColor',[0.3 0.25 0.1],'LineStyle','none');
bodyH  = plot3(ax,NaN,NaN,NaN,'o','MarkerSize',7,'MarkerFaceColor',[0.9 0.95 1], ...
               'MarkerEdgeColor','k','LineStyle','none');
coneH  = gobjects(4,1);
for a = 1:4
    coneH(a) = plot3(ax,NaN,NaN,NaN,'-','Color',[1 0.85 0.25 0.55],'LineWidth',0.9);
end
footH  = plot3(ax,NaN,NaN,NaN,'-','Color',[1 0.85 0.25],'LineWidth',1.6);
dropH  = plot3(ax,NaN,NaN,NaN,':','Color',[0.6 0.6 0.65],'LineWidth',0.8);
ttl    = title(ax,'','Color','w','FontSize',11,'FontWeight','normal');

first = true;
for addr = addresses
    [res, L] = simulate_landing(addr, seed, struct());
    step = max(1, round(numel(L.t)/90));
    for n = 1:step:numel(L.t)
        t = L.t(n);
        % --- pads and their markers
        for i = 1:numel(C.PADS)
            p  = C.PADS(i);
            pp = padAtTime(p, t);
            z  = p.deck + 0.01;
            setSquare(padH(i), pp, z, C.PAD_SIZE, 0);
            yaw = padYaw(p, t);
            bc = pp + C.TAG_OFFSET*[cos(yaw) sin(yaw)];
            setSquare(bigH(i), bc, z+0.005, C.TAG_LARGE, yaw);
            setSquare(smlH(i), pp, z+0.005, C.TAG_SMALL, yaw);
            set(txtH(i),'Position',[pp(1)+0.95 pp(2) z+0.1]);
        end

        % --- the drone, drawn at its true attitude
        R = quatlib.toRot(L.q(n,:));
        c = [L.x(n) L.y(n) L.z(n)];
        % The airframe is 0.52 m across. Drawn true to scale it is a
        % couple of pixels at this zoom, so it is drawn at DRONE_SCALE
        % times life size - the only thing in this view that is not to
        % scale, and the attitude it is drawn at is the real one.
        Larm = 0.26 * DRONE_SCALE;
        a1 = c + (R*[ Larm;0;0])';  a2 = c + (R*[-Larm;0;0])';
        a3 = c + (R*[0; Larm;0])';  a4 = c + (R*[0;-Larm;0])';
        set(armH(1),'XData',[a1(1) a2(1)],'YData',[a1(2) a2(2)],'ZData',[a1(3) a2(3)]);
        set(armH(2),'XData',[a3(1) a4(1)],'YData',[a3(2) a4(2)],'ZData',[a3(3) a4(3)]);
        set(rotorH,'XData',[a1(1) a2(1) a3(1) a4(1)], ...
                   'YData',[a1(2) a2(2) a3(2) a4(2)], ...
                   'ZData',[a1(3) a2(3) a3(3) a4(3)]);
        set(bodyH,'XData',c(1),'YData',c(2),'ZData',c(3));
        set(trailH,'XData',L.x(1:n),'YData',L.y(1:n),'ZData',L.z(1:n));

        % --- the camera cone, projected along the camera's OWN axis
        deck = L.deck(n);
        b3   = R(:,3);                       % body z, world frame
        camC = c' - C.CAMDROP*b3;            % camera sits below the body
        axisv = -b3;                         % it looks along -body z
        if axisv(3) < -0.05
            s = (camC(3) - deck) / (-axisv(3));
            ctr = camC + s*axisv;
            half = s * (C.IMG_H/2)/C.FOCAL;
            ex = R(:,1)*half; ey = R(:,2)*half;
            corners = [ctr+ex+ey, ctr-ex+ey, ctr-ex-ey, ctr+ex-ey];
            for a = 1:4
                set(coneH(a),'XData',[camC(1) corners(1,a)], ...
                             'YData',[camC(2) corners(2,a)], ...
                             'ZData',[camC(3) corners(3,a)]);
            end
            cc = [corners corners(:,1)];
            col = pick(L.seen(n)>0, [0.35 0.95 0.45], [1 0.35 0.3]);
            set(footH,'XData',cc(1,:),'YData',cc(2,:),'ZData',cc(3,:),'Color',col);
        end
        set(dropH,'XData',[c(1) c(1)],'YData',[c(2) c(2)],'ZData',[deck c(3)]);

        % ---- the onboard camera: project the addressed pad and its two
        % markers through the same pinhole model the perception code uses
        pTgt = C.PADS([C.PADS.id] == addr);
        ppT  = padAtTime(pTgt, t);
        yawT = padYaw(pTgt, t);
        zT   = pTgt.deck + 0.01;
        camPos = c' - C.CAMDROP*b3;
        okPad = projectSquare(cPad, C, R, camPos, ppT, zT, C.PAD_SIZE, yawT);
        bcT = ppT + C.TAG_OFFSET*[cos(yawT) sin(yawT)];
        projectSquare(cBig, C, R, camPos, bcT, zT, C.TAG_LARGE, yawT);
        projectSquare(cSml, C, R, camPos, ppT, zT, C.TAG_SMALL, yawT);
        if L.seen(n) > 0
            set(cLock,'String','LOCK','Color',[0.4 1 0.5]);
        elseif strcmp(L.state{n},'FINAL')
            set(cLock,'String','COMMITTED - marker wider than frame', ...
                      'Color',[0.45 0.8 1]);
        else
            set(cLock,'String','NO LOCK','Color',[1 0.45 0.4]);
        end
        set(camTtl,'String',sprintf('onboard camera    %.2f m above deck', L.agl(n)));
        if ~okPad, set(cLock,'String','NO LOCK','Color',[1 0.45 0.4]); end

        % --- a chase view that follows the drone
        pad = 7;
        xlim(ax, c(1) + [-pad pad]);
        ylim(ax, c(2) + [-pad pad]);
        zlim(ax, [0 max(12, c(3)+3)]);

        set(ttl,'String',sprintf(['address %d   %s   |   %s    alt %.2f m    ' ...
            'above deck %.2f m    miss %.2f m    tilt %.1f deg'], ...
            addr, res.name, L.state{n}, L.z(n), L.agl(n), L.err(n), L.tilt(n)));
        drawnow limitrate
        if ~isempty(gifFile), writeGif(fig, gifFile, first); first = false; end
    end
    fprintf('   address %2d  %-8s %5.1f cm in %.1f s\n', ...
            addr, res.outcome, 100*res.error_m, res.time_s);
end
if ~isempty(gifFile), fprintf('\n   wrote %s\n', gifFile); end
end

% ------------------------------------------------------------- helpers
function ok = projectSquare(h, C, R, camPos, centre, z, side, yaw)
%PROJECTSQUARE  Put a world-frame square into the image, through the same
%   pinhole model the perception code uses: a point is taken into the body
%   frame, and the camera looks along -body z with focal length f.
d = side/2; cy = cos(yaw); sy = sin(yaw);
loc = [-d -d; d -d; d d; -d d];
loc = (([cy -sy; sy cy])*loc')';
u = zeros(1,4); v = zeros(1,4); ok = true;
for i = 1:4
    p = [centre(1)+loc(i,1); centre(2)+loc(i,2); z];
    b = R' * (p - camPos);          % world -> body
    if -b(3) <= 0.05, ok = false; break; end
    u(i) = C.IMG_W/2 + C.FOCAL * ( b(1) / -b(3));
    v(i) = C.IMG_H/2 - C.FOCAL * ( b(2) / -b(3));
end
if ok
    set(h,'XData',u,'YData',v,'Visible','on');
else
    set(h,'Visible','off');
end
end

function setSquare(h, centre, z, side, yaw)
d = side/2;
c = cos(yaw); s = sin(yaw);
pts = [-d -d; d -d; d d; -d d];
R = [c -s; s c];
pts = (R*pts')';
set(h,'XData',centre(1)+pts(:,1),'YData',centre(2)+pts(:,2), ...
      'ZData',z*ones(4,1));
end

function drawBox(ax, cx, cy, z0, w, d, h, col)
x = cx + [-w/2 w/2]; y = cy + [-d/2 d/2]; z = [z0 z0+h];
V = [x(1) y(1) z(1); x(2) y(1) z(1); x(2) y(2) z(1); x(1) y(2) z(1);
     x(1) y(1) z(2); x(2) y(1) z(2); x(2) y(2) z(2); x(1) y(2) z(2)];
F = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
patch(ax,'Vertices',V,'Faces',F,'FaceColor',col,'EdgeColor',[0.25 0.25 0.27], ...
      'FaceLighting','gouraud','AmbientStrength',0.55);
end

function yaw = padYaw(p, t)
switch p.motion
    case 'line',   yaw = p.heading;
    case 'circle', yaw = atan2(cos(p.omega*t), -sin(p.omega*t));
    otherwise,     yaw = 0;
end
end

function p = padAtTime(pad, t)
switch pad.motion
    case 'static', p = pad.home;
    case 'line'
        d = [cos(pad.heading) sin(pad.heading)];
        p = pad.home + pad.half*sin(pad.speed*t/pad.half)*d;
    case 'circle'
        p = pad.home + pad.radius*[cos(pad.omega*t)-1, sin(pad.omega*t)];
end
end

function c = kindColour(p)
switch p.kind
    case 'ground',  c = [0.95 0.60 0.20];
    case 'vehicle', c = [0.25 0.55 0.92];
    otherwise,      c = [0.38 0.72 0.38];
end
end

function v = pick(c, a, b)
if c, v = a; else, v = b; end
end

function s = onoff(tf)
if tf, s = 'on'; else, s = 'off'; end
end

function writeGif(fig, file, first)
[ind, cm] = rgb2ind(frame2im(getframe(fig)), 64);
if first
    imwrite(ind, cm, file, 'gif', 'LoopCount', Inf, 'DelayTime', 0.09);
else
    imwrite(ind, cm, file, 'gif', 'WriteMode', 'append', 'DelayTime', 0.09);
end
end

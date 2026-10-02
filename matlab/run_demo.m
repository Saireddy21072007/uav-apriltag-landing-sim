function run_demo(addresses, seed, gifFile)
%RUN_DEMO  Animate a sequence of landings and optionally write a GIF.
%
%   RUN_DEMO                      fly 6 -> 3 -> 9 (roof, vehicle, tower)
%   RUN_DEMO([4 5], 0)            fly two addresses
%   RUN_DEMO([6 3 9], 1, 'demo.gif')   ...and save an animation
%
%   The view is top-down. The shrinking circle around the drone is the
%   camera's actual footprint on the deck plane: watching it close below
%   the marker IS the decision height, which is the point in the descent
%   where vision can no longer help and the drone commits.

if nargin < 1 || isempty(addresses), addresses = [6 3 9]; end
if nargin < 2 || isempty(seed), seed = 1; end
if nargin < 3, gifFile = ''; end

C = droneconfig();
fig = figure('Color','w','Position',[80 80 620 740], ...
             'Visible', onoff(isempty(gifFile)));
ax = axes(fig); hold(ax,'on'); axis(ax,'equal');
xlim(ax,[-11 11]); ylim(ax,[-23 17]);
set(ax,'Color',[0.93 0.93 0.93],'XGrid','on','YGrid','on');
xlabel(ax,'x  (m)   across the street'); ylabel(ax,'y  (m)   along the street');

% the roadway
patch(ax,[-3 3 3 -3],[-23 -23 17 17],[0.80 0.80 0.80],'EdgeColor','none');
% the buildings, generated FROM the address book's heights
for i = 1:numel(C.PADS)
    p = C.PADS(i);
    if strcmp(p.kind,'roof')
        rectangle(ax,'Position',[p.home(1)-2.2 p.home(2)-2.2 4.4 4.4], ...
                  'FaceColor',[0.72 0.70 0.66],'EdgeColor',[0.45 0.43 0.40]);
        text(ax,p.home(1),p.home(2)-2.9,sprintf('%.1f m',p.height), ...
             'HorizontalAlignment','center','FontSize',7);
    end
end

padH = gobjects(numel(C.PADS),1);
lblH = gobjects(numel(C.PADS),1);
for i = 1:numel(C.PADS)
    p = C.PADS(i);
    padH(i) = plot(ax,p.home(1),p.home(2),'s','MarkerSize',11, ...
                   'MarkerFaceColor',kindColour(p),'MarkerEdgeColor','k');
    lblH(i) = text(ax,p.home(1)+0.7,p.home(2),sprintf('%d',p.id), ...
                   'FontWeight','bold','FontSize',9);
end
fovH   = plot(ax,NaN,NaN,'-','Color',[0.1 0.6 0.2],'LineWidth',1.2);
trailH = plot(ax,NaN,NaN,'-','Color',[0.2 0.4 0.8],'LineWidth',1.0);
droneH = plot(ax,NaN,NaN,'o','MarkerSize',9,'MarkerFaceColor','w', ...
              'MarkerEdgeColor','k','LineWidth',1.5);
ttl = title(ax,'','FontSize',10,'FontWeight','normal');

th = linspace(0,2*pi,60);
first = true;
for a = addresses
    [res, L] = simulate_landing(a, seed, struct());
    step = max(1, round(numel(L.t)/70));       % ~70 frames per address
    for n = 1:step:numel(L.t)
        for i = 1:numel(C.PADS)
            pp = padAtTime(C.PADS(i), L.t(n));
            set(padH(i),'XData',pp(1),'YData',pp(2), ...
                'MarkerFaceColor',kindColour(C.PADS(i)), ...
                'MarkerSize', 11 + 6*(C.PADS(i).id == a));
            set(lblH(i),'Position',[pp(1)+0.7 pp(2) 0]);
        end
        % the camera footprint, to scale on the deck plane
        camH = max(0, L.agl(n) - C.CAMDROP);
        r = camH * (C.IMG_H/2) / C.FOCAL;
        set(fovH,'XData',L.x(n)+r*cos(th),'YData',L.y(n)+r*sin(th), ...
            'Color', pick(L.seen(n)>0,[0.1 0.6 0.2],[0.8 0.2 0.2]));
        set(trailH,'XData',L.x(1:n),'YData',L.y(1:n));
        set(droneH,'XData',L.x(n),'YData',L.y(n));
        set(ttl,'String',sprintf(['address %d  %s   |   %s   alt %.2f m   ' ...
            'above deck %.2f m   offset %.2f m   tilt %.1f deg'], ...
            a, res.name, L.state{n}, L.z(n), L.agl(n), L.err(n), L.tilt(n)));
        drawnow limitrate
        if ~isempty(gifFile)
            writeGif(fig, gifFile, first); first = false;
        end
    end
    fprintf('   address %2d  %-8s %5.1f cm in %.1f s\n', ...
            a, res.outcome, 100*res.error_m, res.time_s);
end
if ~isempty(gifFile)
    fprintf('\n   wrote %s\n', gifFile);
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

function writeGif(fig, file, first)
frame = getframe(fig);
[ind, cm] = rgb2ind(frame2im(frame), 64);
if first
    imwrite(ind, cm, file, 'gif', 'LoopCount', Inf, 'DelayTime', 0.09);
else
    imwrite(ind, cm, file, 'gif', 'WriteMode', 'append', 'DelayTime', 0.09);
end
end

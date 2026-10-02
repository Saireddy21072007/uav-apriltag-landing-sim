classdef quatlib
%QUATLIB  Quaternion algebra for attitude, including SLERP.
%
%   q = [w x y z], unit norm, rotating BODY vectors into the WORLD frame -
%   the same convention as the Python/MuJoCo build.
%
%   The base paper works in Euler angles: it composes R = Rz(a)Ry(t)Rx(b)
%   and controls roll, pitch and yaw as three separate numbers. This
%   implementation uses unit quaternions instead, for three reasons that
%   matter here:
%
%     * No gimbal lock and no wrap. Euler angles are singular at
%       pitch = 90 deg and discontinuous at +-180 deg of yaw. A vehicle
%       chasing a pad that drives a circle crosses the yaw wrap on nearly
%       every lap, and every difference taken across that wrap is wrong by
%       360 degrees unless it is special-cased.
%     * The attitude error IS a rotation. The error between where the
%       drone points and where it should point is itself a rotation, and
%       the quaternion product gives it directly - no trigonometry and no
%       small-angle assumption.
%     * Shortest arc for free. q and -q are the same rotation, so the sign
%       of one dot product guarantees the controller always turns the
%       short way round.

methods (Static)

    function q = identity()
        q = [1 0 0 0];
    end

    function q = normalize(q)
        n = norm(q);
        if n < 1e-12, q = [1 0 0 0]; else, q = q / n; end
    end

    function r = conj(q)
        r = [q(1) -q(2) -q(3) -q(4)];
    end

    function r = mul(a, b)
        % Hamilton product: the rotation b followed by the rotation a.
        r = [a(1)*b(1) - a(2)*b(2) - a(3)*b(3) - a(4)*b(4), ...
             a(1)*b(2) + a(2)*b(1) + a(3)*b(4) - a(4)*b(3), ...
             a(1)*b(3) - a(2)*b(4) + a(3)*b(1) + a(4)*b(2), ...
             a(1)*b(4) + a(2)*b(3) - a(3)*b(2) + a(4)*b(1)];
    end

    function R = toRot(q)
        q = quatlib.normalize(q);
        w = q(1); x = q(2); y = q(3); z = q(4);
        R = [1-2*(y*y+z*z), 2*(x*y-w*z),   2*(x*z+w*y);
             2*(x*y+w*z),   1-2*(x*x+z*z), 2*(y*z-w*x);
             2*(x*z-w*y),   2*(y*z+w*x),   1-2*(x*x+y*y)];
    end

    function b3 = bodyZ(q)
        % Third column of the rotation matrix: the axis the rotors push
        % along, in world axes.
        w = q(1); x = q(2); y = q(3); z = q(4);
        b3 = [2*(x*z+w*y); 2*(y*z-w*x); 1-2*(x*x+y*y)];
    end

    function q = fromYaw(psi)
        q = [cos(psi/2) 0 0 sin(psi/2)];
    end

    function q = fromRotvec(v)
        a = norm(v);
        if a < 1e-12, q = [1 0 0 0]; return; end
        s = sin(a/2) / a;
        q = [cos(a/2), v(1)*s, v(2)*s, v(3)*s];
    end

    function v = toRotvec(q)
        % Rotation vector (axis * angle), taking the SHORT way round.
        % This is the function the attitude controller is built on: the
        % vector part of the error quaternion, sign-corrected, is
        % proportional to the axis and angle the drone must turn through,
        % and it is continuous everywhere a controller operates.
        q = quatlib.normalize(q);
        if q(1) < 0, q = -q; end        % q and -q are the same rotation
        v = q(2:4);
        s = norm(v);
        if s < 1e-9, v = 2*v; return; end
        ang = 2 * atan2(s, q(1));
        v = v * (ang / s);
    end

    function q = slerp(a, b, t)
        % Spherical linear interpolation between two orientations.
        %
        % A straight lerp fails twice: the result is not a unit quaternion,
        % so it is not a rotation until renormalised, and because q and -q
        % are the same rotation a naive lerp between two nearby
        % orientations can travel the long way round the sphere -
        % interpolating 170 deg to 190 deg through 0 instead of through
        % 180. Slerp fixes the sign first, then moves along the great
        % circle at constant angular rate.
        a = quatlib.normalize(a);
        b = quatlib.normalize(b);
        d = dot(a, b);
        if d < 0, b = -b; d = -d; end
        if d > 0.9995
            q = quatlib.normalize(a + t*(b-a));
            return
        end
        th = acos(max(-1, min(1, d)));
        q = (sin((1-t)*th)*a + sin(t*th)*b) / sin(th);
    end

    function q = fromAxes(b1, b2, b3)
        % Build a quaternion from three orthonormal body axes given as
        % columns. Shepperd's method: take the branch whose divisor is
        % largest, so it is never ill-conditioned.
        R = [b1(:) b2(:) b3(:)];
        tr = trace(R);
        if tr > 0
            s = 2*sqrt(tr + 1);
            q = [0.25*s, (R(3,2)-R(2,3))/s, (R(1,3)-R(3,1))/s, (R(2,1)-R(1,2))/s];
        elseif R(1,1) > R(2,2) && R(1,1) > R(3,3)
            s = 2*sqrt(1 + R(1,1) - R(2,2) - R(3,3));
            q = [(R(3,2)-R(2,3))/s, 0.25*s, (R(1,2)+R(2,1))/s, (R(1,3)+R(3,1))/s];
        elseif R(2,2) > R(3,3)
            s = 2*sqrt(1 + R(2,2) - R(1,1) - R(3,3));
            q = [(R(1,3)-R(3,1))/s, (R(1,2)+R(2,1))/s, 0.25*s, (R(2,3)+R(3,2))/s];
        else
            s = 2*sqrt(1 + R(3,3) - R(1,1) - R(2,2));
            q = [(R(2,1)-R(1,2))/s, (R(1,3)+R(3,1))/s, (R(2,3)+R(3,2))/s, 0.25*s];
        end
        q = quatlib.normalize(q);
    end

    function psi = yawOf(q)
        psi = 2 * atan2(q(4), q(1));
    end

end
end

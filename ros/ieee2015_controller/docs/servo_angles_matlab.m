function [A]  =  sideServoAngles(P)
%  OVERVIEW:
%  The two side - servos can move the end - effector to any point P in the 
%  vertical planar workspace defined by the current orientation of the base.
%  The servo angles A(1) and A(2) prescribe this positioning.

%  I/O:
%  P is where the center of end - effector servo - horn will be moved to. P must
%  be a 2D vector expressed in the coordinate system described next.
%  **P MUST BE IN MILLIMETERS**
%  A is an array of the two servo angles. See:
%  **SERVO ANGLES ARE IN DEGREES**

%  COORDINATE SYSTEM:
%   Origin at the servo axle (the shared axis of the two side - servos).
%   X - axis parallel to the ground, positive in the "forwards" arm direction.
%   Y - axis perpendicular to the ground and positive upwards.
%   **BOTH SERVO ANGLES ARE MEASURED FROM THE POSITIVE X - AXIS**

%  APPROACH:
%  The inverse kinematics were derived for a special point on the
%  arm we can call "the scara point." The required "scara angles" that
%  position the scara point can be determined analytically. The true servo
%  angles have a conditional relationship with these scara angles.

%  We will first define a conservative workspace that keeps the arm from
%  colliding with itself and the chassis. Singularity checks are later.
%  THESE BOUNDARIES DO NOT CURRENTLY CONSIDER THE SIZE OF THE EFFECTOR TOOL
if(P(2) > 160)
    error('Arm reaching too high -  some links will collide near the base.');
elseif(P(2) <  - 105)
    error('Arm reaching too low -  possible collision with the ground.');
elseif(P(1) < 25)
    error('Arm is going to collide with itself.');
elseif(P(2) < 0 && P(1) < 185)
    error('Arm is going to collide with the chassis.');
elseif(P(2) > 80 && P(1) < 50)
    error('Arm is going to collide with its back - carriage.');
end

%  To position the actual end - effector servohorn at P (as opposed to the
%  scara point), we create a new point (x,y) that is the location the
%  scara point would have to be in if the servohorn is to be at P.
x = P(1) - 34.504386; % mm
y = P(2) + 33.909704; % mm

%  Two fundamental link lengths (the "scara links") drive the kinematics:
L1 = 148; % mm
L2 = 160; % mm

%  Analytical expression for the cosine of the second (outer) scara angle:
c2 = ((L1^2 + L2^2) - (x^2 + y^2))/(2 * L1 * L2);
%  The sine of this angle (s2) can also be found by considering that
%  s2^2 + c2^2 = 1. This will be used to simplify the next calculation.
s2 = sqrt(1 - c2^2);
%  Analytical expression for the cosine of the first (inner) scara angle:
c1 = (L1 * x - L2 * (x * c2 + y * s2))/(L1^2 + ((L2 * s2)^2 + (L2 * c2)^2) - 2 * L1 * L2 * c2);

%  Obtain servo angles from scara angles:
A(1) = acosd(c1);
A(2) = A(1) + acosd(c2);
%  The sign of the first servo angle depends on a lot of things. To play it
%  safe, just see if the solution works (to within a micron) and try the
%  other sign if it doesn't.
if (abs(x - (L1 * cosd(A(1)) - L2 * cosd(A(2)))) > 10^ - 6 || abs(y - (L1 * sind(A(1)) - L2 * sind(A(2))) > 10^ - 6))
    A(1) =  - A(1);
    A(2) = A(1) + acosd(c2);
end

%  Verify that any real solutions worked, else the point is unattainable.
if (imag(A(1))~ = 0 || imag(A(2))~ = 0)
    error('Position unattainable!');
end

%  A singularity occurs wherever the servo arms become colinear. Even being
%  near one is dangerous because the structure will become unstable and can
%  flip into a new configuration only fixable manually.
if (abs(A(2) - A(1)) < 15 || abs(A(2) - (A(1) + 180)) < 15)
    error('Approaching a singularity!');
end

%  See Jason for complete derivation of all this. It isn't very difficult.
end
function [q1,q2,q3] = InverseKinematics(x,y,z)
%Define the parameters
d1 = 75.51; % mm
a2 = 67.87; %mm
a3 = 122.11; %mm
r = sqrt(x^2+y^2);
s = z-d1;
%% Calculate q1
q1 = atan2(y,x);
q1 = q1*(180/pi);
%% Calculate q3
q31 = ((z-d1)^2+x^2+y^2-a2^2-a3^2);
q32 = 2*a2*a3;
q3 = -acos(q31/q32); % for elbow up position, minus sign is given
q3 = q3*(180/pi);
%% Calculate q2
q21 = 2*a2*sqrt(s^2+r^2);
q22 = a2^2-a3^2+r^2+s^2;
phi1 = acos(q22/q21);
phi2 = atan2(s,r);
q2 = phi2 + phi1;
q2 = q2*(180/pi);
end







%%
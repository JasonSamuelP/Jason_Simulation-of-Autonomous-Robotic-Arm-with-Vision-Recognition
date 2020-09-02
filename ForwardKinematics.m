% Function to calculate the Transformation matrices
% Calculation is based on degree  cosd --> Cosine in degree
function T__EndEffector = ForwardKinematics(q01, q02, q03)

    syms th d a alph % Define the variables for tetha link offset(d), link length(a), and link twist(alph)

    % Formula for Link Coordinate transformation matrices (i T i-1)
    T = [ cosd(th)      -cosd(alph)*sind(th)     sind(alph)*sind(th)      a*cosd(th)    
          sind(th)      cosd(alph)*cosd(th)      -sind(alph)*cosd(th)     a*sind(th)
          0             sind(alph)              cosd(alph)              d
          0             0                       0                       1];
        %syms q1 % Define the variable for joint angle 1(th)
   T10 = subs(T,{th,d,a,alph},{q01,75.51,0,90});              % d = 75.51 mm measured from the support origin to forearm origin
        %syms q2 % Define the variable for joint angle 2(th)
   T21 = subs(T,{th,d,a,alph},{q02,0,67.87,0});               % a = 67.87 mm
        %syms q3 % Define the variable for joint angle 3(th)
   T32 = subs(T,{th,d,a,alph},{q03,0,122.11,0});              % a = 122.11 mm         
  % Transformation matrices for end effector frame referring from base frame 
   T_EndEffector = T10*T21*T32;
   T__EndEffector = double(T_EndEffector);   
end

% Include the Gripper Offset

%  %syms q1 % Define the variable for joint angle 1(th)
%    T10 = subs(T,{th,d,a,alph},{q1,75.51,0,90})              % d = 75.51 mm measured from the support origin to forearm origin
%         %syms q2 % Define the variable for joint angle 2(th)
%    T21 = subs(T,{th,d,a,alph},{q2,0,67.87,0})               % a = 67.87 mm
%         %syms q3 % Define the variable for joint angle 3(th)
%    T32 = subs(T,{th,d,a,alph},{q3-90,0,13.02,-90})              % a = 122.11 mm
%    
%    T43 = subs(T,{th,d,a,alph},{0,122.11,0,0})              
% 
%   % Transformation matrices for end effector frame referring from base frame 
%    T_EndEffector = T10*T21*T32*T43;
%    T__EndEffector = double(T_EndEffector) 


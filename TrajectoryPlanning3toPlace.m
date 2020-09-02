function [qd3toPlace,vd3toPlace,ad3toPlace,VariableA3toPlace,steptime3toplace,qd3_initialplace,qd3_finalplace] = TrajectoryPlanning3toPlace(q3p, v0, q03, vf, t0, tf)
%% Trajectory Planning Matlab (Joint rotation from one position to the desired position) --> NOT THE ENDEFFECTOR


t = linspace(t0,tf,100*(tf-t0));
% linspace --> from t0 value to tf with these amounts of steps (100*(tf-t0))
c = ones(size(t));
% Change the value of every step became 1
steptime3toplace = 100*(tf-t0);

% define the Coefficient Matrices
M = [1      t0      t0^2        t0^3;
     0      1       2*t0        3*t0^2;
     1      tf      tf^2        tf^3;
     0      1       2*tf        3*tf^2 ];
 % Initialize the Vector of initial data
 b = [  q03;
        v0;
        q3p;
        vf];
    
 % Calculate the Vector Coefficient of the Cubic Polynomial
a = M\b;
 
 VariableA3toPlace = a;
 
 
 %% Display the equations q, v, a equations
 
 % Since t and c are matrices
 % .* --> Multiply per element,  both matrices sizes must be same or
 % compatible
 % .^ --> Raises each element of A to the corresponding powers in B, in
 % this case each of the elements in t is raised to 3
 qd3toPlace = a(1).*c + a(2).*t + a(3).*t.^2 + a(4).*t.^3;
 vd3toPlace =  a(2).*c + 2*a(3).*t + 3*a(4).*t.^2;
 ad3toPlace = 2*a(3).*c + 6*a(4).*t; 
 
 qd3_initialplace = qd3toPlace(1);
 qd3_finalplace = qd3toPlace(1000);
 

 
%% Plot the graph for q, v, and a
% tiledlayout(3,1) % Create data and 2-by-1 tiled chart layout
% % Position
% ax1 = nexttile;
% plot(ax1,t,qd)
% title(ax1,'Joint Displacement')
% ylabel(ax1,'qd')
% % Velocity
% ax2 = nexttile;
% plot(ax2,t,vd)
% title(ax2,'Joint Velocity')
% ylabel(ax2,'vd')
% % Acceleration
% ax3 = nexttile;
% plot(ax3,t,ad)
% title(ax3,'Joint Acceleration')
% ylabel(ax3,'ad')
end
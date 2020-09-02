function [qd2home,vd2home,ad2home,VariableA2home,steptime2home,qd2_initialhome,qd2_finalhome] = TrajectoryPlanning2tohome(q02, v0, q2, vf, t0, tf)
%% Trajectory Planning Matlab (Joint rotation from one position to the desired position) --> NOT THE ENDEFFECTOR
% q0 = Initial Position
% qf = Final Position
% v0 = Initial Velocity
% vf = Final Velocity
% t0 = Initial time
% tf = final time

%d = input('Input the parameters = [q0,v0,qf,vf,t0,tf] = ') % ask the user to input the parameters
% Need to answer like this; [value value value value value value]
% q0 = d(1);
% v0 = d(2);
% qf = d(3);
% vf = d(4);
% t0 = d(5);
% tf = d(6);

t = linspace(t0,tf,100*(tf-t0));
% linspace --> from t0 value to tf with these amounts of steps (100*(tf-t0))
c = ones(size(t));
% Change the value of every step became 1
steptime2home = 100*(tf-t0);

% define the Coefficient Matrices
M = [1      t0      t0^2        t0^3;
     0      1       2*t0        3*t0^2;
     1      tf      tf^2        tf^3;
     0      1       2*tf        3*tf^2 ];
 % Initialize the Vector of initial data
 b = [  q2;
        v0;
        q02;
        vf];
    
 % Calculate the Vector Coefficient of the Cubic Polynomial
a = M\b;
 VariableA2home = a;
 %% Display the equations q, v, a equations
 
 % Since t and c are matrices
 % .* --> Multiply per element,  both matrices sizes must be same or
 % compatible
 % .^ --> Raises each element of A to the corresponding powers in B, in
 % this case each of the elements in t is raised to 3
 qd2home = a(1).*c + a(2).*t + a(3).*t.^2 + a(4).*t.^3;
 vd2home =  a(2).*c + 2*a(3).*t + 3*a(4).*t.^2;
 ad2home = 2*a(3).*c + 6*a(4).*t; 
 
 qd2_initialhome = qd2home(1);
 qd2_finalhome = qd2home(1000);
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
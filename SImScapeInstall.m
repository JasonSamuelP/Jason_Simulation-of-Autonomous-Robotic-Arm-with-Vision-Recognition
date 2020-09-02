%% Install Simscape and IMPORT model from solidworks to SIMSCAPE
% 1. Download the matlab file and zip file from the link below
% https://www.mathworks.com/help/physmod/smlink/ug/installing-and-linking-simmechanics-link-software.html
% 2. Right click on matlab and set as "Run as administrator"
% 3. type this on command window : install_addon('smlink.r2019a.win64.zip')
% 4. type : regmatlabserver --> Register matlab as an automation server (Each time you export a CAD assembly model, the Simscape Multibody Link plug-in attempts to connect to MATLAB. For the connection to occur, you must register MATLAB as an automation server)
% 5. Enable the Simscape Multibody Link Plug-In, type: smlink_linksw 

% Open solidworks, check tools, and add-in --> You should see "Sim Mechanics Add in"
% In solidworks, select the model , click tool, and export to XML file
% type this on command window: smimport('Robotic_Arm_For_SIMScape.xml')

% Type : sm_import_robot_stepfiles , for tutorial of Robotic arm in CAD file in simscape

%% Rigid Body From simscape
%   open_system('Robotic_Arm_For_SIMScape_2.slx')
%    [robot,importInfo] = importrobot(gcs)
%    showdetails(importInfo)
%    show(robot) % From solidworks and simscape
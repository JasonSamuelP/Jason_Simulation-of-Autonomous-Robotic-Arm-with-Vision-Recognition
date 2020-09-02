clc; 
clear;
close all;
%% Set up webcam 
objects = imaqfind;                             % Find video input objects in memory
delete(objects)                                 % Delete a video input object from memory
vid=videoinput('winvideo',1,'MJPG_1280x720');   % Create Video Object
preview(vid)                                    % Preview Video
%% Set up Arduino in Matlab + servo + ultrasonic Sensor
% Create an Arduino Object in Matlab
ArduinoObject = arduino('COM3','Uno','Libraries',{'Servo','Ultrasonic'});
% Create Servo Object
BaseServo = servo(ArduinoObject,'D9');      % Connected to Digital Port number 9
CurrentPosition = readPosition(BaseServo);  % Read the current position of base servo
writePosition(BaseServo,0)                  % Set servo position to 0 degree
double pos;                                     
% Create Ultrasonic Object
ultrasonicObject = ultrasonic(ArduinoObject,'D12','D13','OutputFormat','double');
% D12 is trigger pin and D13 is echo pin, echo time value is double

key = 0;
%% For blue Object
    for pos = 0.15:0.15:0.9
    %% Initialize the Servo Setting   
    writePosition(BaseServo,pos);               % Set value for servo position
    CurrentPosition = readPosition(BaseServo);   % Read the current servo angle
    indegree = (CurrentPosition/1)*180;         % to degree conversion
    disp(indegree);                             % Read and display the current angle in degree
    pause(1);                                   % Pause for 1 second
    %% Blue Object Detection
    % Get snapshot
    b = getsnapshot(vid);
    b = b(1:end,110:1210,:);                    % Crop the video width (220 to 1100) --> total width is 880 pixels
    %% Call the Area Calculation Function for each RGB Color Plane
    [BlueArea1,RedArea1,GreenArea1] = MyFunction(b); 
    %% Will determine what is the object color
     if BlueArea1 > RedArea1 && BlueArea1 > GreenArea1
         targetcolor = 'Blue1';
     elseif RedArea1 > BlueArea1 && RedArea1 > GreenArea1 
         targetcolor = 'Red12';
     else
         targetcolor = 'Green';
     end
     
% For Blue object detection for bounding box and centroid
 % start processing of blue object 
  if targetcolor == 'Blue1'
    fprintf('Target Color is Blue\n\n')
    %% Preview of stream image frames
    set(vid,'FramesPerTrigger',inf);            % It is set to infinity, object will keep acquiring frame until it is stopped   
    set(vid,'ReturnedColorspace','rgb')         % Colorspace use is RGB        
    vid.FrameGrabInterval=5;                    % Extract screenshot/frame from video every 5 millisecond
    start(vid);
    while(vid.FramesAcquired<=500)              % The Streaming will end after 500 frames/screenshots
    b = getsnapshot(vid);                       % Declare variable data to store the extracted frame
    pause(0.05);
    diff_im = imsubtract(b(:,:,3),rgb2gray(b)); % Subtract the blue component from the grayscale image
    diff_im = medfilt2(diff_im,[3,3]);          % 2D Median filtering
    diff_im = im2bw(diff_im,0.18);              % Grayscale to binary image
    BW2 = bwareafilt(diff_im,1);                % To extract objects from binary image by size (target image, keep n largest objects in the image
    stats = regionprops(BW2,'BoundingBox','Centroid'); 
    imshow(b);                                  
    hold on
    % Loop for rectangular bounding boxes
    for object=1:length(stats)                  
        bb = stats(object).BoundingBox;
        bc = stats(object).Centroid;
        % Rectangle function is used to draw rectangle with data from variables bb and bc
        rectangle('Position',bb,'EdgeColor','b','Linewidth',2.5)
        plot(bc(1),bc(2),'-m+')
        % Write the coordinate of the centroid on the screen
        c=text(bc(1)+15,bc(2), strcat(' X: ', num2str(round(bc(1))), ' Y: ', num2str(round(bc(2)))));
        set(c, 'FontName', 'Calibri', 'FontWeight', 'bold', 'FontSize', 10, 'Color', 'black') 
    end
    
    %% Align the Object Centroid to Camera Centroid (Blue Object)
    centerx = regionprops(BW2,'Centroid');
    centerx = (centerx.Centroid(1,1));                      % obtain the centroid of x value
    center2x = (1280 - centerx);                            % 1280 pixels is the Camera width (X direction)  
    center3x = (640 - center2x);                            % 640 is the center point of the camera width (X direction)
    if (center3x > 0)   % When object centroid is at the right side of the mid camera frame
        Rcentx = round(centerx);
        if Rcentx >= 639 && Rcentx <= 641
            fprintf('At the center already');
            disp(Rcentx)
            disp(posz)
            break
        end
        posz = readPosition(BaseServo);
        posz = posz - 0.01 - (abs(rand()))/50; % random number is introduced for decrement or increment servo position ... 
        writePosition(BaseServo,posz);        % since the servo is limited to min increment/decrement of 1.8 degree
        disp(Rcentx)
        disp(posz)
        pause(0.4); 
    elseif (center3x < 0) % When object centroid is at the left side of the mid camera frame
        Rcentx = round(centerx); 
        if Rcentx >= 639 && Rcentx <= 641
            fprintf('At the center already');
            disp(Rcentx)
            disp(posz)
            break
        end
        posz = readPosition(BaseServo);
        posz = posz + 0.01 + (abs(rand()))/50;
        writePosition(BaseServo,posz); 
        pause(0.4);
    end     
    %%
    hold off 
    end 
    %% Measure the distance using ultrasonic Sensor (Blue Object)
    pause(2);
    Total_distance_cm = 0;
    for time = 1:1:25
        pause(0.25);
        distance = readDistance(ultrasonicObject);
        distance_cm = distance*100;
        fprintf('Current distance is %.4f cm\n',distance_cm)
        Total_distance_cm = distance_cm + Total_distance_cm;
    end
    AverageDistance_cm = Total_distance_cm/25;
    disp(AverageDistance_cm)
    pause(0.5);
    
    stop(vid),
    flushdata(vid);
    % clear all;
    %%  Finding x,y,z Coordinates using basic math and sine rule
    
    % First, Find the x and y coordinates, for z is already fixed (27.63mm) 
    OrgToUSsensor = 7.5; % In cm, Origin to US Sensor Distance
    OrgToObj =  (AverageDistance_cm + OrgToUSsensor + 0.9)*10; 
    % In mm, origin to target object distance 
    % (0.9 is the target object width(1.8) divided by 2)
    Angle_x_y = pi/2; % Angle between x and y coordinates
    Angle_x_OrgToObj = (pi*pos)-(pi/2); % But it can be minus, % Angle between x and OrgToObj (Servo Motor Angle)
    Angle_OrgToObj_y = pi - Angle_x_y - abs(Angle_x_OrgToObj); 
    
    % Since we know the hypotenuse (OrgToObj) and all the 3 angles , 
    % Sine Rule is used to calculate the X and Y Coordinates
    if Angle_x_OrgToObj == 0
        x = OrgToObj;
        y = 0;
        z = 27.63; % Z is constant, since it is the height of the target object to the ground
    else
        x =  (OrgToObj*sin(Angle_OrgToObj_y))/sin(Angle_x_y); % In mm
        y =  (OrgToObj*sin(Angle_x_OrgToObj))/sin(Angle_x_y); % In mm
        z = 27.63; % Z is constant, since it is the height of the target object to the ground
    end
    %% Perform the Inverse kinematic and Pick and Place function
 % Home Configuration --> Configuration for rest position  
 % d1 = 75.51mm , a2 = 67.87 mm , a3 = 122.11 mm
 if key == 0
        q01 = -91.8; % adjust according to the simulation dimension (org = -90degree)
        q02 = 60;
        q03 = -50;
 elseif key == 1
        q01 = q1; % q1 from previous configuration became the home configuration for this state
        q02 = 60;
        q03 = -50;
 end

   q04 = 140;  % 173 = right gripper (q4), 161 = Left Gripper(q5) For closed condition            
   q05 = 194;  % 140 = right gripper (q4), 194 = Left Gripper(q5) For max open condition
 % Calling forward kinematics function
 T__EndEffector = ForwardKinematics(q01, q02, q03); % End effector transformation was obtained for home configurations
 
 % Call the inverse kinematic functions
 [q1,q2,q3] = InverseKinematics(x,y,z);
 q1 = q1 - 5; % adjust according to the simulation dimension 
 % 4.8
 
 % Control the gripper
 % 173 = right gripper (q4), 161 = Left Gripper(q5) For closed condition
 % 140 = right gripper (q4), 194 = Left Gripper(q5) For max open condition
 q4 = 173;            
 q5 = 161;
 
% Call the trajectory Planning
t0 = 0;
tf = 10;
v0 = 0;
vf = 0;
[qd1,vd1,ad1,VariableA1,steptime1,qd1_initial,qd1_final] = TrajectoryPlanning(q01, v0, q1, vf, t0, tf); % for Joint 1
% Cubic Polynomials Coefficients for TrajectoryPlanning q01 to q1
 fcn11 = VariableA1(1);
 fcn12 = VariableA1(2);
 fcn13 = VariableA1(3);
 fcn14 = VariableA1(4);

[qd2,vd2,ad2,VariableA2,steptime2,qd2_initial,qd2_final] = TrajectoryPlanning2(q02, v0, q2, vf, t0, tf); % For Joint 2
 %Cubic Polynomials Coefficients for TrajectoryPlanning q02 to q2
 fcn21 = VariableA2(1);
 fcn22 = VariableA2(2);
 fcn23 = VariableA2(3);
 fcn24 = VariableA2(4);
 
[qd3,vd3,ad3,VariableA3,steptime3,qd3_initial,qd3_final] = TrajectoryPlanning3(q03, v0, q3, vf, t0, tf); % for Joint 3
 %Cubic Polynomials Coefficients for TrajectoryPlanning q03 to q3
 fcn31 = VariableA3(1);
 fcn32 = VariableA3(2);
 fcn33 = VariableA3(3);
 fcn34 = VariableA3(4);

[qd2home,vd2home,ad2home,VariableA2home,steptime2tohome,qd2_initialhome,qd2_finalhome] = TrajectoryPlanning2tohome(q02, v0, q2, vf, t0, tf);
 %Cubic Polynomials Coefficients for TrajectoryPlanning q2 to q02
 fcn21home = VariableA2home(1);
 fcn22home = VariableA2home(2);
 fcn23home = VariableA2home(3);
 fcn24home = VariableA2home(4);

 [qd3home,vd3home,ad3home,VariableA3home,steptime3tohome,qd3_initialhome,qd3_finalhome] = TrajectoryPlanning3tohome(q03, v0, q3, vf, t0, tf);
%Cubic Polynomials Coefficients for TrajectoryPlanning q3 to q03
 fcn31home = VariableA3home(1);
 fcn32home = VariableA3home(2);
 fcn33home = VariableA3home(3);
 fcn34home = VariableA3home(4);
% Place Configuration
 q1p = 88.2;
 q2p = 35;
 q3p = -75;
 %q04 = 140;  % 173 = right gripper (q4), 161 = Left Gripper(q5) For closed condition            
 %q05 = 194;  % 140 = right gripper (q4), 194 = Left Gripper(q5) For max open condition 
 
[qd1toPlace,vd1toPlace,ad1toPlace,VariableA1toPlace,steptime1toplace,qd1_initialplace,qd1_finalplace] = TrajectoryPlanning1toPlace(q1p, v0, q1, vf, t0, tf);
%Cubic Polynomials Coefficients for TrajectoryPlanning q1 to q1p 
 fcn1_1place= VariableA1toPlace(1);
 fcn1_2place= VariableA1toPlace(2);
 fcn1_3place= VariableA1toPlace(3);
 fcn1_4place = VariableA1toPlace(4);

[qd2toPlace,vd2toPlace,ad2toPlace,VariableA2toPlace,steptime2toplace,qd2_initialplace,qd2_finalplace] = TrajectoryPlanning2toPlace(q2p, v0, q02, vf, t0, tf);
%Cubic Polynomials Coefficients for TrajectoryPlanning q02 to q2p
fcn2_1place= VariableA2toPlace(1);
 fcn2_2place= VariableA2toPlace(2);
 fcn2_3place= VariableA2toPlace(3);
 fcn2_4place = VariableA2toPlace(4);

[qd3toPlace,vd3toPlace,ad3toPlace,VariableA3toPlace,steptime3toplace,qd3_initialplace,qd3_finalplace] = TrajectoryPlanning3toPlace(q3p, v0, q03, vf, t0, tf);
%Cubic Polynomials Coefficients for TrajectoryPlanning q03 to q3p
fcn3_1place= VariableA3toPlace(1);
fcn3_2place= VariableA3toPlace(2);
fcn3_3place= VariableA3toPlace(3);
fcn3_4place = VariableA3toPlace(4);

[qd2toPlacetohome,vd2toPlacetohome,ad2toPlacetohome,VariableA2toPlacetohome,steptime2toplacetohome,qd2_initialplacetohome,qd2_finalplacetohome] = TrajectoryPlanning2Placetohome(q2p, v0, q02, vf, t0, tf);
%Cubic Polynomials Coefficients for TrajectoryPlanning q2p to q02
fcn2_1placetohome= VariableA2toPlacetohome(1);
fcn2_2placetohome= VariableA2toPlacetohome(2);
fcn2_3placetohome= VariableA2toPlacetohome(3);
fcn2_4placetohome = VariableA2toPlacetohome(4);

[qd3toPlacetohome,vd3toPlacetohome,ad3toPlacetohome,VariableA3toPlacetohome,steptime3toplacetohome,qd3_initialplacetohome,qd3_finalplacetohome] = TrajectoryPlanning3Placetohome(q3p, v0, q03, vf, t0, tf);
%Cubic Polynomials Coefficients for TrajectoryPlanning q3p to q03
fcn3_1placetohome= VariableA3toPlacetohome(1);
fcn3_2placetohome= VariableA3toPlacetohome(2);
fcn3_3placetohome= VariableA3toPlacetohome(3);
fcn3_4placetohome= VariableA3toPlacetohome(4);

% velocity for place to current target
v0 = 0;
vf = 0;
[qd1toPlacetoPrevious,vd1toPlacetoPrevious,ad1toPlacetoPrevious,VariableA1toPlacetoPrevious,steptime1toplacetoPrevious,qd1_initialplacetoPrevious,qd1_finalplacetoPrevious] = TrajectoryPlanning1PlacetoPrevious(q1p, v0, q1, vf, t0, tf);
%Cubic Polynomials Coefficients for TrajectoryPlanning q1p to q1
fcn1_1placetoPrevious= VariableA1toPlacetoPrevious(1);
fcn1_2placetoPrevious= VariableA1toPlacetoPrevious(2);
fcn1_3placetoPrevious= VariableA1toPlacetoPrevious(3);
fcn1_4placetoPrevious= VariableA1toPlacetoPrevious(4);
% Open Simulink
syst = 'Robotic_Arm_For_SIMScape_2'; % Define syst as the simulink file
open(syst)
sim(syst)
pause(40);
%% Updated Home Configuration
q01 = q1;
q02 = 60;
q03 = -50;
key = 1;   
    fprintf('Process Finished');
else
      fprintf('Object is not blue');
      pause(0.5);     
end
    end
%% For Red Object
    for pos = 0.9:-0.15:0.15   
        %Scan for Red
    writePosition(BaseServo,pos);               % Set value to servo
    CurrentPosition = readPosition(BaseServo);   % Read the current servo angle
    indegree = (CurrentPosition/1)*180;         
    disp(indegree);                             % Read and display the current angle in degree
    pause(1); 
    % get snapshot
    b = getsnapshot(vid);
    b = b(1:end,110:1210,:);                    % Crop the video width (220 to 1100) --> total width is 880 pixels 
    % Call a function to calculate the area for each color plane
    [BlueArea1,RedArea1,GreenArea1] = MyFunction(b); 
    % Will determine what is the object color
     if BlueArea1 > RedArea1 && BlueArea1 > GreenArea1
         targetcolor = 'Blue1';
     elseif RedArea1 > BlueArea1 && RedArea1 > GreenArea1 
         targetcolor = 'Red12';
     else
         targetcolor = 'Green';
     end
    
if targetcolor == 'Red12'
      fprintf('Target Color is Red\n\n')
      % Set the frame
    set(vid,'FramesPerTrigger',inf);        % 
    set(vid,'ReturnedColorspace','rgb')
    vid.FrameGrabInterval=5;                % Extract screenshot from video every 5 millisecond
    start(vid);
    while(vid.FramesAcquired<=500)
    % Preview of stream image frames
    
    b = getsnapshot(vid);                         % Declare variable data to store the extracted frame
    
    diff_im = imsubtract(b(:,:,1),rgb2gray(b)); % Subtract the red component from the grayscale image
    diff_im = medfilt2(diff_im,[3,3]);          % 2D Median filtering
    diff_im = im2bw(diff_im,0.18);              % Grayscale to binary image
    BW2 = bwareafilt(diff_im,1);                % to extract objects from binary image by size (target image, keep n largest objects in the image
    stats = regionprops(BW2,'BoundingBox','Centroid');  % Image blob analysis, we get set properties for each labelled region
    imshow(b);                                  % Image is shown
    hold on
    
    for object=1:length(stats)                  % Loop for rectangular bounding boxes
        bb = stats(object).BoundingBox;
        bc = stats(object).Centroid;
     % Rectangle function is used to draw rectangle with data from variables bb and bc
        rectangle('Position',bb,'EdgeColor','r','Linewidth',2.5)
        plot(bc(1),bc(2),'-m+')
        % Write the coordinate of the centroid on the screen
        c=text(bc(1)+15,bc(2), strcat(' X: ', num2str(round(bc(1))), ' Y: ', num2str(round(bc(2)))));
        set(c, 'FontName', 'Calibri', 'FontWeight', 'bold', 'FontSize', 10, 'Color', 'black');
        
    end
    %% Align the object centroid to camera centroid (Red Object)
    centerx = regionprops(BW2,'Centroid');
    centerx = (centerx.Centroid(1,1));                      % obtain the centroid of x value
    center2x = (1280 - centerx);                            % 1280 pixels is the Camera width (X direction)  
    center3x = (640 - center2x);                            % 640 is the center point of the camera width (X direction)
    if (center3x > 0)   % When object centroid is at the right side of the mid camera frame
        Rcentx = round(centerx);
        if Rcentx >= 639 && Rcentx <= 641
            fprintf('At the center already');
            disp(Rcentx)
            disp(posz)
            break
        end
        posz = readPosition(BaseServo);
        posz = posz - 0.01 - (abs(rand()))/50; % random number is introduced for decrement or increment servo position ... 
        writePosition(BaseServo,posz);        % since the servo is limited to min increment/decrement of 1.8 degree
        disp(Rcentx)
        disp(posz)
        pause(0.4); 
    elseif (center3x < 0) % When object centroid is at the left side of the mid camera frame
        Rcentx = round(centerx); 
        if Rcentx >= 639 && Rcentx <= 641
            fprintf('At the center already');
            disp(Rcentx)
            disp(posz)
            break
        end
        posz = readPosition(BaseServo);
        posz = posz + 0.01 + (abs(rand()))/50;
        writePosition(BaseServo,posz); 
        pause(0.4);
    end   
    hold off
    end
    %% Measure the distance using ultrasonic Sensor (Red Object)
    pause(2);
    Total_distance_cm = 0;
    for time = 1:1:25
        pause(0.25);
        distance = readDistance(ultrasonicObject);
        distance_cm = distance*100;
        fprintf('Current distance is %.4f cm\n',distance_cm)
        Total_distance_cm = distance_cm + Total_distance_cm;
        
    end
    AverageDistance_cm = Total_distance_cm/25;
    pause(0.5);
 
    stop(vid),
    flushdata(vid);
    % clear all;
    
    %%  Finding x,y,z Coordinates using basic math and sine rule
    
    % First, Find the x and y coordinates, for z is already fixed (27.63mm) 
    OrgToUSsensor = 7.5; % In cm, Origin to US Sensor Distance
    OrgToObj =  (AverageDistance_cm + OrgToUSsensor + 0.9)*10; % In mm, origin to target object distance (0.9 is the target object width(1.8) divided by 2)
    Angle_x_y = pi/2; % Angle between x and y coordinates
    Angle_x_OrgToObj = (pi*pos)-(pi/2); % But it can be minus, % Angle between x and OrgToObj (Servo Motor Angle)
    Angle_OrgToObj_y = pi - Angle_x_y - abs(Angle_x_OrgToObj); 
    
    % Since we know the hypotenuse (OrgToObj) and all the 3 angles , 
    % Sine Rule is used to calculate the X and Y Coordinates
   if Angle_x_OrgToObj == 0
        x = OrgToObj;
        y = 0;
        z = 27.63; % Z is constant, since it is the height of the target object to the ground
    else
        x =  (OrgToObj*sin(Angle_OrgToObj_y))/sin(Angle_x_y); % In mm
        y =  (OrgToObj*sin(Angle_x_OrgToObj))/sin(Angle_x_y); % In mm
        z = 27.63; % Z is constant, since it is the height of the target object to the ground
    end
    
    %% Perform the Inverse kinematic and Pick and Place function
 % Home Configuration --> Configuration for rest position  
 % d1 = 75.51mm , a2 = 67.87 mm , a3 = 122.11 mm
 
 if key == 0
        q01 = q1; % q1 from previous configuration became the home configuration for this state
        q02 = 60;
        q03 = -50;
 elseif key == 1
        q01 = q1; % q1 from previous configuration became the home configuration for this state
        q02 = 60;
        q03 = -50;
 end

   q04 = 140;  % 173 = right gripper (q4), 161 = Left Gripper(q5) For closed condition            
   q05 = 194;  % 140 = right gripper (q4), 194 = Left Gripper(q5) For max open condition
 % Calling forward kinematics function
 T__EndEffector = ForwardKinematics(q01, q02, q03); % End effector transformation was obtained for home configurations
 
 % Call the inverse kinematic functions
 [q1,q2,q3] = InverseKinematics(x,y,z);
 q1 = q1 - 5; % adjust according to the simulation dimension 
 
 % Control the gripper
 % 173 = right gripper (q4), 161 = Left Gripper(q5) For closed condition
 % 140 = right gripper (q4), 194 = Left Gripper(q5) For max open condition
 q4 = 173;            
 q5 = 161;
 
% Call the trajectory Planning
t0 = 0;
tf = 10;
v0 = 0;
vf = 0;
[qd1,vd1,ad1,VariableA1,steptime1,qd1_initial,qd1_final] = TrajectoryPlanning(q01, v0, q1, vf, t0, tf); % for Joint 1
% Cubic Polynomials Coefficients for TrajectoryPlanning q01 to q1
 fcn11 = VariableA1(1);
 fcn12 = VariableA1(2);
 fcn13 = VariableA1(3);
 fcn14 = VariableA1(4);

[qd2,vd2,ad2,VariableA2,steptime2,qd2_initial,qd2_final] = TrajectoryPlanning2(q02, v0, q2, vf, t0, tf); % For Joint 2
 %Cubic Polynomials Coefficients for TrajectoryPlanning q02 to q2
 fcn21 = VariableA2(1);
 fcn22 = VariableA2(2);
 fcn23 = VariableA2(3);
 fcn24 = VariableA2(4);

[qd3,vd3,ad3,VariableA3,steptime3,qd3_initial,qd3_final] = TrajectoryPlanning3(q03, v0, q3, vf, t0, tf); % for Joint 3
 %Cubic Polynomials Coefficients for TrajectoryPlanning q03 to q3
 fcn31 = VariableA3(1);
 fcn32 = VariableA3(2);
 fcn33 = VariableA3(3);
 fcn34 = VariableA3(4);

[qd2home,vd2home,ad2home,VariableA2home,steptime2tohome,qd2_initialhome,qd2_finalhome] = TrajectoryPlanning2tohome(q02, v0, q2, vf, t0, tf);
 %Cubic Polynomials Coefficients for TrajectoryPlanning q2 to q02
 fcn21home = VariableA2home(1);
 fcn22home = VariableA2home(2);
 fcn23home = VariableA2home(3);
 fcn24home = VariableA2home(4);

 [qd3home,vd3home,ad3home,VariableA3home,steptime3tohome,qd3_initialhome,qd3_finalhome] = TrajectoryPlanning3tohome(q03, v0, q3, vf, t0, tf);
%Cubic Polynomials Coefficients for TrajectoryPlanning q3 to q03
 fcn31home = VariableA3home(1);
 fcn32home = VariableA3home(2);
 fcn33home = VariableA3home(3);
 fcn34home = VariableA3home(4);
% Place Configuration
 q1p = 88.2;
 q2p = 35;
 q3p = -75;
 %q04 = 140;  % 173 = right gripper (q4), 161 = Left Gripper(q5) For closed condition            
 %q05 = 194;  % 140 = right gripper (q4), 194 = Left Gripper(q5) For max open condition 

[qd1toPlace,vd1toPlace,ad1toPlace,VariableA1toPlace,steptime1toplace,qd1_initialplace,qd1_finalplace] = TrajectoryPlanning1toPlace(q1p, v0, q1, vf, t0, tf);
%Cubic Polynomials Coefficients for TrajectoryPlanning q1 to q1p 
 fcn1_1place= VariableA1toPlace(1);
 fcn1_2place= VariableA1toPlace(2);
 fcn1_3place= VariableA1toPlace(3);
 fcn1_4place = VariableA1toPlace(4);

[qd2toPlace,vd2toPlace,ad2toPlace,VariableA2toPlace,steptime2toplace,qd2_initialplace,qd2_finalplace] = TrajectoryPlanning2toPlace(q2p, v0, q02, vf, t0, tf);
%Cubic Polynomials Coefficients for TrajectoryPlanning q02 to q2p
fcn2_1place= VariableA2toPlace(1);
 fcn2_2place= VariableA2toPlace(2);
 fcn2_3place= VariableA2toPlace(3);
 fcn2_4place = VariableA2toPlace(4);

[qd3toPlace,vd3toPlace,ad3toPlace,VariableA3toPlace,steptime3toplace,qd3_initialplace,qd3_finalplace] = TrajectoryPlanning3toPlace(q3p, v0, q03, vf, t0, tf);
%Cubic Polynomials Coefficients for TrajectoryPlanning q03 to q3p
fcn3_1place= VariableA3toPlace(1);
fcn3_2place= VariableA3toPlace(2);
fcn3_3place= VariableA3toPlace(3);
fcn3_4place = VariableA3toPlace(4);

[qd2toPlacetohome,vd2toPlacetohome,ad2toPlacetohome,VariableA2toPlacetohome,steptime2toplacetohome,qd2_initialplacetohome,qd2_finalplacetohome] = TrajectoryPlanning2Placetohome(q2p, v0, q02, vf, t0, tf);
%Cubic Polynomials Coefficients for TrajectoryPlanning q2p to q02
fcn2_1placetohome= VariableA2toPlacetohome(1);
fcn2_2placetohome= VariableA2toPlacetohome(2);
fcn2_3placetohome= VariableA2toPlacetohome(3);
fcn2_4placetohome = VariableA2toPlacetohome(4);

[qd3toPlacetohome,vd3toPlacetohome,ad3toPlacetohome,VariableA3toPlacetohome,steptime3toplacetohome,qd3_initialplacetohome,qd3_finalplacetohome] = TrajectoryPlanning3Placetohome(q3p, v0, q03, vf, t0, tf);
%Cubic Polynomials Coefficients for TrajectoryPlanning q3p to q03
fcn3_1placetohome= VariableA3toPlacetohome(1);
fcn3_2placetohome= VariableA3toPlacetohome(2);
fcn3_3placetohome= VariableA3toPlacetohome(3);
fcn3_4placetohome= VariableA3toPlacetohome(4);

% velocity for place to current target

[qd1toPlacetoPrevious,vd1toPlacetoPrevious,ad1toPlacetoPrevious,VariableA1toPlacetoPrevious,steptime1toplacetoPrevious,qd1_initialplacetoPrevious,qd1_finalplacetoPrevious] = TrajectoryPlanning1PlacetoPrevious(q1p, v0, q1, vf, t0, tf);
%Cubic Polynomials Coefficients for TrajectoryPlanning q1p to q1
fcn1_1placetoPrevious= VariableA1toPlacetoPrevious(1);
fcn1_2placetoPrevious= VariableA1toPlacetoPrevious(2);
fcn1_3placetoPrevious= VariableA1toPlacetoPrevious(3);
fcn1_4placetoPrevious= VariableA1toPlacetoPrevious(4);
% Open Simulink
syst = 'Robotic_Arm_For_SIMScape_2'; % Define syst as the simulink file
open(syst)
sim(syst)
pause(40);
%% Updated Home Configuration
q01 = q1;
q02 = 60;
q03 = -50;
key = 0;

    sprintf('Process Finished')
else
      fprintf('Object is not Red');
      pause(0.5);
end
    end
%% For Green Object 
    for pos = 0.15:0.15:0.9
        %Scan for Green
    writePosition(BaseServo,pos);               % Set value to servo
    CurrentPosition = readPosition(BaseServo);   % Read the current servo angle
    indegree = (CurrentPosition/1)*180;         
    disp(indegree);                             % Read and display the current angle in degree
    pause(1);
    
    % get snapshot
    b = getsnapshot(vid);
    b = b(1:end,110:1210,:);                    % Crop the video width (220 to 1100) --> total width is 880 pixels
    
    % Call a function to calculate the area for each color plane
    [BlueArea1,RedArea1,GreenArea1] = MyFunction(b); 
    
    % Will determine what is the object color
     if BlueArea1 > RedArea1 && BlueArea1 > GreenArea1
         targetcolor = 'Blue1';
     elseif RedArea1 > BlueArea1 && RedArea1 > GreenArea1 
         targetcolor = 'Red12';
     else
         targetcolor = 'Green';
     end

if targetcolor == 'Green'
    fprintf('Target Color is Green\n\n')
    % Set the frame
    set(vid,'FramesPerTrigger',inf);        % 
    set(vid,'ReturnedColorspace','rgb')
    vid.FrameGrabInterval=5;                % Extract screenshot from video every 5 millisecond
    start(vid);
while(vid.FramesAcquired<=500)
    % Preview of stream image frames
    b = getsnapshot(vid);                         % Declare variable data to store the extracted frame
    diff_im = imsubtract(b(:,:,2),rgb2gray(b));       % Subtract the green component from the grayscale image
    diff_im = medfilt2(diff_im,[3,3]);                % 2D Median filtering
    diff_im = imbinarize(diff_im,0.05);               % Grayscale to binary image, the value can be adjusted to make sure the algorithm more robust, didnt change easily if there is exposure difference
    BW2 = bwareafilt(diff_im,1);                % to extract objects from binary image by size (target image, keep n largest objects in the image)
    stats = regionprops(BW2,'BoundingBox','Centroid');  % Image blob analysis, we get set properties for each labelled region
    imshow(b);                                  % Image is shown
    hold on
    for object=1:length(stats)                  % Loop for rectangular bounding boxes
        bb = stats(object).BoundingBox;
        bc = stats(object).Centroid;
     % Rectangle function is used to draw rectangle with data from variables bb and bc
        rectangle('Position',bb,'EdgeColor','g','Linewidth',2.5)
        plot(bc(1),bc(2),'-m+')
        % Write the coordinate of the centroid on the screen
        c=text(bc(1)+15,bc(2), strcat(' X: ', num2str(round(bc(1))), ' Y: ', num2str(round(bc(2)))));
        set(c, 'FontName', 'Calibri', 'FontWeight', 'bold', 'FontSize', 10, 'Color', 'white');
    end
    
    %% Align the object centroid to camera centroid (Green Object)
      centerx = regionprops(BW2,'Centroid');
    centerx = (centerx.Centroid(1,1));                      % obtain the centroid of x value
    center2x = (1280 - centerx);                            % 1280 pixels is the Camera width (X direction)  
    center3x = (640 - center2x);                            % 640 is the center point of the camera width (X direction)
    if (center3x > 0)   % When object centroid is at the right side of the mid camera frame
        Rcentx = round(centerx);
        if Rcentx >= 639 && Rcentx <= 641
            fprintf('At the center already');
            disp(Rcentx)
            disp(posz)
            break
        end
        posz = readPosition(BaseServo);
        posz = posz - 0.01 - (abs(rand()))/50; % random number is introduced for decrement or increment servo position ... 
        writePosition(BaseServo,posz);        % since the servo is limited to min increment/decrement of 1.8 degree
        disp(Rcentx)
        disp(posz)
        pause(0.3); 
    elseif (center3x < 0) % When object centroid is at the left side of the mid camera frame
        Rcentx = round(centerx); 
        if Rcentx >= 639 && Rcentx <= 641
            fprintf('At the center already');
            disp(Rcentx)
            disp(posz)
            break
        end
        posz = readPosition(BaseServo);
        posz = posz + 0.01 + (abs(rand()))/50;
        writePosition(BaseServo,posz); 
        pause(0.3);
    end  
    hold off  
end
%% Measure the distance using ultrasonic Sensor (Green Object)
    pause(2);
    Total_distance_cm = 0;
    for time = 1:1:25
        pause(0.25);
        distance = readDistance(ultrasonicObject);
        distance_cm = distance*100;
        fprintf('Current distance is %.4f cm\n',distance_cm)
        Total_distance_cm = distance_cm + Total_distance_cm;
    end
    AverageDistance_cm = Total_distance_cm/25;
    pause(0.5);
    
    stop(vid),
    flushdata(vid);
    % clear all;
    
    %%  Finding x,y,z Coordinates using basic math and sine rule
    
    % First, Find the x and y coordinates, for z is already fixed (27.63mm) 
    OrgToUSsensor = 7.5; % In cm, Origin to US Sensor Distance
    OrgToObj =  (AverageDistance_cm + OrgToUSsensor + 0.9)*10; % In mm, origin to target object distance (0.9 is the target object width(1.8) divided by 2)
    Angle_x_y = pi/2; % Angle between x and y coordinates
    Angle_x_OrgToObj = (pi*pos)-(pi/2); % But it can be minus, % Angle between x and OrgToObj (Servo Motor Angle)
    Angle_OrgToObj_y = pi - Angle_x_y - abs(Angle_x_OrgToObj); 
     
    % Since we know the hypotenuse (OrgToObj) and all the 3 angles , 
    % Sine Rule is used to calculate the X and Y Coordinates
    if Angle_x_OrgToObj == 0
        x = OrgToObj;
        y = 0;
        z = 27.63; % Z is constant, since it is the height of the target object to the ground
    else
        x =  (OrgToObj*sin(Angle_OrgToObj_y))/sin(Angle_x_y); % In mm
        y =  (OrgToObj*sin(Angle_x_OrgToObj))/sin(Angle_x_y); % In mm
        z = 27.63; % Z is constant, since it is the height of the target object to the ground
    end
    
    disp(x);
    disp(y);
    disp(z);
    %% Perform the Inverse kinematic and Pick and Place function
   
 % Home Configuration --> Configuration for rest position  
 % d1 = 75.51mm , a2 = 67.87 mm , a3 = 122.11 mm
  if key == 0
        q01 = q1; % q1 from previous configuration became the home configuration for this state
        q02 = 60;
        q03 = -50;
 elseif key == 1
        q01 = q1; % q1 from previous configuration became the home configuration for this state
        q02 = 60;
        q03 = -50;
 end 
 
   q04 = 140;  % 173 = right gripper (q4), 161 = Left Gripper(q5) For closed condition            
   q05 = 194;  % 140 = right gripper (q4), 194 = Left Gripper(q5) For max open condition
 % Calling forward kinematics function
 T__EndEffector = ForwardKinematics(q01, q02, q03); % End effector transformation was obtained for home configurations
 
 % Call the inverse kinematic functions
 [q1,q2,q3] = InverseKinematics(x,y,z);
 q1 = q1 - 5; 
 
 % Control the gripper
 % 173 = right gripper (q4), 161 = Left Gripper(q5) For closed condition
 % 140 = right gripper (q4), 194 = Left Gripper(q5) For max open condition
 q4 = 173;            
 q5 = 161;
 
% Call the trajectory Planning
t0 = 0;
tf = 10;
v0 = 0;
vf = 0;
[qd1,vd1,ad1,VariableA1,steptime1,qd1_initial,qd1_final] = TrajectoryPlanning(q01, v0, q1, vf, t0, tf); % for Joint 1
% Cubic Polynomials Coefficients for TrajectoryPlanning q01 to q1
 fcn11 = VariableA1(1);
 fcn12 = VariableA1(2);
 fcn13 = VariableA1(3);
 fcn14 = VariableA1(4);

[qd2,vd2,ad2,VariableA2,steptime2,qd2_initial,qd2_final] = TrajectoryPlanning2(q02, v0, q2, vf, t0, tf); % For Joint 2
 %Cubic Polynomials Coefficients for TrajectoryPlanning q02 to q2
 fcn21 = VariableA2(1);
 fcn22 = VariableA2(2);
 fcn23 = VariableA2(3);
 fcn24 = VariableA2(4);

[qd3,vd3,ad3,VariableA3,steptime3,qd3_initial,qd3_final] = TrajectoryPlanning3(q03, v0, q3, vf, t0, tf); % for Joint 3
 %Cubic Polynomials Coefficients for TrajectoryPlanning q03 to q3
 fcn31 = VariableA3(1);
 fcn32 = VariableA3(2);
 fcn33 = VariableA3(3);
 fcn34 = VariableA3(4);

[qd2home,vd2home,ad2home,VariableA2home,steptime2tohome,qd2_initialhome,qd2_finalhome] = TrajectoryPlanning2tohome(q02, v0, q2, vf, t0, tf);
 %Cubic Polynomials Coefficients for TrajectoryPlanning q2 to q02
 fcn21home = VariableA2home(1);
 fcn22home = VariableA2home(2);
 fcn23home = VariableA2home(3);
 fcn24home = VariableA2home(4);

 [qd3home,vd3home,ad3home,VariableA3home,steptime3tohome,qd3_initialhome,qd3_finalhome] = TrajectoryPlanning3tohome(q03, v0, q3, vf, t0, tf);
%Cubic Polynomials Coefficients for TrajectoryPlanning q3 to q03
 fcn31home = VariableA3home(1);
 fcn32home = VariableA3home(2);
 fcn33home = VariableA3home(3);
 fcn34home = VariableA3home(4);
% Place Configuration
 q1p = 88.2;
 q2p = 35;
 q3p = -75;
 %q04 = 140;  % 173 = right gripper (q4), 161 = Left Gripper(q5) For closed condition            
 %q05 = 194;  % 140 = right gripper (q4), 194 = Left Gripper(q5) For max open condition 
 
[qd1toPlace,vd1toPlace,ad1toPlace,VariableA1toPlace,steptime1toplace,qd1_initialplace,qd1_finalplace] = TrajectoryPlanning1toPlace(q1p, v0, q1, vf, t0, tf);
%Cubic Polynomials Coefficients for TrajectoryPlanning q1 to q1p 
 fcn1_1place= VariableA1toPlace(1);
 fcn1_2place= VariableA1toPlace(2);
 fcn1_3place= VariableA1toPlace(3);
 fcn1_4place = VariableA1toPlace(4);

[qd2toPlace,vd2toPlace,ad2toPlace,VariableA2toPlace,steptime2toplace,qd2_initialplace,qd2_finalplace] = TrajectoryPlanning2toPlace(q2p, v0, q02, vf, t0, tf);
%Cubic Polynomials Coefficients for TrajectoryPlanning q02 to q2p
fcn2_1place= VariableA2toPlace(1);
 fcn2_2place= VariableA2toPlace(2);
 fcn2_3place= VariableA2toPlace(3);
 fcn2_4place = VariableA2toPlace(4);

[qd3toPlace,vd3toPlace,ad3toPlace,VariableA3toPlace,steptime3toplace,qd3_initialplace,qd3_finalplace] = TrajectoryPlanning3toPlace(q3p, v0, q03, vf, t0, tf);
%Cubic Polynomials Coefficients for TrajectoryPlanning q03 to q3p
fcn3_1place= VariableA3toPlace(1);
fcn3_2place= VariableA3toPlace(2);
fcn3_3place= VariableA3toPlace(3);
fcn3_4place = VariableA3toPlace(4);

[qd2toPlacetohome,vd2toPlacetohome,ad2toPlacetohome,VariableA2toPlacetohome,steptime2toplacetohome,qd2_initialplacetohome,qd2_finalplacetohome] = TrajectoryPlanning2Placetohome(q2p, v0, q02, vf, t0, tf);
%Cubic Polynomials Coefficients for TrajectoryPlanning q2p to q02
fcn2_1placetohome= VariableA2toPlacetohome(1);
fcn2_2placetohome= VariableA2toPlacetohome(2);
fcn2_3placetohome= VariableA2toPlacetohome(3);
fcn2_4placetohome = VariableA2toPlacetohome(4);

[qd3toPlacetohome,vd3toPlacetohome,ad3toPlacetohome,VariableA3toPlacetohome,steptime3toplacetohome,qd3_initialplacetohome,qd3_finalplacetohome] = TrajectoryPlanning3Placetohome(q3p, v0, q03, vf, t0, tf);
%Cubic Polynomials Coefficients for TrajectoryPlanning q3p to q03
fcn3_1placetohome= VariableA3toPlacetohome(1);
fcn3_2placetohome= VariableA3toPlacetohome(2);
fcn3_3placetohome= VariableA3toPlacetohome(3);
fcn3_4placetohome= VariableA3toPlacetohome(4);

% velocity for place to current target

[qd1toPlacetoPrevious,vd1toPlacetoPrevious,ad1toPlacetoPrevious,VariableA1toPlacetoPrevious,steptime1toplacetoPrevious,qd1_initialplacetoPrevious,qd1_finalplacetoPrevious] = TrajectoryPlanning1PlacetoPrevious(q1p, v0, q1, vf, t0, tf);
%Cubic Polynomials Coefficients for TrajectoryPlanning q1p to q1
fcn1_1placetoPrevious= VariableA1toPlacetoPrevious(1);
fcn1_2placetoPrevious= VariableA1toPlacetoPrevious(2);
fcn1_3placetoPrevious= VariableA1toPlacetoPrevious(3);
fcn1_4placetoPrevious= VariableA1toPlacetoPrevious(4);
% Open Simulink
syst = 'Robotic_Arm_For_SIMScape_2'; % Define syst as the simulink file
open(syst)
sim(syst)
pause(40);
%% Updated Home Configuration
q01 = q1;
q02 = 60;
q03 = -50;
key = 1;
    sprintf('Process Finished')
    else
      fprintf('Object is not Green');
      pause(0.5);
    end
end
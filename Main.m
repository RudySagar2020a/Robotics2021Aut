%% Main

%% Clear+Prepare Setup

close all
set(0,'DefaultFigureWindowStyle','docked')
clear
clc
clf

%% Setup Background Environment

workspace = [-1 1 -1 1 0 1];
% scale = 0;
 
% [xTL,BL;xTR,xBR],[yTL,yBL;yTR,yBR],[zTL,zBL;zTR,zBR]
 
% X-Wall (XZ)
surf([-4,-4;4,4],[4,4;4,4],[0,4;0,4],...
    'CData',imread('concrete2.jpg'),'FaceColor','texturemap');

hold on
 
% Y-wall (YZ)
surf([4,4;4,4],[-4,-4;4,4],[0,4;0,4],...
    'CData',imread('concrete2.jpg'),'FaceColor','texturemap');
 
% Floor(Z) (XY)
surf([-4,-4;4,4],[-4,4;-4,4],[0,0;0,0],...
    'CData',imread('concrete2.jpg'),'FaceColor','texturemap');

%% Setup Items in Environment

% ----------------------------------
% UR3 Reach = 500mm (0.5m)
% UR5 Reach = 850mm (0.85m)
% ----------------------------------

% EDIT ROBOT ARM TRANSLATION HERE---

UR3Loc = transl(2.0,-0.75,0.005); %TRANSLATE FOR UR3 Arm
%NOTE: POSE for LINEAR UR5 changed @LinearUR5 (line 62)

% 3D PLOT ROBOT ARM IN WORKSPACE---

UR3Arm = UR3(workspace, 'UR3Arm', UR3Loc);
UR5Arm = LinearUR5(false);

% EDIT ITEM TRANSLATION HERE---

B1  = transl(2.5,  -1.0,  0.005);              %EDIT TRANSLATE FOR Brick1
B2  = transl(2.45, -0.75, 0.005)*trotz(pi/2);  %EDIT TRANSLATE FOR Brick2
B3  = transl(1.65, -1.0,  0.005);              %EDIT TRANSLATE FOR Brick3
B4  = transl(1.8,  -0.75, 0.005)*trotz(pi/2);  %EDIT TRANSLATE FOR Brick4
B5  = transl(2.2,  -0.9,  0.005);              %EDIT TRANSLATE FOR Brick5
B6  = transl(2.8,  1.0,   0.005);              %EDIT TRANSLATE FOR Brick6
B7  = transl(2.5,  0.4,   0.005)*trotz(pi/2);  %EDIT TRANSLATE FOR Brick7
B8  = transl(1.5,  0.9,   0.005)*trotz(pi/2);  %EDIT TRANSLATE FOR Brick8
B9  = transl(1.35, 0.25,  0.005);              %EDIT TRANSLATE FOR Brick9

% 3D PLOT ITEMS IN WORKSPACE---

Brick1 = Item(workspace,'Brick','1', B1);
Brick2 = Item(workspace,'Brick','2', B2);
Brick3 = Item(workspace,'Brick','3', B3);
Brick4 = Item(workspace,'Brick','4', B4);
Brick5 = Item(workspace,'Brick','5', B5);
Brick6 = Item(workspace,'Brick','6', B6);
Brick7 = Item(workspace,'Brick','7', B7);
Brick8 = Item(workspace,'Brick','8', B8);
Brick9 = Item(workspace,'Brick','9', B9);

% fence1 = Item(workspace,'fence','11',transl(-1.0, 2.65, 1.1));
% fence2 = Item(workspace,'fence','12',transl(-1.0, 0.5, 1.1));
% fence3 = Item(workspace,'fence','13',transl(-1.0,-2.50, 1.1));
% 
% eStop  = Item(workspace, 'eStopLP',   '15', transl(-2.0, 3.9, 1.0));
% fireExt= Item(workspace, 'fireExtLP', '16', transl(-3.0, 3.75,0.15));

%% Point Cloud - Max Reach and Max Volume

% [pointCloud] = UR3.PointCloud();

%% Pickup and Drop-off Location (Pose & Location)

% GoalPose for Brick Wall
goalPose = transl(2.0, -0.25, 0.005);

% You need to be able to provide evidence (log, command window, text on the
% GUI or figure) that your robot really does get the end-effector to the 
% desired pickup location. SHOULD BE WITHIN 10-20MM (=0.01)
endEffectorCheck = transl(0.0, 0.0, -0.1);

% Initialise Movement class to perform Movement functions
move = Move();

% Main Movement - Pick and Place of Bricks

% disp('Motion: Searching for Brick and Picking Up');
% Move(UR3Arm, Brick1.model.base, []);
% 
% disp('Motion: Moving Brick to Drop-Off Location');
% Move(UR3Arm, dropOff, [Brick1]);

disp('Motion: Searching for Brick and Picking Up');
% B1End = B1 * endEffectorCheck;
move.moveBrick(UR3Arm, Brick1, goalPose);

disp('Motion: Moving Brick to Drop-Off Location');
move.moveBrick(UR3Arm, Brick1, goalPose * trotx(pi));

disp('Brick Wall built! Project Complete!');

%% MAIN TESTING CODE

% -- SIMULATE OBJECTS IN ENVIRONMENT (works) -----------------------------

% [faceData,vertexData,data] = plyread('Brick.ply','tri');
% 
% % Scale the colours to be 0-to-1 (they are originally 0-to-255
% vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
%  
% % Then plot the trisurf
% tableMesh_h = trisurf(faceData,vertexData(:,1),vertexData(:,2),vertexData(:,3) ...
%     ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');

% -- TESTING UR3 JOINTS AS ITEMS BEFORE APPLYING TO UR3.m (works) --------
% joint0 = Item(workspace, 'UR3Joint0', '0', transl(1.5,-0.75,0.01)*trotz(pi));
% joint1 = Item(workspace, 'UR3Joint1', '1', transl(1.5,-0.75,0.01));
% joint2 = Item(workspace, 'UR3Joint2', '2', transl(1.5,-0.75,0.01));
% joint3 = Item(workspace, 'UR3Joint3', '3', transl(1.5,-0.75,0.01));
% joint4 = Item(workspace, 'UR3Joint4', '4', transl(1.5,-0.75,0.01));
% joint5 = Item(workspace, 'UR3Joint5', '5', transl(1.5,-0.75,0.01));
% joint6 = Item(workspace, 'UR3Joint6', '6', transl(1.5,-0.75,0.01));


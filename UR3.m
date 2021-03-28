%% -- NOTE --
% This M file was copied from UR5.m file form Peter Corke's 
% Robotic Toolbox as a guide for UR3 component of Assignment

% DH Parameters have been changed + Gripper Function was removed

%% 
classdef UR3 < handle
    properties %constant property
        %> Robot model
        model;
        location;
        workspace = [-1 1 -1 1 0.1 1];
        plyData;
        startPoseJoints;
        maxReachRadius;
        maxVolume;
        pointCloud;
    end
    
    methods %% Static Class for UR3 robot simulation

        function self = UR3(workspace,modelName,location)
            self.workspace = workspace;
            self.model.base = location;
            self.location = location;
            self.GetUR3Robot(modelName,workspace);
            self.PlotAndColourRobot();
            self.startPoseJoints = zeros(1,6);
        end

%% GetUR3Robot
% Given a name (optional), create and return a UR3 robot model

function GetUR3Robot(self, modelName, workspace)
    pause(0.001);
    workspace = [-1 1 -1 1 0 1];
    
% UR3 DH parameters found in Peter Corke's Github as mdl_ur3.m file: 
% https://github.com/petercorke/robotics-toolbox-matlab/blob/master/models/mdl_ur3.m
% UR3 Joint Limits:
% https://www.manualslib.com/manual/1235383/Universal-Robots-Ur3-Cb3.html?page=96

    % CHANGE DH PARAMETERS
    L1 = Link('d',0.1519, 'a',0,       'alpha',pi/2, 'offset',0);
    L2 = Link('d',0,      'a',-0.24365,'alpha',0,    'offset',-pi/2);
    L3 = Link('d',0,      'a',-0.21325,'alpha',0,    'offset',0);
    L4 = Link('d',0.11235,'a',0,       'alpha',pi/2, 'offset',-pi/2);
    L5 = Link('d',0.08535,'a',0,       'alpha',-pi/2,'offset',0);
    L6 = Link('d',0.0819, 'a',0,       'alpha',0,    'offset',0);

    L1.qlim = [deg2rad(-180),deg2rad(180)];
    L2.qlim = [deg2rad(-180),deg2rad(180)];
    L3.qlim = [deg2rad(-180),deg2rad(180)];
    L4.qlim = [deg2rad(-180),deg2rad(180)];
    L5.qlim = [deg2rad(-180),deg2rad(180)];
    L6.qlim = [deg2rad(-360),deg2rad(360)];
        
    self.model = SerialLink([L1 L2 L3 L4 L5 L6],'name', modelName);
    
    % ADDED FOR THE ASSIGNMENT, RETURN BACK TO NOTHING ONCE COMPLETE
    % CHANGED POSE FOR A1 BELOW - transl(-X, Z, Y):
    % NEW CHANGE: (multiply base by location as you've already preset
    % location parameter and passing it to Get function when calling
    % in main. Now you can change location of base in Main.
    
    % self.model.base = self.model.base * transl(2.5,-2.0,0.01);
     self.model.base = self.model.base * self.location;
%      q = zeros(1,6);
%      self.model.plot(q,'workspace', workspace);
end

%% PlotAndColourRobot
% Given a robot index, add the glyphs (vertices and faces) and
% colour them in if data is available 
         function PlotAndColourRobot(self)%robot,workspace)
            for linkIndex = 0:self.model.n
                [faceData, vertexData, plyData{linkIndex + 1}] = plyread ...
                    (['UR3Joint',num2str(linkIndex),'.ply'],'tri');
                self.model.faces{linkIndex+1} = faceData;
                self.model.points{linkIndex+1} = vertexData;
            end

%           Display robot
            self.model.plot3d(zeros(1,self.model.n),'noarrow','workspace',self.workspace);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end  
            self.model.delay = 0;

%           Try to correctly colour the arm (if colours are in ply file data)
            for linkIndex = 0:self.model.n
                handles = findobj('Tag', self.model.name);
                h = get(handles,'UserData');
                try 
                    h.link(linkIndex+1).Children.FaceVertexCData = ...
                        [plyData{linkIndex+1}.vertex.red ...
                       , plyData{linkIndex+1}.vertex.green ...
                       , plyData{linkIndex+1}.vertex.blue]/255;
                    h.link(linkIndex+1).Children.FaceColor = 'interp';
                catch ME_1
                    disp(ME_1);
                    continue;
                end
            end
         end

%% Point Cloud

        function [pointCloud] = PointCloud(self, stepCount)
% Lab 3 Point Cloud
% 2.4 Sample the joint angles within the joint limits at 'step-Count' degree increments between each of the joint limits
% & 2.5 Use fkine to determine the point in space for each of these poses, so that you end up with a big list of points
        stepRads = deg2rad(stepCount);
        qlim = self.model.qlim;
        % Don't need to worry about joint 6
        pointCloudeSize = prod(floor((qlim(1:5,2)-qlim(1:5,1))/stepRads + 1));
        pointCloud = zeros(pointCloudeSize,3);
        counter = 1;
        tic
            for q1 = qlim(1,1):stepRads:qlim(1,2)
                for q2 = qlim(2,1):stepRads:qlim(2,2)        
                    for q3 = qlim(3,1):stepRads:qlim(3,2)            
                        for q4 = qlim(4,1):stepRads:qlim(4,2)                
                            for q5 = qlim(5,1):stepRads:qlim(5,2)                    
                                % Don't need to worry about joint 6, just assume it=0
                                    q6 = 0;%                     
                                for q6 = qlim(6,1):stepRads:qlim(6,2)                        
                                    q = [q1,q2,q3,q4,q5,q6];                        
                                    tr = self.model.fkine(q);                                                
                                    pointCloud(counter,:) = tr(1:3,4)';                        
                                    counter = counter + 1;                         
                                    if mod(counter/pointCloudeSize * 100,1) == 0                            
                                        disp(['After ',num2str(toc),' seconds, completed ',num2str(counter/pointCloudeSize * 100),'% of poses']);                        
                                    end%                     
                                end
                            end
                        end
                    end
                end
            end
        end
        
        % Convhull function to calculate volume of pointCloud
        % https://www.mathworks.com/help/matlab/ref/convhull.html#bspql3e-1

        function av = MaxVolume(self)
            [k,av] = convhull(self.pointCloud);
            self.maxVolume = av;
        end

%% Second Approach

%         function [sideRadius, topRadius, volume] = PointCloud(self)
%         % Lab 3 Point Cloud
%         % 2.4 Sample the joint angles within the joint limits at 'step-Count' degree increments between each of the joint limits
%         % & 2.5 Use fkine to determine the point in space for each of these poses, so that you end up with a big list of points
%         stepRads = deg2rad(50);
%         qlim = self.UR3.model.qlim;
%         % Don't need to worry about joint 6
%         pointCloudeSize = prod(floor((qlim(1:5,2)-qlim(1:5,1))/stepRads + 1));
%         self.pointCloud = zeros(pointCloudeSize,3);
%         counter = 1;
%         tic
%             for q1 = qlim(1,1):stepRads:qlim(1,2)
%                 for q2 = qlim(2,1):stepRads:qlim(2,2)        
%                     for q3 = qlim(3,1):stepRads:qlim(3,2)            
%                         for q4 = qlim(4,1):stepRads:qlim(4,2)                
%                             for q5 = qlim(5,1):stepRads:qlim(5,2)                    
%                                 % Don't need to worry about joint 6, just assume it=0
%                                     q6 = 0;%                     
%                                 for q6 = qlim(6,1):stepRads:qlim(6,2)                        
%                                     q = [q1,q2,q3,q4,q5,q6];                        
%                                     tr = self.UR3.model.fkine(q);                                                
%                                     self.pointCloud(counter,:) = tr(1:3,4)';                        
%                                     counter = counter + 1;                         
%                                     if mod(counter/pointCloudeSize * 100,1) == 0                            
%                                         disp(['After ',num2str(toc),' seconds, completed ',num2str(counter/pointCloudeSize * 100),'% of poses']);                        
%                                     end%                     
%                                 end
%                             end
%                         end
%                     end
%                 end
%             end
%         disp('max side radius is: ');
%         sideRadius = max([abs(self.pointCloud(:,1)),abs(self.pointCloud(:,2))]);
%         disp('max top radius is: ');
%         topRadius  = max([abs(self.pointCloud(:,3))]);
%         disp('max volume is: ');
%         volume     = (4 * pi * ([sideRadius, topRadius])^3)/3;
%         end
    end
end
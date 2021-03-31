classdef Movement < handle
    %MOVEMENT Class for moving Robot Arm, pose and matrix

    % !!!!!!!!!!!!!
    % ORIGINAL movement CLASS NOT USED AYNMORE---
    % !!!!!!!!!!!!!
    
    % referenced CowHerd class & Lab4 Solutions file for jtraj function
    % fkine used to find joint-config needed for end-effector
    % to move towards brickPose
    % jtraj plots 'waypoint(1)' trajectory using 'ikcon(2)' to avoid collision
    % between robot arms(1), robot arm-floor(1) and self-collision(2)
% waypoints guide below:
% https://www.mathworks.com/matlabcentral/fileexchange/71130-trajectory-planning-for-robot-manipulators    
    
properties
        initPose;
        brickPose;
%       steps = 50;
    end
    
    methods

        % Copy this Item Class format for Movement function
%         function self = Item(workspace, modelName, modelNumber, location)
%             self.plotAndColour(workspace, modelName, modelNumber, location)
%         end

        function self = Movement()
        end
        
        % Setup Arm to move towards Bricks location
        function moveRobotArm(self, robot, goalPose, Item)
%             iQ = Item.model.getpos;     % current bricks pose
%             TiQ = Item.model.fkine(iQ); % translate for end-effector joint config.

            iQ = fkine(Item.model.getpos);
            currentQ = robot.model.getpos; % current end-effectors pose
            newQ = robot.model.ikcon(transl(Item.iQ), currentQ);
            
            qMatrix = jtraj(currentQ, newQ, 50);
            for i = 1:size(qMatrix, 1)
                robot.model.animate(robot.model, qMatrix(i,:));
                drawnow();
            end
            pause(0.01)
            
            currentQ = robot.model.getpos;
            newQ = robot.model.ikcon(transl(Item.iQ), currentQ);
            
            qMatrix = jtraj(newQ, goalPose, 50);
            for i = 1:size(qMatrix, 1)
                robot.model.animate(robot.model, qMatrix(i,:));
                drawnow();
            end
            pause(0.01)
        end
            
            
%             trajectory = jtraj(initPose, robot.model.getpos(), 50);
%             
%             for trajectoryStep = 1:size(trajectory)
%                 jointTrajectory = trajectory(trajectoryStep,:);
%                 robot.model.animate(jointTrajectory);
%             end
%             pause(0.01)
%         end
        
        % Once Robot Arm has picked up Brick, move to dropoff location
%         function moveRobotAndBrick(self, robot, item, goalPose)
%             initPose = robot.model.ikcon(robot.model.getpos(), goalPose);
%             newTrajectory = jtraj(initPose, robot.model.getpos(), 50);
%             
%             for trajectoryStep = 1:size(newTrajectory)
%                 jointTrajectory = newTrajectory(trajectoryStep,:);
%                 robot.model.animate(jointTrajectory);
%                 
%                 endEffectorPose = robot.model.fkine(jointTrajectory);
%                 item.model.base = endEffectorPose * trotx(pi);
%                 item.model.animate(0);
%             end
%             pause(0.01)
%         end
        
    end
end

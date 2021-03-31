classdef Move < handle
    
    % !!!!!!!!!!!!!
    % SECONDARY movement CLASS NOT USED AYNMORE---
    % !!!!!!!!!!!!!
    
    properties
        
    end
    methods
        
        function self = Move()
        end
        
        function moveBrick(self, robot, Item, goalPose)
            itemQ = Item.model.base
            currentQ = robot.model.getpos
            goalQ = robot.model.ikine(Item.model.base, ([20 20 20 20 20 20]));
% newQ = robot.model.ikine(transl(Item.TQ), currentQ);

            qMatrix = jtraj(currentQ, goalQ, 50);
            for i = 1:size(qMatrix, 1)
                robot.model.animate(qMatrix(i,:));
                pause(0.1);
            end
            pause(0.01)
            
%             iQ = Item.model.getpos;
%             TiQ = fkine(iQ);
%             currentQ = robot.model.getpos;
%             newQ = robot.model.ikine(transl(Item.TiQ), currentQ);
%             
%             qMatrix = jtraj(newQ, goalPose, 50);
%             for i = 1:size(qMatrix, 1)
%                 robot.animate(robot.model, qMatrix(i,:));
%                 drawnow();
%             end
%             pause(0.01)
            
        end
        
    end
    
end










% function [RobotArmGoalPose,Item] = Move(RobotArm,GoalPose,Item)
% 
% zoffset = -0.1;
% 
% % animate 1
% % initQ = RobotArm.model.getpos .* [1,0,0,0,0,0];
% 
% goalQ = RobotArm.model.ikcon(GoalPose, RobotArm.model.getpos);
% 
% jointTrajectory = jtraj(RobotArm.model.getpos(), goalQ, 50);
% 
% for trajStep = 1:size(jointTrajectory,1)
%     q = jointTrajectory(trajStep,:);
%     RobotArm.model.animate(q);
%     if isempty(Item) == 0
%         newBase = RobotArm.model.fkine(q);
%     end
%     for i = 1:size(Item,1)
%         Item(i).model.base = newBase;
%         Item(i).model.animate(0);
%     end
%     pause(0.01);
% end
% pause(0.01);
% 
% if size(GoalPose,2) == 6
%     return
% else
%     goalQ = RobotArm.model.ikcon(GoalPose * trotx(pi),RobotArm.model.getpos);
% end
% 
% jointTrajectory = jtraj(RobotArm.model.getpos(), goalQ,30);
% 
% for trajStep = 1:size(jointTrajectory,1)
%     q = jointTrajectory(trajStep,:);
%     RobotArm.model.animate(q);
%     if isempty(Item) == 0
%         newBase = RobotArm.model.fkine(q);
%     end
%     
%     for i = 1:size(Item,1)
%         Item(i).model.base = newBase;
%         Item(i).model.animate(0);
%     end
%     pause(0.01);
% end
% pause(0.01);
% 
% RobotArmGoalPose = RobotArm.model.fkine(RobotArm.model.getpos);
% end
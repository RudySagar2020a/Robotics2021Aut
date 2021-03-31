% Function to take end-effector to brick pose, pickup brick and place at
% goalpose location

function [] = MoveFunction(robot, Item, goalPose)
    
%     goalPose = Item.model.location;
    itemT = Item.model.base
    currentQ = robot.model.getpos
    goalQ = robot.model.ikine(itemT, currentQ);
    % newQ = robot.model.ikine(transl(Item.TQ), currentQ);
    T = robot.model.fkine(goalQ)
    itemT
    
    qMatrix = jtraj(currentQ, goalQ, 50);
    for i = 1:size(qMatrix, 1)
        robot.model.animate(qMatrix(i,:));
        pause(0.1);
    end
    pause(0.01)
    
    itemT = Item.model.base;
    currentQ = robot.model.getpos;
    goalQ = robot.model.ikine(itemT, currentQ);
    newQ = robot.model.ikine(itemT, goalQ);
    
    qMatrix = jtraj(goalQ, newQ, 50);
    for i = 1:size(qMatrix, 1)
        q = qMatrix(i,:);
        robot.model.animate(q);
        newBase = robot.model.fkine(q);
        Item.model.base = newBase;
        Item.model.animate(0);
        pause(0.1);
    end
    pause(0.01)
    
end
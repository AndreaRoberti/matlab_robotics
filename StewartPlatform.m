classdef StewartPlatform < handle
    properties
        sim

        motors
        tips
        targets
        base
        tip
        target
        initTipPose
        initTipMatrix
     
        scriptHandle
    end

    methods
        function obj = StewartPlatform(client)
            obj.sim = client.require('sim');

            obj.motors = cell(1, 6);
            obj.tips = cell(1, 5);
            obj.targets = cell(1, 5);
            for i = 1:6
                obj.motors{i} = obj.sim.getObject(['./motor', num2str(i)]);
            end
            for i = 1:5
                obj.tips{i} = obj.sim.getObject(['./downArm', num2str(i), 'Tip']);
                obj.targets{i} = obj.sim.getObject(['./downArm', num2str(i), 'Target']);
            end
            obj.base = obj.sim.getObject('./stewartPlatform');
            obj.tip = obj.sim.getObject('./tip');
            obj.target = obj.sim.getObject('./target');

            obj.scriptHandle = obj.sim.getScript(obj.sim.scripttype_childscript,obj.base);

            obj.initTipPose = obj.sim.getObjectPose(obj.tip, obj.base);
            % quat2tform ([q,x,y,z])
            obj.initTipMatrix = quat2tform([obj.initTipPose{7},obj.initTipPose{4},obj.initTipPose{5},obj.initTipPose{6}]);
            obj.initTipMatrix(1:3,4) = [obj.initTipPose{1}, obj.initTipPose{2}, obj.initTipPose{3}];
            % disp(obj.initTipMatrix);


        end

        function obj = exampleIK(obj, t)
            new_pose_matrix = obj.initTipMatrix;
            new_pose_matrix(3,4) = obj.initTipMatrix(3,4) + 0.1 * sin(t);

            pose_to_send_quat = tform2quat(new_pose_matrix);
            pose_to_send_pos = tform2trvec(new_pose_matrix);

            pose_to_send = [pose_to_send_pos, pose_to_send_quat(2),pose_to_send_quat(3),pose_to_send_quat(4),pose_to_send_quat(1)];

            obj.sim.callScriptFunction('remoteAPI_IK',obj.scriptHandle,pose_to_send)

        end
    end

end

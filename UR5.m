classdef UR5 < IRobotArm
    methods
        function obj = UR5(sim)   
            obj = obj@IRobotArm(sim, 6,{'joint1','joint2','joint3','joint4','joint5','joint6'});
        end
    end
end
classdef Panda < IRobotArm
    methods
        function obj = Panda(sim)   
            obj = obj@IRobotArm(sim, 7,{'Franka_joint1','Franka_joint2','Franka_joint3','Franka_joint4','Franka_joint5','Franka_joint6','Franka_joint6'});
        end
    end
end
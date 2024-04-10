classdef IRobotArm < handle
    properties(Access = protected)
        sim
        NR_OF_SLAVES

        joint_handle

        last_joint_position
    end

    properties(Access = public)
        out_joint_positions

        in_positions_cmd

    end

    methods
        function obj = IRobotArm(sim, jointNum, jointNames)
            obj.NR_OF_SLAVES = jointNum;

            obj.sim = sim;

            obj.joint_handle(1:obj.NR_OF_SLAVES) = -1;

            obj.out_joint_positions(1:obj.NR_OF_SLAVES) = 0.0;

            obj.in_positions_cmd(1:obj.NR_OF_SLAVES) = 0.0;

            for i = 0:obj.NR_OF_SLAVES-1

                obj.joint_handle(i+1) = obj.sim.getObject(strcat (':/',num2str(cell2mat(jointNames(i+1)))));
            end

        end


        function obj = readJointState(obj)
            for i = 1:obj.NR_OF_SLAVES
                obj.out_joint_positions(i) =  obj.sim.getJointPosition(obj.joint_handle(i));
            end
        end

        function obj = setJointSetPoints(obj)
            for i = 1:obj.NR_OF_SLAVES
                obj.sim.setJointTargetPosition(obj.joint_handle(i),obj.in_positions_cmd(i));
            end
        end

        function obj = setJointTargetPosition(obj,in_position)
            for i = 1:obj.NR_OF_SLAVES
                obj.sim.setJointTargetPosition(obj.joint_handle(i),in_position(i));
            end
        end

        function obj = configure(obj)

        end

        function obj = start(obj)

        end

        function obj = update(obj)
            readJointState(obj);
            setJointSetpoints(obj);
        end

        function obj = stop(obj)

        end

    end
end
addpath("coppelia\");

client = RemoteAPIClient();
sim = client.require('sim');

% https://github.com/CoppeliaRobotics/zmqRemoteApi/tree/master/clients/matlab
% https://manual.coppeliarobotics.com/en/simulation.htm#stepped
sim.setStepping(true)

target = sim.getObject('/Target'); % exists inside the scene

ur5 = UR5(sim);

sim.startSimulation();

% EXAMPLE : INITIAL POSE
position = sim.getObjectPosition(target,-1);
orientation = sim.getObjectQuaternion(target,-1);


while true
    t = sim.getSimulationTime();
    if t >= 15; break; end
    fprintf('Simulation time: %.2f [s]\n', t);

    
    % move the target handle 
    % SAME ORIENTATION
    % OFFSET IN Z (0.2)
    sim.setObjectPosition(target,[position{1},position{2}, position{3}-0.2],-1)
    sim.setObjectQuaternion(target,orientation,-1)
    
    % move to zeroPosition
    % ur5.setJointTargetPosition([0,0,0,0,0,0,0]);

    sim.step();
end
sim.stopSimulation();
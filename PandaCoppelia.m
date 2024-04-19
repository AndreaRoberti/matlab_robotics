addpath("coppelia\");

client = RemoteAPIClient();
sim = client.require('sim');

% https://github.com/CoppeliaRobotics/zmqRemoteApi/tree/master/clients/matlab
% https://manual.coppeliarobotics.com/en/simulation.htm#stepped
sim.setStepping(true)

target = sim.getObject('/Target'); % exists inside the scene

panda = Panda(sim);

sim.startSimulation();

while true
    t = sim.getSimulationTime();
    if t >= 15; break; end
    fprintf('Simulation time: %.2f [s]\n', t);
    position = sim.getObjectPosition(target,-1);
    
    % move the target handle
    sim.setObjectPosition(target,[0.4,0,0.4],-1)
    sim.setObjectQuaternion(target,[-1.0,0,0.0,0.0],-1)
    
    % move to zeroPosition
    % panda.setJointTargetPosition([0,0,0,0,0,0,0]);

    % move joints using finalPosition, generated from peter corke code (PandaRobot.m)
    % panda.setJointTargetPosition(finalPosition);

    sim.step();
end
sim.stopSimulation();
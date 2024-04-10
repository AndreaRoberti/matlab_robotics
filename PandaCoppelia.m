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

    panda.setJointTargetPosition([0,0,0,0,0,0,0]);
    % panda.setJointTargetPosition(finalPosition);

    sim.step();
end
sim.stopSimulation();
addpath("coppelia\");

client = RemoteAPIClient();
sim = client.require('sim');

% https://github.com/CoppeliaRobotics/zmqRemoteApi/tree/master/clients/matlab
% https://manual.coppeliarobotics.com/en/simulation.htm#stepped
sim.setStepping(true)


stewart_platform = StewartPlatform(client);


sim.startSimulation();

while true
    t = sim.getSimulationTime();
    if t >= 15; break; end
    fprintf('Simulation time: %.2f [s]\n', t);

    stewart_platform.exampleIK(t);

    sim.step();
end
sim.stopSimulation();
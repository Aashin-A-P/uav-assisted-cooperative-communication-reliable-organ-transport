function train_dqn_agent()

clc; clear;

env = make_drl_env(45*60);

obsInfo = getObservationInfo(env);
actInfo = getActionInfo(env);

numObs = obsInfo.Dimension(1);   % should be 16 now
numAct = numel(actInfo.Elements);

layers = [
    featureInputLayer(numObs,'Normalization','none','Name','state')
    fullyConnectedLayer(256,'Name','fc1')
    reluLayer('Name','relu1')
    fullyConnectedLayer(256,'Name','fc2')
    reluLayer('Name','relu2')
    fullyConnectedLayer(numAct,'Name','q_output')
];

lgraph = layerGraph(layers);

repOpts = rlRepresentationOptions( ...
    'LearnRate',1e-3, ...
    'GradientThreshold',1);

critic = rlQValueRepresentation( ...
    lgraph, ...
    obsInfo, ...
    actInfo, ...
    'Observation',{'state'}, ...
    repOpts);

agentOpts = rlDQNAgentOptions;
agentOpts.UseDoubleDQN = true;
agentOpts.MiniBatchSize = 256;
agentOpts.ExperienceBufferLength = 1e5;
agentOpts.DiscountFactor = 0.99;
agentOpts.TargetUpdateFrequency = 4;

agentOpts.EpsilonGreedyExploration.Epsilon = 1.0;
agentOpts.EpsilonGreedyExploration.EpsilonMin = 0.02;
agentOpts.EpsilonGreedyExploration.EpsilonDecay = 2e-4;

agent = rlDQNAgent(critic, agentOpts);

trainOpts = rlTrainingOptions;
trainOpts.MaxEpisodes = 600;                 % generalized + curriculum needs more
trainOpts.MaxStepsPerEpisode = 400;
trainOpts.ScoreAveragingWindowLength = 30;
trainOpts.Plots = "training-progress";
trainOpts.Verbose = true;

trainingStats = train(agent, env, trainOpts);

save("trained_dqn_agent.mat","agent","trainingStats");

disp("Training Complete ✔");

end
function train_ppo_agent()

env = make_drl_env_ppo(45*60);

obsInfo = getObservationInfo(env);
actInfo = getActionInfo(env);

numObs = obsInfo.Dimension(1);
numAct = numel(actInfo.Elements);

%% ================= Actor =================
actorNet = [
    featureInputLayer(numObs,'Normalization','none','Name','obs')
    fullyConnectedLayer(256)
    reluLayer
    fullyConnectedLayer(256)
    reluLayer
    fullyConnectedLayer(numAct)
    softmaxLayer
];

actor = rlDiscreteCategoricalActor( ...
    actorNet, ...
    obsInfo, ...
    actInfo, ...
    ObservationInputNames="obs");

%% ================= Critic =================
criticNet = [
    featureInputLayer(numObs,'Normalization','none','Name','obs')
    fullyConnectedLayer(256)
    reluLayer
    fullyConnectedLayer(256)
    reluLayer
    fullyConnectedLayer(1)
];

critic = rlValueFunction( ...
    criticNet, ...
    obsInfo, ...
    ObservationInputNames="obs");

%% ================= PPO Options =================
agentOpts = rlPPOAgentOptions;
agentOpts.ExperienceHorizon = 1024;
agentOpts.ClipFactor = 0.2;
agentOpts.EntropyLossWeight = 0.01;
agentOpts.MiniBatchSize = 256;
agentOpts.NumEpoch = 4;
agentOpts.DiscountFactor = 0.99;
agentOpts.GAEFactor = 0.95;

agent = rlPPOAgent(actor, critic, agentOpts);

%% ================= Training =================
trainOpts = rlTrainingOptions;
trainOpts.MaxEpisodes = 900;
trainOpts.MaxStepsPerEpisode = 350;
trainOpts.ScoreAveragingWindowLength = 25;
trainOpts.Plots = "training-progress";
trainOpts.Verbose = true;

trainingStats = train(agent, env, trainOpts);

save("trained_ppo_agent.mat","agent","trainingStats");

end
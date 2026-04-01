function drl_nav_train_agent()

env = drl_nav_make_env(45*60);

obsInfo = getObservationInfo(env);
actInfo = getActionInfo(env);

numObs = obsInfo.Dimension(1);
numAct = numel(actInfo.Elements);

actorNet = [
    featureInputLayer(numObs,Normalization="none",Name="obs")
    fullyConnectedLayer(256)
    reluLayer
    fullyConnectedLayer(256)
    reluLayer
    fullyConnectedLayer(numAct)
    softmaxLayer
];

actor = rlDiscreteCategoricalActor(actorNet, obsInfo, actInfo, ...
    ObservationInputNames="obs");

criticNet = [
    featureInputLayer(numObs,Normalization="none",Name="obs")
    fullyConnectedLayer(256)
    reluLayer
    fullyConnectedLayer(256)
    reluLayer
    fullyConnectedLayer(1)
];

critic = rlValueFunction(criticNet, obsInfo, ...
    ObservationInputNames="obs");

agentOpts = rlPPOAgentOptions;
agentOpts.ExperienceHorizon = 2048;
agentOpts.ClipFactor = 0.2;
agentOpts.EntropyLossWeight = 0.015;
agentOpts.MiniBatchSize = 512;
agentOpts.NumEpoch = 6;
agentOpts.DiscountFactor = 0.998;
agentOpts.GAEFactor = 0.97;

agentOpts.ActorOptimizerOptions.LearnRate  = 3e-4;
agentOpts.CriticOptimizerOptions.LearnRate = 1e-3;

agent = rlPPOAgent(actor, critic, agentOpts);

trainOpts = rlTrainingOptions;
trainOpts.MaxEpisodes = 1200;
trainOpts.MaxStepsPerEpisode = 350;
trainOpts.ScoreAveragingWindowLength = 40;
trainOpts.Plots = "training-progress";
trainOpts.Verbose = true;

trainingStats = train(agent, env, trainOpts);

save("drl_nav_trained_agent.mat","agent","trainingStats");

end
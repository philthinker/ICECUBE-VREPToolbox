function [hyperparams] = sample_IBPparam(F,hyperparams,hyperhyperparams)

a_gamma=hyperhyperparams.a_gamma;
b_gamma=hyperhyperparams.b_gamma;
harmonic = hyperhyperparams.harmonic;

Kplus = sum(sum(F,1)>0);
gamma0 = randgamma(a_gamma + Kplus) / (b_gamma + harmonic);

hyperparams.gamma0 = gamma0;
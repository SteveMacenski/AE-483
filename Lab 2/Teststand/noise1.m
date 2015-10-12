function out=noise1(x)

%%% Sample noise
out=x+normrnd(0,.004,size(x));
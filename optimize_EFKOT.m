function [lamm, err_gm] = optimize_EFKOT(ytr, ignore, Fs)

% --- Hybrid GA + fmincon options ---
fminuncOptions = optimoptions(@fmincon, ...
    'Display','iter', ...
    'UseParallel','always', ...
    'Algorithm','active-set');

ga_opts = gaoptimset( ...
    'TolFun',1e-12, ...
    'StallGenLimit',20, ...
    'Generations',50, ...
    'Display','iter', ...
    'UseParallel','always', ...
    'HybridFcn',{@fmincon, fminuncOptions});


% --- Objective ---
h = @(X) GA_EKFOT(X, ytr, ignore, Fs);
nvars = 5;

% --- Bounds ---
LB = [1e-7  1e-8  1e-7*ones(1,1)  1e-7*ones(1,1)  1e-5];
UB = [ inf   inf   inf*ones(1,1)  inf*ones(1,1)  inf];

err_gm = nan;

% --- Run GA until valid result ---
while isnan(err_gm)
    [lamm, err_gm] = ga(h, nvars, [], [], [], [], LB, UB, [], [], ga_opts);
end

% --- Re-run if any parameter equals zero ---
if any(lamm == 0)
    [lamm, err_gm] = ga(h, nvars, [], [], [], [], LB, UB, [], [], ga_opts);
end

end
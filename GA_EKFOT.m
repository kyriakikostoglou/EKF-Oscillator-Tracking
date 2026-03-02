function [Jmean, Jmat] = GA_EKFOT(X, ytr, ignore, Fs)
%GA_EKFOT  Objective wrapper for GA across trials/channels.
%
% ytr expected shape: [M x T x nTrials]
% Returns:
%   Jmat  : [M x nTrials] per-signal costs
%   Jmean : scalar mean cost

[M, ~, nTr] = size(ytr);
Jmat = nan(M, nTr);

for j = 1:nTr
    for m = 1:M
        y = squeeze(ytr(m,:,j));
        Jmat(m,j) = EKFOT(X, y, ignore, Fs);
    end
end

Jmean = mean(Jmat(:), 'omitnan');
end
function polf = run_EKFOT(sig_in, lam, ignore, Fs)
%RUN_EKFOT  EKF oscillator tracking wrapper (pmax=1, pmax2=0)
%
% Inputs:
%   sig_in : [M x T] signal to be analysed (channels x time)
%   LAM    : optimal lambda value(s) from GA
%
% Output:
%   polf   : stacked output
%            [A_est (M x T);
%             f_est (M x T);
%             ptr   (M x T)]
%
% Requires in workspace or as globals: Fs, ignore, SIM_KF_ARMA28



[M, T] = size(sig_in);

% ---- Preallocate ----
A_out   = nan(M, T);
f_out   = nan(M, T);
ptr_out = nan(M, T);

for m = 1:M
    yy = double(sig_in(m,:));  % 1 x T

    % Pass GA-optimized LAM to the EKF simulator
    [thm, ~, ~, ptrr] = SIM_EKFOT(lam, yy, ignore, Fs);

    % Frequency (rad/s → Hz)
    omega_est = thm(3, :);
    f_est     = abs(omega_est) / (2*pi);

    % Amplitude proxy from oscillator states
    x1    = thm(1, :);
    x2    = thm(2, :);
    A_est = sqrt(x1.^2 + x2.^2);

    % Store
    A_out(m,:)   = A_est;
    f_out(m,:)   = f_est;
    ptr_out(m,:) = ptrr;
end

% ---- Final stacked output ----
polf = cat(1, A_out, f_out, ptr_out);

end
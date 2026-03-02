function [thm, e, J, ptr] = SIM_EKFOT(X, Y, ignore, Fs)
%SIM_EKFOT_CLEAN  Run EKF oscillator tracking and return trajectories.
%
% Outputs
%   thm : [4 x N] posterior state estimates z(k)
%   e   : [1 x N] innovations
%   J   : scalar cost (same as EKFOT_clean)
%   ptr : [1 x N] Frobenius norm of P(k) as a rough uncertainty proxy

Ts = 1/Fs;

% --- Continuous -> discrete scaling ---
R2_ct = X(1);
Q_ct  = X(2);
R2    = R2_ct * Ts;              % measurement noise variance per sample
R1    = Q_ct  * Ts * eye(4);     % process noise covariance per sample

omega0 = X(3);                   % [rad/s]
alpha0 = X(4);                   % [1/s]
pin    = X(end);                 % P0 scale

% --- Init ---
Y   = double(Y(:)');             % ensure row
N   = numel(Y);

z   = [0; 0; omega0; alpha0];
P   = pin * eye(4);
H   = [1 0 0 0];

e   = zeros(1, N);
ptr = zeros(1, N);
thm = zeros(4, N);

for k = 3:N
    % --- record posterior ---
    thm(:,k) = z;

    % --- predict ---
    a11    =  2*exp(-z(4)*Ts)*cos(z(3)*Ts);
    a12    = -exp(-2*z(4)*Ts);
    A2     = [a11, a12; 1, 0];

    x_pred = A2 * z(1:2);
    z_pred = [x_pred; z(3); z(4)];

    % --- linearize ---
    F          = eye(4);
    F(1:2,1:2) = A2;

    F(1,3) = -2*exp(-z(4)*Ts)*sin(z(3)*Ts)*Ts * z(1);
    F(1,4) = -2*Ts*exp(-z(4)*Ts)*cos(z(3)*Ts)*z(1) ...
             + 2*Ts*exp(-2*z(4)*Ts)*z(2);

    P_pred = F*P*F' + R1;

    % --- update ---
    y_pred = H * z_pred;
    inov   = Y(k) - y_pred;

    S = H*P_pred*H' + R2;        % scalar
    K = (P_pred*H') / S;         % 4x1

    z = z_pred + K*inov;
    P = (eye(4)-K*H)*P_pred;

    % --- wrap omega (discrete angle wrapping) ---
    z(3) = wrapToPiLocal(z(3)*Ts)/Ts;

    e(k)   = inov;
    ptr(k) = norm(P, 'fro');
    thm(:,k) = z;                % store updated posterior
end

Up = e(ignore:end);
J  = norm(Up) / norm(Y(ignore:end));

end


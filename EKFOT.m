function [J, Up] = EKFOT(X, Y, ignore, Fs)
%EKFOT_CLEAN  EKF oscillator tracking cost for GA optimization.
%
%   [J, Up] = EKFOT_clean(X, Y, ignore, Fs)
%
% Inputs
%   X      : parameter vector [R2_ct, Q_ct, omega0, alpha0, P0scale]
%            R2_ct  : continuous-time measurement noise intensity
%            Q_ct   : continuous-time process noise intensity (scalar, shared)
%            omega0 : initial angular frequency [rad/s]
%            alpha0 : initial damping [1/s]
%            P0scale: scale factor for initial covariance
%   Y      : signal (1 x N) or (N x 1)
%   ignore : number of initial samples to ignore in cost
%   Fs     : sampling rate [Hz]
%
% Method
%   AR(2) resonator with time-varying omega, alpha estimated by EKF:
%     a1 =  2*exp(-alpha*Ts)*cos(omega*Ts)
%     a2 = -exp(-2*alpha*Ts)
%   State z = [x(k); x(k-1); omega; alpha]
%   y(k) = x(k) + v(k)
%
% Cost
%   J = ||innovation(ignore:end)|| / ||Y(ignore:end)||

% --- Continuous -> discrete scaling ---
Ts = 1/Fs;
R2_ct = X(1);
Q_ct  = X(2);
R2    = R2_ct * Ts;             % measurement noise variance per sample
R1    = Q_ct  * Ts * eye(4);    % process noise covariance per sample

omega = X(3);                   % [rad/s] (despite old comment "rad/sample")
alpha = X(4);                   % [1/s]
pin   = X(end);                 % P0 scale

% --- Init ---
z = [0; 0; omega; alpha];       % [x(k); x(k-1); omega; alpha]
P = pin * eye(4);
H = [1 0 0 0];

N = numel(Y);
Y = double(Y(:)');              % ensure row
e = zeros(1, N);

for k = 3:N
    % --- Predict ---
    a11    =  2*exp(-z(4)*Ts)*cos(z(3)*Ts);
    a12    = -exp(-2*z(4)*Ts);
    A2     = [a11, a12; 1, 0];

    x_pred = A2 * z(1:2);
    z_pred = [x_pred; z(3); z(4)];

    % --- Linearize (Jacobian) ---
    F          = eye(4);
    F(1:2,1:2) = A2;

    F(1,3) = -2*exp(-z(4)*Ts)*sin(z(3)*Ts)*Ts * z(1);
    F(1,4) = -2*Ts*exp(-z(4)*Ts)*cos(z(3)*Ts)*z(1) ...
             + 2*Ts*exp(-2*z(4)*Ts)*z(2);

    P_pred = F*P*F' + R1;

    % --- Update ---
    y_pred = H * z_pred;
    inov   = Y(k) - y_pred;

    S = H*P_pred*H' + R2;        % scalar
    K = (P_pred*H') / S;         % 4x1

    z = z_pred + K*inov;
    P = (eye(4)-K*H)*P_pred;

    % --- Wrap omega (discrete-time angle wrapping) ---
    z(3) = wrapToPiLocal(z(3)*Ts)/Ts;

    e(k) = inov;
end

Up = e(ignore:end);
J  = norm(Up) / norm(Y(ignore:end));

end

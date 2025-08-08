function dxdt = observer_augmented_ode(t, x_aug,A, B, C, K, L)
    n = length(A);
    x = x_aug(1:n);           % True state
    x_hat = x_aug(n+1:end);   % Estimated state
    % Output and estimate
    y = C * x;
    y_hat = C * x_hat;

    % Control input using estimated state
    u =  K * x_hat;

    % True system
    dx = A * x + B * u;

    % Observer
    dx_hat = A * x_hat + B * u + L * (y - y_hat);

    % Combined derivative
    dxdt = [dx; dx_hat];
end

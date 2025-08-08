function dxdt = nonlinear_dynamics(t, x, K, M, m, c, g, x_eq)
    x1=x(1); x2=x(2); x3=x(3); x4=x(4); x5=x(5); x6=x(6);

    % 1. Calculate linearized state for the controller
    x_lin = x - x_eq;
    
    % 2. Calculate linearized control input u_lin = K*x_lin
    u_lin = K * x_lin;
    
    % 3. Convert u_lin to nonlinear physical forces Fs and Fl
    Fs = u_lin(1);
    Fl = u_lin(2) -m*g; % Isolate Fl
    
    % 4. Calculate derivatives using nonlinear system equations
    dxdt = zeros(6,1);
    dxdt(1) = x4;
    dxdt(2) = x5;
    dxdt(3) = x6;
    dxdt(4) = (Fs-c*x4+Fl*sin(x3)+m*x4*sin(x3)^2+2*m*x5*x6*sin(x3)+m*x2*x6^2*sin(x3))/(-m*cos(x3)^2+M+m);
    dxdt(5) = (m*x2*x6^2+Fl+g*m*cos(x3)+m*x4*sin(x3))/m;
    den6 = x2*(-m*cos(x3)^2+M+m);
    num6 = Fs*cos(x3)-c*x4*cos(x3)-g*m*sin(x3)+Fl*cos(x3)*sin(x3)-2*M*x5*x6-2*m*x5*x6-M*g*sin(x3)+g*m*cos(x3)^2*sin(x3)+m*x4*cos(x3)*sin(x3)^2+2*m*x5*x6*cos(x3)^2+2*m*x2*x6^2*cos(x3)*sin(x3);
    dxdt(6) = num6 / den6;
end
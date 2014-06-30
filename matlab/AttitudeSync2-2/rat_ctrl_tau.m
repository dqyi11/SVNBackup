function tau = rat_ctrl_tau(in)

    sigma        = in(1:3);
    sigma_dot    = in(4:6);
    sigma_ddot   = in(7:9);
    sigma_r      = in(10:12);
    sigma_r_dot  = in(13:15);
    sigma_r_ddot = in(16:18);
    
    %J = [1 0 0;0 1 0; 0 0 1];
    J = [1 0.1 0.1; 0.1 0.1 0.1; 0.1 0.1 0.9];
    S_sigma = [0 -sigma(3) sigma(2); sigma(3) 0 -sigma(1); -sigma(2) sigma(1) 0];
    S_sigma_dot = [0 -sigma_dot(3) sigma_dot(2); sigma_dot(3) 0 -sigma_dot(1); - sigma_dot(2) sigma_dot(1) 0];
    F = 0.5* (0.5*(1 - sigma'*sigma)*eye(3) + S_sigma + sigma*sigma');
    invF = inv(F);
    dot_F = 0.5 * ((-2 * sigma_dot'*sigma)*eye(3)+ S_sigma_dot + sigma * sigma_dot' + sigma_dot * sigma');
    H = invF' * J * invF;
    X = J * invF * sigma_dot;
    S_X = [0 -X(3) X(2); X(3) 0 -X(1);-X(2) X(1) 0];
    C = - invF' * J * invF * dot_F * invF - invF' * S_X * invF;
    
    sigma_t = sigma - sigma_r;
    sigma_t_dot = sigma_dot - sigma_r_dot;
    %sigma_t_ddot = sigma_ddot - sigma_r_ddot;
    
    %sigma_d = sigma_r;
    sigma_d_dot = sigma_r_dot;
    sigma_d_ddot = sigma_r_ddot;
       
    Lambda = eye(3);
    K = 2 * eye(3);
    
    tau = F' *( H*(sigma_d_ddot - Lambda * sigma_t_dot) + C*(sigma_d_dot - Lambda * sigma_t) - K*(sigma_t_dot + Lambda * sigma_t) );
    
end
function tau = csa_ctrl_tau(in)
    y = in(1:3);
    sigma = in(4:6);
    sigma_r_delta = in(7:9);
    sigma_net_delta = in(10:12);
    
    S_sigma = [0 -sigma(3) sigma(2); sigma(3) 0 -sigma(1); -sigma(2) sigma(1) 0];
    F = 0.5*((1-sigma'*sigma)*0.5*eye(3) + S_sigma + sigma * sigma');
       
    tau = - F' *(sigma_net_delta + sigma_r_delta + 10* y); 
    %tau = - F' *(sigma_r_delta + y);
end
function y = csa_ctrl_y(in)
    estX = in(1:3);
    sigma = in(4:6);
    sigma_net_delta = in(7:9);

    P = 2 * eye(3);
    Gamma = - eye(3);
    k = 1;
    
    %y = P * Gamma * estX + k * P * sigma + P * sigma_net_delta;  
    y = P * Gamma * estX + k * P * sigma;
end

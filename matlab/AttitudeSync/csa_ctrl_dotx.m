function dotEstX =csa_ctrl_dotx(in)
    estX = in(1:3);
    sigma = in(4:6);
    sigma_net_delta = in(7:9);

    Gamma = - eye(3);
    k = 1;
    
    dotEstX = Gamma * estX + k * sigma + sigma_net_delta;
    %dotEstX = Gamma * estX + k * sigma
end

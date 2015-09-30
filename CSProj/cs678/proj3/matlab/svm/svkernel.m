function k = svkernel(ker, u, v)
    p1 = 1;
    ru = size(u,1);
    cu = size(u,2);
    rv = size(v,1);
    cv = size(v,2);
    switch lower(ker)
        case 'linear'
            k = u*v';
        case 'poly'
            k = (u*v' + 1)^p1;
        case 'rbf'
            k = exp(-(u-v)*(u-v)'/(2*p1^2));  
    end

end
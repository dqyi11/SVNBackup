function k = svkernel2(ker, u, v)
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
            k = zeros(ru, rv);
            for i=1:1:ru
                for j=1:1:rv
                    delta = u(i,:)-v(j,:);
                    k(i,j) = exp(-delta * delta'/(2*pi^2));
                end
            end  
    end

end
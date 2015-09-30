function y = Kernel(ker, x1, x2)

p1 = 1;

switch lower(ker)
    case 'linear'
        y = x1 * x2';
    case 'poly'
        y = (x1 * x2' + 1)^p1;
    case 'rbf'
        y = exp(-(x1-x2)*(x1-x2)'/(2*p1^2));
end
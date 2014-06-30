stable = []
unstable = []
for chi = 0.0:0.01:1.0 
    for phi = 0.0:0.1:15.0
        if ( phi > 0 & phi <= ((1+chi)-2*sqrt(chi))/2 ) | ( phi > ((1+chi)+2*sqrt(chi))/chi & phi < 2*(1+chi)/chi ) 
            stable = [stable [chi; phi; val]]
        else
            unstable = [unstable [chi; phi; val]]
        end
        
    end
end

scatter(stable(1,:), stable(2,:),  'rs')
hold on
scatter(unstable(1,:), unstable(2,:),  'bs')
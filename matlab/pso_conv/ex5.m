x = 0.001:0.01:1.0 
phi = []
for x = 0.001:0.01:1.0
    
    phi = [phi 2*(1+x)/x]
end
figure(1)
plot(chi,phi)
ylim([0,15])


gb_bound = 3.0
pb_bound = 3.0

phi1 = 2
phi2 = 2
inertia = 0.8

pos = (rand() - 0.5) * 20
vel = 0.0

P = [4.51000000000000,0;0,2.17000000000000]
Q = [0.234800000000000,-0.0259199999999997;-0.0259199999999997,0.704768000000000]


pbHist = []
gbHist = []
posHist = []
bdHist = []

boundA = calcBoundA(phi1+phi2, inertia)
boundB = calcBoundB(phi1, phi2, inertia)
bound_scale = calcBound(P, Q, boundA , boundB)

    
for i = 1:1:1000
        
    pb = pb_bound * rand()
    gb = gb_bound * rand()
    
    bound = bound_scale * norm([pb, gb])

    u1 = rand()
    u2 = rand()
    localForce = phi1 * u1 * (pb - pos)
    globalForce = phi2 * u2 * (gb - pos)

    vel = inertia * (vel + localForce + globalForce)
    pos = pos + vel
    
    pbHist = [pbHist pb]
    gbHist = [gbHist gb]
    posHist = [posHist pos]
    bdHist = [bdHist bound]
end

plot(1:1:1000, posHist, 'b')
hold on
%plot(1:1:1000, bdHist, 'r--')
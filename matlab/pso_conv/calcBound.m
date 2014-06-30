function bound = calcBound(P, Q, boundA, boundB)

minP = min(eig(P))
maxP = max(eig(P))

normP = norm(P, Inf)

minQ = min(eig(Q))

alpha_1 = minP
alpha_2 = maxP
alpha_3 = 0.5 * minQ
sigma = 2 * boundA^2 * normP^2 * boundB^2 / minQ^2 + normP * boundB^2

bound = alpha_2 * sigma / (alpha_1 * alpha_3)


